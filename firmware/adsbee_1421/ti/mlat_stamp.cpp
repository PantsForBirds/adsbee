#include "mlat_stamp.hh"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)
#include DeviceFamily_constructPath(driverlib/timer.h)
#include DeviceFamily_constructPath(driverlib/udma.h)
#include DeviceFamily_constructPath(inc/hw_event.h)
#include DeviceFamily_constructPath(inc/hw_gpio.h)
#include DeviceFamily_constructPath(inc/hw_gpt.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include <ti/drivers/Power.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/power/PowerCC26X2.h>

#include "cycle_counter.hh"

// uDMA channel 9 (GPT0A) control structures; the linker script places them in the driver's control table.
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT0APriControlTableEntry, (UDMA_CHAN_TIMER0_A + UDMA_PRI_SELECT));
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT0AAltControlTableEntry, (UDMA_CHAN_TIMER0_A + UDMA_ALT_SELECT));

namespace mlat_stamp {

static constexpr uint32_t kProbeIoid = IOID_13;  // LR2021 DIO5.
static constexpr uint16_t kHalf = 64;            // Ring half: one uDMA transfer.
static constexpr uint16_t kRingLen = 2 * kHalf;
static constexpr uint16_t kMaxPending = 32;
static constexpr int32_t kTsMaxAfter = 300 * 32;                // 32 MHz ticks: latest event after the request.
static constexpr uint32_t kStaleTicks = 100000 * kTicksPerUs;  // Events older than 100 ms are dropped.

Counters c;
uint32_t delay_min_ticks = 90 * kTicksPerUs;
uint32_t delay_max_ticks = 160 * kTicksPerUs;
int32_t agree_center = 0;
int32_t agree_tol = 0x7FFFFFFF;  // Wide open until calibrated.

static bool inited = false;
static uint8_t mode = 0;
static bool isr_capture = false;
static bool capture_on = false;
static HwiP_Struct hwi;
static bool hwi_made = false;
static volatile uint32_t ring[kRingLen];
static volatile uint16_t isr_head = 0;  // Interrupt capture: next write.
static uint16_t tail = 0;               // Next ring entry to read.
static uint64_t now64 = 0;              // 64-bit extension of the counter, advanced in Poll().
static uint32_t now24 = 0;
static uint64_t pending[kMaxPending];   // Edge times (ticks) not yet used or dropped, oldest first.
static uint16_t num_pending = 0;
static uint64_t boundary64 = 0;         // Latest time a FIFO read may have been sampled.

bool Inited() { return inited; }
uint32_t NowSlow() { return HWREG(GPT0_BASE + GPT_O_TAV) & kTickMask; }
uint8_t Mode() { return mode; }
bool IsrCapture() { return isr_capture; }
bool PinHigh() { return (HWREG(GPIO_BASE + GPIO_O_DIN31_0) >> kProbeIoid) & 1; }

static void Advance() {
    uint32_t now = NowSlow();
    now64 += (now - now24) & kTickMask;
    now24 = now;
}
static uint64_t To64(uint32_t ticks) { return now64 - ((now24 - ticks) & kTickMask); }

static void CaptureIsr(uintptr_t /*arg*/) {
    uint32_t start = CycleCounter::Now();
    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    ring[isr_head] = HWREG(GPT0_BASE + GPT_O_TAR);
    isr_head = static_cast<uint16_t>((isr_head + 1) & (kRingLen - 1));
    c.isr_edges = c.isr_edges + 1;
    uint32_t cycles = CycleCounter::Since(start);
    c.isr_cycles = c.isr_cycles + cycles;
    if (cycles > c.isr_max_cycles) c.isr_max_cycles = cycles;
}

static void DmaArm(uint32_t select, volatile uint32_t* dst) {
    uDMAChannelControlSet(UDMA0_BASE, UDMA_CHAN_TIMER0_A | select,
                          UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_1);
    uDMAChannelTransferSet(UDMA0_BASE, UDMA_CHAN_TIMER0_A | select, UDMA_MODE_PINGPONG,
                           reinterpret_cast<void*>(GPT0_BASE + GPT_O_TAR), const_cast<uint32_t*>(dst), kHalf);
}

static void DmaStart() {
    uDMAChannelDisable(UDMA0_BASE, UDMA_CHAN_TIMER0_A);
    HWREG(UDMA0_BASE + UDMA_O_CLEARCHNLPRIALT) = 1u << UDMA_CHAN_TIMER0_A;
    uDMAChannelAttributeDisable(UDMA0_BASE, UDMA_CHAN_TIMER0_A,
                                UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    DmaArm(UDMA_PRI_SELECT, ring);
    DmaArm(UDMA_ALT_SELECT, ring + kHalf);
    HWREG(EVENT_BASE + EVENT_O_UDMACH9SSEL) = EVENT_UDMACH9SSEL_EV_GPT0A_DMABREQ;
    HWREG(EVENT_BASE + EVENT_O_UDMACH9BSEL) = EVENT_UDMACH9BSEL_EV_GPT0A_DMABREQ;
    HWREG(GPT0_BASE + GPT_O_DMAEV) |= GPT_DMAEV_CAEDMAEN;
    uDMAChannelEnable(UDMA0_BASE, UDMA_CHAN_TIMER0_A);
    tail = 0;
}

static void DmaStop() {
    HWREG(GPT0_BASE + GPT_O_DMAEV) &= ~GPT_DMAEV_CAEDMAEN;
    uDMAChannelDisable(UDMA0_BASE, UDMA_CHAN_TIMER0_A);
}

// Ring index the uDMA writes next.
static uint16_t DmaHead() {
    for (int tries = 0; tries < 4; tries++) {
        uint32_t alt = (HWREG(UDMA0_BASE + UDMA_O_SETCHNLPRIALT) >> UDMA_CHAN_TIMER0_A) & 1;
        uint32_t ctrl = alt ? dmaGPT0AAltControlTableEntry.ui32Control : dmaGPT0APriControlTableEntry.ui32Control;
        uint32_t alt_again = (HWREG(UDMA0_BASE + UDMA_O_SETCHNLPRIALT) >> UDMA_CHAN_TIMER0_A) & 1;
        if (alt != alt_again) continue;
        uint32_t remaining = (ctrl & UDMA_MODE_M) == UDMA_MODE_STOP ? 0 : ((ctrl & UDMA_XFER_SIZE_M) >> 4) + 1;
        return static_cast<uint16_t>(((alt ? kHalf : 0) + (kHalf - remaining)) & (kRingLen - 1));
    }
    return tail;
}

static void Init() {
    if (inited) return;
    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);  // GPT0 needs the HF clock.
    Power_setDependency(PowerCC26XX_PERIPH_GPT0);
    Power_setDependency(PowerCC26XX_PERIPH_UDMA);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_TIMER0);
    PRCMLoadSet();
    IOCPortConfigureSet(kProbeIoid, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_INPUT | IOC_IOPULL_DOWN);
    TimerDisable(GPT0_BASE, TIMER_A);
    TimerConfigure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerPrescaleSet(GPT0_BASE, TIMER_A, 0xFF);  // Timer extension in capture mode: 24 bits.
    TimerLoadSet(GPT0_BASE, TIMER_A, 0xFFFF);
    TimerEventControl(GPT0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(GPT0_BASE, TIMER_A);
    inited = true;
}

static void CaptureStop() {
    if (!capture_on) return;
    TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
    DmaStop();
    capture_on = false;
}

static void CaptureStart() {
    CaptureStop();
    num_pending = 0;
    if (isr_capture) {
        if (!hwi_made) {
            HwiP_Params params;
            HwiP_Params_init(&params);
            params.priority = ~0u;  // Lowest, with the GPIO interrupts; the time is latched in hardware.
            HwiP_construct(&hwi, INT_GPT0A, CaptureIsr, &params);
            hwi_made = true;
        }
        isr_head = 0;
        tail = 0;
        TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
        TimerIntEnable(GPT0_BASE, TIMER_CAPA_EVENT);
    } else {
        DmaStart();
    }
    capture_on = true;
}

bool SetMode(uint8_t new_mode, bool use_isr) {
    if (new_mode == 0 && !inited) {
        mode = 0;
        return true;
    }
    Init();
    mode = 0;
    CaptureStop();
    isr_capture = use_isr;
    Advance();
    boundary64 = now64;
    if (new_mode & kModeEdge) CaptureStart();
    mode = new_mode;
    return true;
}

void ResetCounters() {
    uintptr_t key = HwiP_disable();
    c = Counters();
    HwiP_restore(key);
}

void Poll() {
    if (!inited) return;
    Advance();
    if (!capture_on) return;
    uint16_t head;
    if (isr_capture) {
        head = isr_head;
    } else {
        head = DmaHead();
        // Hand a finished half back. A stopped half that is the active one means both filled: restart.
        uint32_t alt = (HWREG(UDMA0_BASE + UDMA_O_SETCHNLPRIALT) >> UDMA_CHAN_TIMER0_A) & 1;
        bool pri_stopped = (dmaGPT0APriControlTableEntry.ui32Control & UDMA_MODE_M) == UDMA_MODE_STOP;
        bool alt_stopped = (dmaGPT0AAltControlTableEntry.ui32Control & UDMA_MODE_M) == UDMA_MODE_STOP;
        if ((pri_stopped && alt_stopped) || !uDMAChannelIsEnabled(UDMA0_BASE, UDMA_CHAN_TIMER0_A)) {
            c.dma_overruns++;
            DmaStart();
            num_pending = 0;
            boundary64 = now64;
            return;
        }
        if (alt && pri_stopped) {
            DmaArm(UDMA_PRI_SELECT, ring);
            c.dma_rearms++;
        } else if (!alt && alt_stopped) {
            DmaArm(UDMA_ALT_SELECT, ring + kHalf);
            c.dma_rearms++;
        }
    }
    while (tail != head) {
        uint64_t t = To64(ring[tail] & kTickMask);
        tail = static_cast<uint16_t>((tail + 1) & (kRingLen - 1));
        c.edges++;
        if (num_pending == kMaxPending) {  // Never in practice (one edge per clear): drop the oldest.
            for (uint16_t i = 1; i < kMaxPending; i++) pending[i - 1] = pending[i];
            num_pending--;
        }
        pending[num_pending++] = t;
    }
}

void Boundary() {
    if (!inited) return;
    Advance();
    boundary64 = now64;
}

DebugEntry dbg[kDbgLen];
uint16_t dbg_count = 0;

void PairSweep(const Sweep& sweep, uint16_t k, uint64_t* stamps) {
    Poll();
    if (dbg_count < kDbgLen && (k > 0 || num_pending > 0)) {
        uint64_t a0 = To64(sweep.level_a_ticks);
        DebugEntry& e = dbg[dbg_count++];
        e.k = static_cast<uint8_t>(k);
        e.pending = static_cast<uint8_t>(num_pending);
        e.ab = static_cast<uint16_t>((sweep.level_b_ticks - sweep.level_a_ticks) & kTickMask);
        e.age_a = num_pending ? static_cast<int32_t>(a0 - pending[0]) : 0;
        e.since_boundary = static_cast<int32_t>(a0 - boundary64);
        e.nss_a = static_cast<uint16_t>((sweep.level_a_ticks - sweep.ts_nss_ticks) & kTickMask);
        e.ts_value = sweep.has_ts ? sweep.ts_value : 0;
    }
    c.sweeps++;
    const uint64_t a = To64(sweep.level_a_ticks), b = To64(sweep.level_b_ticks);
    const uint8_t kfield = k > 14 ? 15 : static_cast<uint8_t>(k);
    for (uint16_t i = 0; i < k; i++) stamps[i] = kUnstamped | (static_cast<uint64_t>(kfield) << 56) | (b * 2);
    if (k == 1) c.reads1++;
    if (k == 2) c.reads2++;
    if (k >= 3) c.reads3p++;

    // An event at time t: its packet is complete in the FIFO between t + delay_min and t + delay_max.
    //   in an earlier read, possibly:  t + delay_min <= boundary (the last read's latest sample time)
    //   in this read, surely:          t + delay_max <= a, and not possibly in an earlier read
    //   not in this read, surely:      t + delay_min > b
    enum Where { kEarlier, kHere, kLater, kUnsure };
    auto where = [&](uint64_t t) {
        if (t + delay_min_ticks <= boundary64) return kEarlier;
        if (t + delay_max_ticks <= a) return kHere;
        if (t + delay_min_ticks > b) return kLater;
        return kUnsure;
    };

    // Delay bounds, for isolated frames: one pending edge, newer than the last read.
    if (num_pending == 1 && pending[0] > boundary64) {
        uint64_t t = pending[0];
        if (k == 0 && a > t && a - t < 1000 * kTicksPerUs) {
            if (a - t > c.delay_lo_ticks) c.delay_lo_ticks = static_cast<uint32_t>(a - t);
            if (b - t > c.delay_lo_b_ticks) c.delay_lo_b_ticks = static_cast<uint32_t>(b - t);
        }
        if (k == 1 && a > t) {
            if (b - t < c.delay_hi_ticks) c.delay_hi_ticks = static_cast<uint32_t>(b - t);
            if (a - t < c.delay_hi_a_ticks) c.delay_hi_a_ticks = static_cast<uint32_t>(a - t);
        }
    }

    // Edges: the oldest one that is surely here. Edges possibly in an earlier read are dropped.
    bool edge_here = false, edge_unsure = false;
    uint64_t edge_t = 0;
    uint16_t edges_here = 0, kept = 0;
    for (uint16_t i = 0; i < num_pending; i++) {
        uint64_t t = pending[i];
        Where w = (now64 - t > kStaleTicks) ? kEarlier : where(t);
        if (w == kHere) {
            if (!edge_here) edge_t = t;
            edge_here = true;
            edges_here++;
        } else if (w == kLater || w == kUnsure) {
            if (w == kUnsure) edge_unsure = true;
            if (k == 0 || w == kLater) {
                pending[kept++] = t;  // Not read yet (an empty read takes nothing), or surely later.
            }
        }
    }
    num_pending = kept;

    // Internal timestamp: the latest sync event before the request.
    bool ts_here = false, ts_unsure = false;
    uint64_t ts_t2 = 0;  // 1/96 us.
    // The value is signed: an event between the request's NSS edge and the chip latching the answer reads negative.
    const int32_t ts_value = static_cast<int32_t>(sweep.ts_value);
    if (sweep.has_ts && ts_value > -kTsMaxAfter && ts_value < 0x40000000) {
        // 32 MHz ticks to 96 MHz: x3.
        ts_t2 = static_cast<uint64_t>(static_cast<int64_t>(To64(sweep.ts_nss_ticks) * 2) - static_cast<int64_t>(ts_value) * 3);
        Where w = where(ts_t2 / 2);
        ts_here = w == kHere;
        ts_unsure = w == kUnsure;
    }

    if (k == 0) {
        if (edge_here) c.rej_orphan += edges_here;
        return;  // Nothing was read: the boundary stays.
    }
    if (edges_here > k) c.rej_extra++;

    // Which packet an event that is surely here belongs to. Edge: the first event after a clear, so the packet is
    // known only when it is alone. Timestamp: the latest event before the request, so it is the last packet when no
    // later packet can have completed before the level sample.
    const uint64_t nss = sweep.has_ts ? To64(sweep.ts_nss_ticks) : 0;
    const bool ts_is_last = sweep.has_ts && (k == 1 || b - nss < delay_min_ticks);
    bool edge_ok = edge_here && edges_here == 1 && k == 1;
    bool ts_ok = ts_here && ts_is_last;
    if (edge_ok && ts_ok) {
        int32_t diff = static_cast<int32_t>(static_cast<int64_t>(edge_t * 2) - static_cast<int64_t>(ts_t2));
        if (diff < c.agree_min) c.agree_min = diff;
        if (diff > c.agree_max) c.agree_max = diff;
        int32_t off = diff - agree_center;
        if (off > agree_tol || off < -agree_tol) {
            c.rej_disagree++;
            edge_ok = ts_ok = false;
        }
    }
    const uint64_t kf = static_cast<uint64_t>(kfield) << 56;
    if (edge_ok && ts_ok) {
        stamps[0] = (3ull << 60) | kf | (edge_t * 2);
        c.stamped_both++;
    } else if (edge_ok) {
        stamps[0] = (1ull << 60) | kf | (edge_t * 2);
        c.stamped_edge++;
    } else if (ts_ok) {
        stamps[k - 1] = (2ull << 60) | kf | ts_t2;
        c.stamped_ts++;
    }
    // Two packets, one event each: the edge's packet is the other one when the timestamp names the last.
    if (k == 2 && ts_ok && edge_here && edges_here == 1 && (mode & kModeEdge) &&
        edge_t * 2 + 2 * delay_min_ticks < ts_t2) {
        stamps[0] = (1ull << 60) | kf | (edge_t * 2);
        c.stamped_edge++;
    }
    for (uint16_t i = 0; i < k; i++) {
        if (stamps[i] & kUnstamped) {
            if (edge_unsure || ts_unsure) {
                c.un_ambiguous++;
            } else if (k > 1 && (edge_here || ts_here)) {
                c.un_multi++;
            } else {
                c.un_none++;
            }
        }
    }
    boundary64 = b;
}

static LogEntry log_buf[kLogLen];
static uint16_t log_count = 0;

void LogValid(uint32_t icao, uint16_t df, uint64_t stamp) {
    c.valid++;
    if (mode != 0 && (stamp >> 63) == 0) c.valid_stamped++;
    if (log_count < kLogLen) log_buf[log_count++] = {(icao & 0xFFFFFF) | (static_cast<uint32_t>(df) << 24), stamp};
}
uint16_t LogCount() { return log_count; }
const LogEntry& LogAt(uint16_t i) { return log_buf[i]; }
void LogClear() { log_count = 0; }

}  // namespace mlat_stamp

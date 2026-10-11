#include "mlat_stamp.hh"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)
#include DeviceFamily_constructPath(driverlib/timer.h)
#include DeviceFamily_constructPath(inc/hw_gpt.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include <ti/drivers/Power.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/power/PowerCC26X2.h>

#include "adsbee.hh"
#include "cycle_counter.hh"

namespace mlat_stamp {

static constexpr uint32_t kProbeIoid = IOID_13;  // LR2021 DIO5.
static constexpr uint16_t kRingLen = 128;        // Power of 2.
// SyncwordValid edge to the packet being complete in the FIFO: 112 data bits at 1 Mbit/s.
static constexpr uint32_t kPacketTicks = 112 * kTicksPerUs;
static constexpr uint32_t kMarginTicks = 20 * kTicksPerUs;
static constexpr uint32_t kMinSpacingTicks = 100 * kTicksPerUs;  // Packets start at least 120 us apart.
static constexpr uint32_t kStaleTicks = 100000 * kTicksPerUs;    // Edges older than 100 ms are never paired.

Counters c;

static bool inited = false;
static volatile uint8_t mode = 0;
static HwiP_Struct hwi;
static uint32_t ring[kRingLen];
static volatile uint16_t head = 0;  // Capture interrupt writes.
static volatile uint16_t tail = 0;  // Main loop reads.
static uint64_t now64 = 0;          // 64-bit extension of the counter, advanced in the main loop.
static uint32_t now24 = 0;

static void CaptureIsr(uintptr_t /*arg*/) {
    uint32_t start = CycleCounter::Now();
    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    uint32_t ticks = HWREG(GPT0_BASE + GPT_O_TAR) & kTickMask;
    c.edges = c.edges + 1;
    uint16_t next = static_cast<uint16_t>((head + 1) & (kRingLen - 1));
    if (next == tail) {
        c.ring_dropped = c.ring_dropped + 1;
    } else {
        ring[head] = ticks;
        head = next;
    }
    if (mode == 1) {
        adsbee.lr2021.HandleSyncEdge();
    }
    uint32_t cycles = CycleCounter::Since(start);
    c.isr_cycles = c.isr_cycles + cycles;
    if (cycles > c.isr_max_cycles) c.isr_max_cycles = cycles;
}

void Init() {
    if (inited) return;
    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);  // GPT0 needs the HF clock.
    Power_setDependency(PowerCC26XX_PERIPH_GPT0);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_TIMER0);
    PRCMLoadSet();
    IOCPortConfigureSet(kProbeIoid, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_INPUT | IOC_IOPULL_DOWN);
    TimerDisable(GPT0_BASE, TIMER_A);
    TimerConfigure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerPrescaleSet(GPT0_BASE, TIMER_A, 0xFF);  // Timer extension in capture mode: 24 bits.
    TimerLoadSet(GPT0_BASE, TIMER_A, 0xFFFF);
    TimerEventControl(GPT0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    HwiP_Params params;
    HwiP_Params_init(&params);
    // Same (lowest) priority as the GPIO interrupts that run the drain chain: the handler posts SPI frames under
    // the same rules. The timestamp is latched in hardware, so the priority does not affect it.
    params.priority = ~0u;
    HwiP_construct(&hwi, INT_GPT0A, CaptureIsr, &params);
    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(GPT0_BASE, TIMER_A);
    inited = true;
}

// 0 until Init(): GPT0 has no clock before that, and reading it would fault.
uint32_t Now() { return inited ? (HWREG(GPT0_BASE + GPT_O_TAV) & kTickMask) : 0; }
uint8_t Mode() { return mode; }

void Flush() {
    uintptr_t key = HwiP_disable();
    tail = head;
    HwiP_restore(key);
}

void ResetCounters() {
    uintptr_t key = HwiP_disable();
    c = Counters();
    HwiP_restore(key);
}

void SetMode(uint8_t new_mode) {
    Init();
    TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
    mode = new_mode;
    Flush();
    if (mode != 0) {
        TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
        TimerIntEnable(GPT0_BASE, TIMER_CAPA_EVENT);
    }
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

static inline uint32_t Age(uint32_t boundary, uint32_t ticks) { return (boundary - ticks) & kTickMask; }

void PairBatch(uint32_t boundary_ticks, bool exact_boundary, uint16_t num_packets, uint64_t* stamps) {
    for (uint16_t i = 0; i < num_packets; i++) stamps[i] = 0;
    if (mode == 0 || !inited) return;
    // Advance the 64-bit extension (called at least once per main loop pass, far below the wrap time).
    uint32_t now = Now();
    now64 += (now - now24) & kTickMask;
    now24 = now;
    uint64_t boundary64 = now64 - Age(now, boundary_ticks);

    // Snapshot the queue: units[] are edges, with runs closer than a packet length merged into one unit.
    static uint32_t unit_ticks[kRingLen];
    static bool unit_double[kRingLen];
    uint16_t num_units = 0, consumed_edges[kRingLen + 1];
    uint16_t h = head, t = tail, edges_seen = 0;
    consumed_edges[0] = 0;
    for (uint16_t i = t; i != h; i = static_cast<uint16_t>((i + 1) & (kRingLen - 1))) {
        uint32_t ticks = ring[i];
        edges_seen++;
        uint32_t age = Age(boundary_ticks, ticks);
        if (age > kTickMask / 2) break;  // Edge is after the boundary: belongs to a later batch.
        if (num_units > 0 && Age(ticks, unit_ticks[num_units - 1]) < kMinSpacingTicks) {
            unit_double[num_units - 1] = true;
            consumed_edges[num_units] = edges_seen;
            continue;
        }
        unit_ticks[num_units] = ticks;
        unit_double[num_units] = false;
        num_units++;
        consumed_edges[num_units] = edges_seen;
    }
    // Stale units at the front (older than any packet still in the FIFO could be) are orphans.
    uint16_t first = 0;
    while (first < num_units && Age(boundary_ticks, unit_ticks[first]) > kStaleTicks) first++;
    c.orphans += first;
    // surely: units whose packet must be complete at the boundary. maybe: units whose packet may be complete.
    uint16_t surely = 0, maybe = 0;
    for (uint16_t i = first; i < num_units; i++) {
        uint32_t age = Age(boundary_ticks, unit_ticks[i]);
        if (age >= kPacketTicks + kMarginTicks) surely++;
        if (age + kMarginTicks >= kPacketTicks) maybe++;
    }
    bool ok = num_packets > 0 && (exact_boundary ? (surely <= num_packets && num_packets <= maybe) : (num_packets <= maybe));
    uint16_t take;  // Units to remove from the queue.
    if (ok) {
        for (uint16_t i = 0; i < num_packets; i++) {
            if (unit_double[first + i]) {
                c.doubles++;
                c.unstamped++;
            } else {
                stamps[i] = boundary64 - Age(boundary_ticks, unit_ticks[first + i]);
                c.stamped++;
            }
        }
        take = first + num_packets;
        c.batches_ok++;
    } else {
        // Count mismatch: no packet of this batch gets a stamp. Resync by dropping every unit that must belong to
        // this batch or an earlier one.
        c.unstamped += num_packets;
        if (num_packets > 0) c.batches_mismatch++;
        take = first + (exact_boundary ? surely : (maybe < num_packets ? maybe : num_packets));
        if (exact_boundary && surely > num_packets) c.orphans += surely - num_packets;
    }
    uint16_t drop = consumed_edges[take];
    tail = static_cast<uint16_t>((t + drop) & (kRingLen - 1));
}

}  // namespace mlat_stamp

#include "mlat_clear_dma.hh"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/timer.h)
#include DeviceFamily_constructPath(driverlib/udma.h)
#include DeviceFamily_constructPath(inc/hw_event.h)
#include DeviceFamily_constructPath(inc/hw_gpio.h)
#include DeviceFamily_constructPath(inc/hw_gpt.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_spi.h)
#include DeviceFamily_constructPath(inc/hw_types.h)
#include <ti/drivers/Power.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/power/PowerCC26X2.h>

#include "cycle_counter.hh"
#include "mlat_capture.hh"
#include "ti_drivers_config.h"

// Channel 10 (GPT0B): the clear. Channel 11 (GPT1A): the end of the frame. Channel 12 (GPT1B, software requests
// only): the clear time ring. The linker script places these control structures in the driver's control table.
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT0BPriControlTableEntry, (UDMA_CHAN_TIMER0_B + UDMA_PRI_SELECT));
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT0BAltControlTableEntry, (UDMA_CHAN_TIMER0_B + UDMA_ALT_SELECT));
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT1APriControlTableEntry, (UDMA_CHAN_TIMER1_A + UDMA_PRI_SELECT));
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT1AAltControlTableEntry, (UDMA_CHAN_TIMER1_A + UDMA_ALT_SELECT));
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT1BPriControlTableEntry, (UDMA_CHAN_TIMER1_B + UDMA_PRI_SELECT));
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT1BAltControlTableEntry, (UDMA_CHAN_TIMER1_B + UDMA_ALT_SELECT));

namespace mlat_clear_dma {

static constexpr uint32_t kChClear = UDMA_CHAN_TIMER0_B;
static constexpr uint32_t kChEnd = UDMA_CHAN_TIMER1_A;
static constexpr uint32_t kChRing = UDMA_CHAN_TIMER1_B;
static constexpr uint32_t kClearBytes = 6;
static constexpr uint32_t kHalfLen = kRingLen / 2;
static constexpr uint32_t kDisarmWaitMaxUs = 30;

// Words the lists copy into registers, in SRAM. Set/clear registers ignore zero bits.
static uint32_t kNssMask = 1u << LR_CS;
static uint32_t kClearBit = 1u << kChClear;
static uint32_t kEndBit = 1u << kChEnd;
static uint32_t kRingBit = 1u << kChRing;
static uint32_t kGpt1Start = GPT_CTL_TAEN;
static uint32_t kOne = 1;
static uint32_t kZero = 0;
static uint32_t clear_control = 0;  // Channel 10 primary control word when armed.
static uint32_t end_control = 0;    // Channel 11 primary control word when armed.
static uint32_t delay_ticks = 240;  // 5 us: 48 bits at 12 MHz take 4 us.

static volatile uint32_t chain_busy = 0;  // 1 from the first task of the clear list to the last task of the end list.
static volatile uint8_t rx_sink[kClearBytes];

static bool running = false;
static bool armed = false;
static Counters counters_;

static volatile uint32_t ring[kRingLen];
static uint16_t tail = 0;

#define TASK(count, size, src_inc, src, dst_inc, dst, arb, mode)                                                    \
    uDMATaskStructEntry(count, size, src_inc, const_cast<void*>(reinterpret_cast<const volatile void*>(src)), dst_inc, \
                        const_cast<void*>(reinterpret_cast<const volatile void*>(dst)), arb, mode)

static tDMAControlTable clear_list[9];
static tDMAControlTable end_list[7];

static void BuildLists(const uint8_t* clear_cmd) {
    const uint32_t mem = UDMA_MODE_MEM_SCATTER_GATHER;
    void* const doutclr = reinterpret_cast<void*>(GPIO_BASE + GPIO_O_DOUTCLR31_0);
    void* const doutset = reinterpret_cast<void*>(GPIO_BASE + GPIO_O_DOUTSET31_0);
    void* const txdata = reinterpret_cast<void*>(SPI0_BASE + SPI_O_TXDATA);
    void* const rxdata = reinterpret_cast<void*>(SPI0_BASE + SPI_O_RXDATA);
    void* const enaset = reinterpret_cast<void*>(UDMA0_BASE + UDMA_O_SETCHANNELEN);
    void* const prialt_clr = reinterpret_cast<void*>(UDMA0_BASE + UDMA_O_CLEARCHNLPRIALT);
    void* const softreq = reinterpret_cast<void*>(UDMA0_BASE + UDMA_O_SOFTREQ);
    const tDMAControlTable clear[] = {
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kOne, UDMA_DST_INC_NONE, &chain_busy, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kNssMask, UDMA_DST_INC_NONE, doutclr, UDMA_ARB_1, mem),
        TASK(kClearBytes, UDMA_SIZE_8, UDMA_SRC_INC_8, clear_cmd, UDMA_DST_INC_NONE, txdata, UDMA_ARB_8, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &end_control, UDMA_DST_INC_NONE,
             &dmaGPT1APriControlTableEntry.ui32Control, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kEndBit, UDMA_DST_INC_NONE, prialt_clr, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kEndBit, UDMA_DST_INC_NONE, enaset, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &delay_ticks, UDMA_DST_INC_NONE,
             reinterpret_cast<void*>(GPT1_BASE + GPT_O_TAV), UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kGpt1Start, UDMA_DST_INC_NONE,
             reinterpret_cast<void*>(GPT1_BASE + GPT_O_CTL), UDMA_ARB_1, UDMA_MODE_AUTO),
    };
    const tDMAControlTable end[] = {
        TASK(kClearBytes, UDMA_SIZE_8, UDMA_SRC_INC_NONE, rxdata, UDMA_DST_INC_8, rx_sink, UDMA_ARB_8, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &clear_control, UDMA_DST_INC_NONE,
             &dmaGPT0BPriControlTableEntry.ui32Control, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kClearBit, UDMA_DST_INC_NONE, prialt_clr, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kClearBit, UDMA_DST_INC_NONE, enaset, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kNssMask, UDMA_DST_INC_NONE, doutset, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kRingBit, UDMA_DST_INC_NONE, softreq, UDMA_ARB_1, mem),
        TASK(1, UDMA_SIZE_32, UDMA_SRC_INC_NONE, &kZero, UDMA_DST_INC_NONE, &chain_busy, UDMA_ARB_1, UDMA_MODE_AUTO),
    };
    static_assert(sizeof(clear) <= sizeof(clear_list) && sizeof(end) <= sizeof(end_list), "task lists");
    for (uint32_t i = 0; i < sizeof(clear) / sizeof(clear[0]); i++) clear_list[i] = clear[i];
    for (uint32_t i = 0; i < sizeof(end) / sizeof(end[0]); i++) end_list[i] = end[i];
    const uint32_t num_clear = sizeof(clear) / sizeof(clear[0]);
    const uint32_t num_end = sizeof(end) / sizeof(end[0]);

    dmaGPT0BPriControlTableEntry.pvSrcEndAddr = &clear_list[num_clear - 1].ui32Spare;
    dmaGPT0BPriControlTableEntry.pvDstEndAddr = &dmaGPT0BAltControlTableEntry.ui32Spare;
    clear_control = UDMA_DST_INC_32 | UDMA_SRC_INC_32 | UDMA_SIZE_32 | UDMA_ARB_4 |
                    ((num_clear * 4 - 1) << UDMA_XFER_SIZE_S) | UDMA_MODE_MEM_SCATTER_GATHER;
    dmaGPT0BPriControlTableEntry.ui32Control = clear_control;

    dmaGPT1APriControlTableEntry.pvSrcEndAddr = &end_list[num_end - 1].ui32Spare;
    dmaGPT1APriControlTableEntry.pvDstEndAddr = &dmaGPT1AAltControlTableEntry.ui32Spare;
    end_control = UDMA_DST_INC_32 | UDMA_SRC_INC_32 | UDMA_SIZE_32 | UDMA_ARB_4 |
                  ((num_end * 4 - 1) << UDMA_XFER_SIZE_S) | UDMA_MODE_MEM_SCATTER_GATHER;
    dmaGPT1APriControlTableEntry.ui32Control = 0;  // Armed by the clear list.
}

static void ArmRingHalf(uint32_t select, volatile uint32_t* dst) {
    uDMAChannelControlSet(UDMA0_BASE, kChRing | select, UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_1);
    uDMAChannelTransferSet(UDMA0_BASE, kChRing | select, UDMA_MODE_PINGPONG,
                           reinterpret_cast<void*>(GPT0_BASE + GPT_O_TAV), const_cast<uint32_t*>(dst), kHalfLen);
}

static void StartRing() {
    uDMAChannelDisable(UDMA0_BASE, kChRing);
    HWREG(UDMA0_BASE + UDMA_O_CLEARCHNLPRIALT) = kRingBit;
    ArmRingHalf(UDMA_PRI_SELECT, ring);
    ArmRingHalf(UDMA_ALT_SELECT, ring + kHalfLen);
    uDMAChannelEnable(UDMA0_BASE, kChRing);
    tail = 0;
}

static void ChannelReset(uint32_t ch) {
    uDMAChannelDisable(UDMA0_BASE, ch);
    HWREG(UDMA0_BASE + UDMA_O_CLEARCHNLPRIALT) = 1u << ch;
    uDMAChannelAttributeDisable(UDMA0_BASE, ch, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_REQMASK);
}

void Start(const uint8_t* clear_cmd) {
    if (running || !mlat_capture::Running()) {
        return;
    }
    Power_setDependency(PowerCC26XX_PERIPH_GPT1);
    // GPT1A: one-shot delay, started by the clear list; its timeout requests channel 11.
    TimerDisable(GPT1_BASE, TIMER_BOTH);
    TimerConfigure(GPT1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT);
    TimerPrescaleSet(GPT1_BASE, TIMER_A, 0);
    TimerLoadSet(GPT1_BASE, TIMER_A, delay_ticks);
    TimerIntDisable(GPT1_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
    HWREG(GPT1_BASE + GPT_O_DMAEV) = GPT_DMAEV_TATODMAEN;

    ChannelReset(kChClear);
    ChannelReset(kChEnd);
    ChannelReset(kChRing);
    uDMAChannelAttributeEnable(UDMA0_BASE, kChClear, UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    uDMAChannelAttributeEnable(UDMA0_BASE, kChEnd, UDMA_ATTR_HIGH_PRIORITY);
    BuildLists(clear_cmd);
    HWREG(EVENT_BASE + EVENT_O_UDMACH10SSEL) = EVENT_UDMACH10SSEL_EV_GPT0B_DMABREQ;
    HWREG(EVENT_BASE + EVENT_O_UDMACH10BSEL) = EVENT_UDMACH10BSEL_EV_GPT0B_DMABREQ;
    HWREG(EVENT_BASE + EVENT_O_UDMACH11SSEL) = EVENT_UDMACH11SSEL_EV_GPT1A_DMABREQ;
    HWREG(EVENT_BASE + EVENT_O_UDMACH11BSEL) = EVENT_UDMACH11BSEL_EV_GPT1A_DMABREQ;
    HWREG(EVENT_BASE + EVENT_O_UDMACH12SSEL) = EVENT_UDMACH12SSEL_EV_NONE;
    HWREG(EVENT_BASE + EVENT_O_UDMACH12BSEL) = EVENT_UDMACH12BSEL_EV_NONE;
    StartRing();
    chain_busy = 0;
    uDMAChannelEnable(UDMA0_BASE, kChClear);  // Request masked until Arm().

    // GPT0B: second capture of the DIO5 port event, as the trigger.
    HWREG(EVENT_BASE + EVENT_O_GPT0BCAPTSEL) = EVENT_GPT0BCAPTSEL_EV_PORT_EVENT0;
    TimerPrescaleSet(GPT0_BASE, TIMER_B, 0xFF);
    TimerLoadSet(GPT0_BASE, TIMER_B, 0xFFFF);
    TimerEventControl(GPT0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
    HWREG(GPT0_BASE + GPT_O_DMAEV) |= GPT_DMAEV_CBEDMAEN;
    TimerEnable(GPT0_BASE, TIMER_B);
    armed = false;
    running = true;
}

void Stop() {
    if (!running) {
        return;
    }
    Disarm();
    running = false;
    HWREG(GPT0_BASE + GPT_O_DMAEV) &= ~GPT_DMAEV_CBEDMAEN;
    TimerDisable(GPT0_BASE, TIMER_B);
    TimerDisable(GPT1_BASE, TIMER_BOTH);
    HWREG(GPT1_BASE + GPT_O_DMAEV) = 0;
    uDMAChannelDisable(UDMA0_BASE, kChClear);
    uDMAChannelDisable(UDMA0_BASE, kChEnd);
    uDMAChannelDisable(UDMA0_BASE, kChRing);
    Power_releaseDependency(PowerCC26XX_PERIPH_GPT1);
}

bool Running() { return running; }
bool Armed() { return armed; }

static bool ChainActive() {
    return chain_busy != 0 || dmaGPT0BPriControlTableEntry.ui32Control != clear_control;
}

static void FlushRx() {
    while (!(HWREG(SPI0_BASE + SPI_O_STAT) & SPI_STAT_RFE)) {
        (void)HWREG(SPI0_BASE + SPI_O_RXDATA);
        counters_.rx_flushed_bytes++;
    }
}

void Arm() {
    if (!running || armed) {
        return;
    }
    // A chain pushes bytes into the TX FIFO: only with the SPI enabled and idle.
    if (!(HWREG(SPI0_BASE + SPI_O_CTL1) & SPI_CTL1_ENABLE) || (HWREG(SPI0_BASE + SPI_O_STAT) & SPI_STAT_BUSY)) {
        return;
    }
    uintptr_t key = HwiP_disable();
    // Latched high: no edge can come until a clear, so the software request is the only start.
    const bool latched = mlat_capture::PinHigh();
    HWREG(UDMA0_BASE + UDMA_O_CLEARREQMASK) = kClearBit;
    if (latched) {
        HWREG(UDMA0_BASE + UDMA_O_SOFTREQ) = kClearBit;
        counters_.kicks++;
    }
    armed = true;
    HwiP_restore(key);
}

bool Disarm() {
    if (!running) {
        return true;
    }
    uintptr_t key = HwiP_disable();
    if (!armed) {
        HwiP_restore(key);
        return true;
    }
    HWREG(UDMA0_BASE + UDMA_O_SETREQMASK) = kClearBit;
    armed = false;
    counters_.disarms++;
    // A request taken just before the mask shows within a few cycles: look twice, 1 us apart.
    const uint32_t start = CycleCounter::Now();
    const uint32_t settle = CycleCounter::kCpuHz / 1000000;
    const uint32_t limit = kDisarmWaitMaxUs * (CycleCounter::kCpuHz / 1000000);
    bool waited = false;
    bool ok = true;
    while (true) {
        const uint32_t elapsed = CycleCounter::Since(start);
        if (ChainActive()) {
            waited = true;
        } else if (elapsed >= settle) {
            break;
        }
        if (elapsed > limit) {
            ok = false;
            counters_.disarm_timeouts++;
            break;
        }
    }
    if (waited) {
        counters_.disarm_waits++;
        const uint32_t cycles = CycleCounter::Since(start);
        if (cycles > counters_.disarm_wait_max_cycles) counters_.disarm_wait_max_cycles = cycles;
    }
    FlushRx();
    HwiP_restore(key);
    return ok;
}

void SetDelayTicks(uint32_t ticks) {
    delay_ticks = ticks;
    if (running) {
        TimerLoadSet(GPT1_BASE, TIMER_A, ticks);
    }
}
uint32_t DelayTicks() { return delay_ticks; }

static bool RingHalfStopped(const volatile tDMAControlTable& entry) {
    return (entry.ui32Control & UDMA_MODE_M) == UDMA_MODE_STOP;
}
static bool RingAltActive() { return (HWREG(UDMA0_BASE + UDMA_O_SETCHNLPRIALT) >> kChRing) & 1; }

static uint16_t RingHead() {
    for (int tries = 0; tries < 4; tries++) {
        const bool alt = RingAltActive();
        const uint32_t control =
            alt ? dmaGPT1BAltControlTableEntry.ui32Control : dmaGPT1BPriControlTableEntry.ui32Control;
        if (alt != RingAltActive()) {
            continue;
        }
        const uint32_t remaining =
            (control & UDMA_MODE_M) == UDMA_MODE_STOP ? 0 : ((control & UDMA_XFER_SIZE_M) >> 4) + 1;
        return static_cast<uint16_t>(((alt ? kHalfLen : 0) + (kHalfLen - remaining)) & (kRingLen - 1));
    }
    return tail;
}

uint16_t ReadClears(uint32_t* out, bool* overrun) {
    *overrun = false;
    if (!running) {
        return 0;
    }
    const uint16_t head = RingHead();
    const bool alt = RingAltActive();
    const bool pri_stopped = RingHalfStopped(dmaGPT1BPriControlTableEntry);
    const bool alt_stopped = RingHalfStopped(dmaGPT1BAltControlTableEntry);
    if ((pri_stopped && alt_stopped) || !uDMAChannelIsEnabled(UDMA0_BASE, kChRing)) {
        StartRing();
        *overrun = true;
        return 0;
    }
    if (alt && pri_stopped) {
        ArmRingHalf(UDMA_PRI_SELECT, ring);
    } else if (!alt && alt_stopped) {
        ArmRingHalf(UDMA_ALT_SELECT, ring + kHalfLen);
    }
    uint16_t num = 0;
    while (tail != head) {
        out[num++] = ring[tail] & 0x00FFFFFF;
        tail = static_cast<uint16_t>((tail + 1) & (kRingLen - 1));
    }
    return num;
}

const Counters& counters() { return counters_; }
void ResetCounters() { counters_ = Counters(); }

}  // namespace mlat_clear_dma

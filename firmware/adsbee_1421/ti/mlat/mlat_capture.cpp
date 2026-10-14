#include "mlat_capture.hh"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/osc.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)
#include DeviceFamily_constructPath(driverlib/timer.h)
#include DeviceFamily_constructPath(driverlib/udma.h)
#include DeviceFamily_constructPath(inc/hw_event.h)
#include DeviceFamily_constructPath(inc/hw_gpio.h)
#include DeviceFamily_constructPath(inc/hw_gpt.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include <ti/drivers/Power.h>
#include <ti/drivers/dma/UDMACC26XX.h>
#include <ti/drivers/power/PowerCC26X2.h>

// uDMA channel 9 (GPT0A) control structures. The linker script places them in the driver's control table.
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT0APriControlTableEntry, (UDMA_CHAN_TIMER0_A + UDMA_PRI_SELECT));
ALLOCATE_CONTROL_TABLE_ENTRY(dmaGPT0AAltControlTableEntry, (UDMA_CHAN_TIMER0_A + UDMA_ALT_SELECT));

namespace mlat_capture {

static constexpr uint32_t kCaptureIoid = IOID_13;  // LR2021 DIO5.
static constexpr uint32_t kCounterMask = 0x00FFFFFF;
// The uDMA fills the two ring halves in turn (ping-pong); Read() hands a finished half back.
static constexpr uint16_t kHalfLen = kRingLen / 2;

static bool running = false;
static volatile uint32_t ring[kRingLen];
static uint16_t tail = 0;  // Next ring entry to read.

static void ArmHalf(uint32_t select, volatile uint32_t* dst) {
    uDMAChannelControlSet(UDMA0_BASE, UDMA_CHAN_TIMER0_A | select,
                          UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 | UDMA_ARB_1);
    uDMAChannelTransferSet(UDMA0_BASE, UDMA_CHAN_TIMER0_A | select, UDMA_MODE_PINGPONG,
                           reinterpret_cast<void*>(GPT0_BASE + GPT_O_TAR), const_cast<uint32_t*>(dst), kHalfLen);
}

static void StartDma() {
    uDMAChannelDisable(UDMA0_BASE, UDMA_CHAN_TIMER0_A);
    HWREG(UDMA0_BASE + UDMA_O_CLEARCHNLPRIALT) = 1u << UDMA_CHAN_TIMER0_A;
    uDMAChannelAttributeDisable(UDMA0_BASE, UDMA_CHAN_TIMER0_A,
                                UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    ArmHalf(UDMA_PRI_SELECT, ring);
    ArmHalf(UDMA_ALT_SELECT, ring + kHalfLen);
    // Route the timer's capture event to uDMA channel 9 as single and burst request.
    HWREG(EVENT_BASE + EVENT_O_UDMACH9SSEL) = EVENT_UDMACH9SSEL_EV_GPT0A_DMABREQ;
    HWREG(EVENT_BASE + EVENT_O_UDMACH9BSEL) = EVENT_UDMACH9BSEL_EV_GPT0A_DMABREQ;
    HWREG(GPT0_BASE + GPT_O_DMAEV) |= GPT_DMAEV_CAEDMAEN;
    uDMAChannelEnable(UDMA0_BASE, UDMA_CHAN_TIMER0_A);
    tail = 0;
}

static bool HalfStopped(const volatile tDMAControlTable& entry) {
    return (entry.ui32Control & UDMA_MODE_M) == UDMA_MODE_STOP;
}

static bool AltHalfActive() { return (HWREG(UDMA0_BASE + UDMA_O_SETCHNLPRIALT) >> UDMA_CHAN_TIMER0_A) & 1; }

// Ring index the uDMA writes next.
static uint16_t Head() {
    for (int tries = 0; tries < 4; tries++) {
        const bool alt = AltHalfActive();
        const uint32_t control =
            alt ? dmaGPT0AAltControlTableEntry.ui32Control : dmaGPT0APriControlTableEntry.ui32Control;
        if (alt != AltHalfActive()) {
            continue;  // The uDMA changed halves between the two reads.
        }
        const uint32_t remaining =
            (control & UDMA_MODE_M) == UDMA_MODE_STOP ? 0 : ((control & UDMA_XFER_SIZE_M) >> 4) + 1;
        return static_cast<uint16_t>(((alt ? kHalfLen : 0) + (kHalfLen - remaining)) & (kRingLen - 1));
    }
    return tail;
}

void Start() {
    if (running) {
        return;
    }
    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);  // The timer stops in STANDBY.
    Power_setDependency(PowerCC26XX_XOSC_HF);
    Power_setDependency(PowerCC26XX_PERIPH_GPT0);
    Power_setDependency(PowerCC26XX_PERIPH_UDMA);
    IOCPortConfigureSet(kCaptureIoid, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_INPUT | IOC_IOPULL_DOWN);
    TimerDisable(GPT0_BASE, TIMER_A);
    // Timer B stays off unless mlat_clear_dma uses it as the clear trigger.
    TimerConfigure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_CAP_TIME_UP);
    TimerPrescaleSet(GPT0_BASE, TIMER_A, 0xFF);  // In capture mode the prescaler extends the timer to 24 bits.
    TimerLoadSet(GPT0_BASE, TIMER_A, 0xFFFF);
    TimerEventControl(GPT0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(GPT0_BASE, TIMER_A);
    StartDma();
    running = true;
}

void Stop() {
    if (!running) {
        return;
    }
    running = false;
    HWREG(GPT0_BASE + GPT_O_DMAEV) &= ~GPT_DMAEV_CAEDMAEN;
    uDMAChannelDisable(UDMA0_BASE, UDMA_CHAN_TIMER0_A);
    TimerDisable(GPT0_BASE, TIMER_A);
    IOCPortConfigureSet(kCaptureIoid, IOC_PORT_GPIO, IOC_NO_IOPULL);  // Reset state: input buffer off.
    Power_releaseDependency(PowerCC26XX_PERIPH_UDMA);
    Power_releaseDependency(PowerCC26XX_PERIPH_GPT0);
    Power_releaseDependency(PowerCC26XX_XOSC_HF);
    Power_releaseConstraint(PowerCC26XX_DISALLOW_STANDBY);
}

bool Running() { return running; }

bool ClockIsCrystal() { return OSCClockSourceGet(OSC_SRC_CLK_HF) == OSC_XOSC_HF; }

uint32_t Counter() { return HWREG(GPT0_BASE + GPT_O_TAV) & kCounterMask; }

bool PinHigh() { return (HWREG(GPIO_BASE + GPIO_O_DIN31_0) >> kCaptureIoid) & 1; }

uint16_t Read(uint32_t* out, bool* overrun) {
    *overrun = false;
    if (!running) {
        return 0;
    }
    const uint16_t head = Head();
    const bool alt = AltHalfActive();
    const bool pri_stopped = HalfStopped(dmaGPT0APriControlTableEntry);
    const bool alt_stopped = HalfStopped(dmaGPT0AAltControlTableEntry);
    if ((pri_stopped && alt_stopped) || !uDMAChannelIsEnabled(UDMA0_BASE, UDMA_CHAN_TIMER0_A)) {
        // Both halves filled before one was handed back.
        StartDma();
        *overrun = true;
        return 0;
    }
    // Hand the half the uDMA has left back to it.
    if (alt && pri_stopped) {
        ArmHalf(UDMA_PRI_SELECT, ring);
    } else if (!alt && alt_stopped) {
        ArmHalf(UDMA_ALT_SELECT, ring + kHalfLen);
    }
    uint16_t num = 0;
    while (tail != head) {
        out[num++] = ring[tail] & kCounterMask;
        tail = static_cast<uint16_t>((tail + 1) & (kRingLen - 1));
    }
    return num;
}

}  // namespace mlat_capture

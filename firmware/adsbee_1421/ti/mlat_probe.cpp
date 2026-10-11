#include "mlat_probe.hh"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/prcm.h)
#include DeviceFamily_constructPath(driverlib/timer.h)
#include DeviceFamily_constructPath(driverlib/gpio.h)
#include DeviceFamily_constructPath(inc/hw_gpt.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include <ti/drivers/Power.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/power/PowerCC26X2.h>

namespace mlat_probe {

static constexpr uint32_t kProbeIoid = IOID_13;  // LR2021 DIO5.
static constexpr uint16_t kRingLen = 256;        // Power of 2.

volatile bool hold = false;

static bool inited = false;
static HwiP_Struct hwi;
static Edge ring[kRingLen];
static volatile uint16_t head = 0;  // ISR writes.
static volatile uint16_t tail = 0;  // Thread reads.
static volatile uint32_t dropped = 0;
static volatile uint32_t total = 0;

static void CaptureIsr(uintptr_t /*arg*/) {
    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    // In edge-time mode TAR holds the captured count: prescaler (extension) in [23:16], timer in [15:0].
    uint32_t ticks = HWREG(GPT0_BASE + GPT_O_TAR) & 0x00FFFFFF;
    uint8_t level = static_cast<uint8_t>(GPIO_readDio(kProbeIoid));
    total = total + 1;
    uint16_t next = static_cast<uint16_t>((head + 1) & (kRingLen - 1));
    if (next == tail) {
        dropped = dropped + 1;
        return;
    }
    ring[head] = {ticks, level};
    head = next;
}

void Init() {
    if (inited) {
        return;
    }
    // Keep the HF clock and GPT0 running: no standby while the probe firmware is up.
    Power_setConstraint(PowerCC26XX_DISALLOW_STANDBY);
    Power_setDependency(PowerCC26XX_PERIPH_GPT0);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_TIMER0);
    PRCMLoadSet();

    // DIO_13 as an input routed to MCU port event 0, which is the default capture source of GPT0A.
    IOCPortConfigureSet(kProbeIoid, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_INPUT | IOC_IOPULL_DOWN);

    TimerDisable(GPT0_BASE, TIMER_A);
    TimerConfigure(GPT0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerPrescaleSet(GPT0_BASE, TIMER_A, 0xFF);  // Timer extension in capture mode: 24 bits.
    TimerLoadSet(GPT0_BASE, TIMER_A, 0xFFFF);
    TimerEventControl(GPT0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

    HwiP_Params params;
    HwiP_Params_init(&params);
    params.priority = 0;  // Highest: the ISR only reads the latched capture.
    HwiP_construct(&hwi, INT_GPT0A, CaptureIsr, &params);

    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(GPT0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(GPT0_BASE, TIMER_A);
    inited = true;
}

bool IsInited() { return inited; }

uint32_t Now() { return HWREG(GPT0_BASE + GPT_O_TAV) & 0x00FFFFFF; }

uint8_t PinLevel() { return static_cast<uint8_t>(GPIO_readDio(kProbeIoid)); }

void SetEdges(uint8_t mode) {
    TimerEventControl(GPT0_BASE, TIMER_A,
                      mode == 0 ? TIMER_EVENT_POS_EDGE : mode == 1 ? TIMER_EVENT_NEG_EDGE : TIMER_EVENT_BOTH_EDGES);
}

bool PopEdge(Edge& edge) {
    if (tail == head) {
        return false;
    }
    edge = ring[tail];
    tail = static_cast<uint16_t>((tail + 1) & (kRingLen - 1));
    return true;
}

uint32_t DroppedEdges() { return dropped; }
uint32_t TotalEdges() { return total; }

void Flush() {
    tail = head;
    dropped = 0;
    total = 0;
}

}  // namespace mlat_probe

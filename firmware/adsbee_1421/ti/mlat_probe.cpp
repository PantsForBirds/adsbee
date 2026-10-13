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

static uint32_t probe_ioid = IOID_13;       // LR2021 DIO5.
static constexpr uint32_t kTrigIoid = IOID_12;  // LR2021 DIO6.
static constexpr uint16_t kRingLen = 1024;  // Power of 2.

volatile bool hold = false;

static bool inited = false;
static HwiP_Struct hwi;
static Edge ring[kRingLen];
static volatile uint16_t head = 0;  // ISR writes.
static volatile uint16_t tail = 0;  // Thread reads.
static volatile uint32_t dropped = 0;
static volatile uint32_t total = 0;
static volatile bool one_shot = false;
static volatile bool count_only = false;
static volatile uint8_t trig_edges = 0;
static volatile uint16_t trig_width = 0, trig_delay = 0;
static Stats stats;
static uint32_t last_rise = 0, last_fall = 0;
static bool have_rise = false, have_fall = false;

static void CaptureIsr(uintptr_t /*arg*/) {
    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    // In edge-time mode TAR holds the captured count: prescaler (extension) in [23:16], timer in [15:0].
    uint32_t ticks = HWREG(GPT0_BASE + GPT_O_TAR) & 0x00FFFFFF;
    uint8_t level = static_cast<uint8_t>(GPIO_readDio(probe_ioid));
    if (trig_edges & (level ? 2 : 1)) {
        for (volatile uint16_t i = 0; i < trig_delay; i = i + 1) {
        }
        GPIO_setDio(kTrigIoid);
        for (volatile uint16_t i = 0; i < trig_width; i = i + 1) {
        }
        GPIO_clearDio(kTrigIoid);
        stats.trig_pulses++;
    }
    total = total + 1;
    if (level) {
        stats.rising++;
        if (have_fall) {
            uint32_t w = (ticks - last_fall) & 0x00FFFFFF;
            if (w < stats.low_min) stats.low_min = w;
            if (w > stats.low_max) stats.low_max = w;
            stats.low_sum += w;
        }
        last_rise = ticks;
        have_rise = true;
    } else {
        stats.falling++;
        if (have_rise) {
            uint32_t w = (ticks - last_rise) & 0x00FFFFFF;
            if (w < stats.high_min) stats.high_min = w;
            if (w > stats.high_max) stats.high_max = w;
            stats.high_sum += w;
        }
        last_fall = ticks;
        have_fall = true;
    }
    if (count_only) {
        return;
    }
    uint16_t next = static_cast<uint16_t>((head + 1) & (kRingLen - 1));
    if (next == tail) {
        // Ring full: a fast signal on the pin. Stop capturing until the next Flush() so the ISR can't starve the
        // main loop.
        dropped = dropped + 1;
        TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
        return;
    }
    ring[head] = {ticks, level};
    head = next;
    if (one_shot) {
        TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);  // Re-armed by Arm().
    }
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
    IOCPortConfigureSet(probe_ioid, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_INPUT | IOC_IOPULL_DOWN);

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

uint8_t PinLevel() { return static_cast<uint8_t>(GPIO_readDio(probe_ioid)); }

void SetEdges(uint8_t mode) {
    uint8_t edges = mode & 3;
    one_shot = (mode & 4) != 0;
    TimerEventControl(GPT0_BASE, TIMER_A,
                      edges == 0 ? TIMER_EVENT_POS_EDGE : edges == 1 ? TIMER_EVENT_NEG_EDGE : TIMER_EVENT_BOTH_EDGES);
}

void Arm() {
    TimerIntClear(GPT0_BASE, TIMER_CAPA_EVENT);
    TimerIntEnable(GPT0_BASE, TIMER_CAPA_EVENT);
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
    TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
    tail = head;
    dropped = 0;
    total = 0;
    stats = Stats{};
    stats.high_min = stats.low_min = 0xFFFFFFFF;
    have_rise = have_fall = false;
    Arm();
}

void SetSource(uint8_t ioid) {
    TimerIntDisable(GPT0_BASE, TIMER_CAPA_EVENT);
    IOCPortConfigureSet(probe_ioid, IOC_PORT_GPIO, IOC_STD_INPUT | IOC_IOPULL_DOWN);
    probe_ioid = ioid;
    IOCPortConfigureSet(probe_ioid, IOC_PORT_MCU_PORT_EVENT0, IOC_STD_INPUT | IOC_NO_IOPULL);
    Flush();
}

void SetCountOnly(bool on) { count_only = on; }

Stats GetStats() {
    uintptr_t key = HwiP_disable();
    Stats copy = stats;
    HwiP_restore(key);
    return copy;
}

void TrigEnable(bool enable) {
    trig_edges = 0;
    if (enable) {
        GPIO_clearDio(kTrigIoid);
        IOCPortConfigureSet(kTrigIoid, IOC_PORT_GPIO, IOC_STD_OUTPUT);
        GPIO_setOutputEnableDio(kTrigIoid, GPIO_OUTPUT_ENABLE);
    } else {
        GPIO_setOutputEnableDio(kTrigIoid, GPIO_OUTPUT_DISABLE);
        IOCPortConfigureSet(kTrigIoid, IOC_PORT_GPIO, IOC_STD_INPUT | IOC_IOPULL_DOWN);
    }
}

void TrigLevel(bool high) {
    if (high) {
        GPIO_setDio(kTrigIoid);
    } else {
        GPIO_clearDio(kTrigIoid);
    }
}

void TrigAuto(uint8_t edges, uint16_t width, uint16_t delay) {
    trig_width = width;
    trig_delay = delay;
    trig_edges = edges;
}

}  // namespace mlat_probe

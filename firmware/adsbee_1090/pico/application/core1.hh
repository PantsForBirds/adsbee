#pragma once

#include "adsbee.hh"
#include "cpu_utils.hh"
#include "mode_s_packet_decoder.hh"
#include "pico/multicore.h"

extern CPUMonitor core_1_monitor;

#ifdef HARDWARE_UNIT_TESTS
void RunPendingTestFunctionOnCore1();  // Defined in target_test/hardware_unit_tests.cc.
#endif

inline void main_core1() {
#ifdef ISRS_ON_CORE1
    adsbee.InitISRs();
#else
    irq_set_enabled(IO_IRQ_BANK0, false);  // Disable GPIO IRQs (OnDemodBegin) on this core.
    irq_set_enabled(PIO0_IRQ_0, false);    // Disable PIO0 IRQ0 (OnDemodComplete) on this core.
#endif  // ISRS_ON_CORE1

    while (true) {
#ifdef ISRS_ON_CORE1
        // Core 0 may try to toggle the 1090 receiver on and off. Keep an eye on it and turn it on and off here, since
        // it can't be toggle from core 0 without accidentally duplicating all the ISR stuff on that core.
        if (adsbee.Receiver1090IsEnabled() != adsbee.Receiver1090IRQIsEnabled()) {
            adsbee.SyncReceiver1090IRQEnable();
        }
#endif  // ISRS_ON_CORE1
#ifdef HARDWARE_UNIT_TESTS
        RunPendingTestFunctionOnCore1();  // Lets hardware unit tests read core 1 state, e.g. its SysTick.
#endif
        core_1_monitor.Tick();
        decoder.UpdateDecoderLoop();
    }
}

inline void StopCore1() {
    adsbee.DeInitISRs();  // Clean up DMA channels and other ISR resources before stopping core
    multicore_reset_core1();
}
inline void StartCore1() { multicore_launch_core1(main_core1); }
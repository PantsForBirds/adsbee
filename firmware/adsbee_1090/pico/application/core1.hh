#pragma once

#include "adsbee.hh"
#include "cpu_utils.hh"
#include "mode_s_packet_decoder.hh"
#include "pico/multicore.h"
#include "ram_monitor.hh"

extern CPUMonitor core_1_monitor;

#ifdef HARDWARE_UNIT_TESTS
void RunPendingTestFunctionOnCore1();  // Defined in target_test/hardware_unit_tests.cc.
#endif

inline void __not_in_flash_func(main_core1)() {
#ifdef ISRS_ON_CORE1
    adsbee.InitISRs();
#else
    irq_set_enabled(IO_IRQ_BANK0, false);  // Disable GPIO IRQs (OnDemodBegin) on this core.
    irq_set_enabled(PIO0_IRQ_0, false);    // Disable PIO0 IRQ0 (OnDemodComplete) on this core.
#endif  // ISRS_ON_CORE1
    core_1_monitor.TimeInterrupts();

    while (true) {
        core_1_monitor.BeginWork();  // Counted only if the pass demodulated or decoded something.
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
#ifdef ISRS_ON_CORE1
        adsbee.sync_pulse_service.Service(time_us_32());  // Needs this core's MLAT counter.
#endif  // ISRS_ON_CORE1
        bool did_work = adsbee.DemodulateCaptures() || decoder.raw_mode_s_packet_in_queue.Length() > 0;
        decoder.UpdateDecoderLoop();
        core_1_monitor.EndWork(did_work);
    }
}

// Set by LaunchCore1(). Until then StopCore1() and StartCore1() do nothing: a flash write early in boot (a settings
// migration) must not start core 1, whose DMA channels main() would then leak when it launches core 1 itself.
inline bool core1_launched = false;

inline void LaunchCore1() {
    core1_launched = true;
    multicore_reset_core1();
    RAMMonitor::PaintCore1Stack();
    multicore_launch_core1(main_core1);
    adsbee.subg_radio_ll.AllowSyncPulsesInReset();  // With the CC1312 running, its next status read allows them.
}

inline void StopCore1() {
    if (!core1_launched) {
        return;
    }
    adsbee.sync_pulse_service.Block();  // Never stop core 1 in the middle of a sync pulse.
    adsbee.DeInitISRs();  // Clean up DMA channels and other ISR resources before stopping core
    core_1_monitor.StopTimingInterrupts();  // InitISRs() sets the handlers again on restart.
    multicore_reset_core1();
}

inline void StartCore1() {
    if (!core1_launched) {
        return;
    }
    RAMMonitor::PaintCore1Stack();
    multicore_launch_core1(main_core1);
    adsbee.subg_radio_ll.AllowSyncPulsesInReset();  // With the CC1312 running, its next status read allows them.
}
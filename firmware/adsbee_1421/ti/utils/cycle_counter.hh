#pragma once

// CPU cycle counter for the CPU-cost stats in AT+RX_STATS: 48 cycles per microsecond on the CC1314.
//
// It runs on SysTick (free-running, 24 bits, core clock), which this firmware doesn't otherwise use. The
// Cortex-M33 DWT cycle counter would be the natural choice, but on the CC1314 it never counts without a
// debugger attached: DAUTHSTATUS reads 0xAA (secure and non-secure non-invasive debug disabled), and with that
// the DWT counters are held even with DEMCR.TRCENA and DWT_CTRL.CYCCNTENA set (checked on target).
//
// 24 bits wrap every 349 ms, so only intervals shorter than that measure correctly: take Since(start) within
// one measurement, never compare absolute values. Reading it costs one load.
//
// The registers are addressed directly (ARMv8-M Architecture Reference Manual, the same on every Cortex-M33), so
// this header needs no device include.

#include <cstdint>

namespace CycleCounter {

static volatile uint32_t* const kSystCsr = reinterpret_cast<volatile uint32_t*>(0xE000E010u);
static volatile uint32_t* const kSystRvr = reinterpret_cast<volatile uint32_t*>(0xE000E014u);
static volatile uint32_t* const kSystCvr = reinterpret_cast<volatile uint32_t*>(0xE000E018u);
static constexpr uint32_t kSystCsrEnable = 1u << 0;
static constexpr uint32_t kSystCsrClkSourceCpu = 1u << 2;  // No TICKINT: it never interrupts.
static constexpr uint32_t kMask = 0x00FFFFFFu;
static constexpr uint32_t kCpuHz = 48000000u;

// Starts the counter. Idempotent (a running counter is left alone); call again after a low-power sleep.
inline void Enable() {
    if ((*kSystCsr & kSystCsrEnable) == 0) {
        *kSystRvr = kMask;
        *kSystCvr = 0;  // Any write clears it; it reloads from kMask on the next cycle.
        *kSystCsr = kSystCsrClkSourceCpu | kSystCsrEnable;
    }
}

// A reading that counts up (SysTick counts down), modulo 2^24.
inline uint32_t Now() { return (~*kSystCvr) & kMask; }

// Cycles since `start` (a Now() reading less than 349 ms old).
inline uint32_t Since(uint32_t start) { return (Now() - start) & kMask; }

}  // namespace CycleCounter

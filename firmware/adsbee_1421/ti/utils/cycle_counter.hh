#pragma once

// CPU cycle counter (48 cycles per microsecond) for the CPU-cost stats in AT+RX_STATS.
//
// Uses SysTick (24 bits, core clock), which the firmware doesn't otherwise use. The Cortex-M33 DWT cycle counter
// doesn't count on the CC1314 without a debugger attached.
//
// Wraps every 349 ms: only measure shorter intervals with Since(start), never compare absolute values.

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

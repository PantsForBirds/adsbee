#pragma once
// t-1339 LAB ONLY: state of core 1 when it stops handling GPIO interrupts.
// Core 1's SysTick (MLAT wrap, priority 0, every 134 ms) samples the interrupted context (stacked PC/LR/xPSR) and
// core 1's private NVIC/SCB state. A HardFault on either core records its frame and spins (counting).
#include <stdint.h>

#include "hardware/structs/scb.h"
#include "hardware/structs/nvic.h"
#include "hardware/structs/sio.h"

struct T1339Diag {
    volatile uint32_t loop;          // Core 1 main loop passes.
    volatile uint32_t loop_primask;  // PRIMASK seen in the core 1 loop (thread mode).
    volatile uint32_t ticks;         // Core 1 SysTick wraps sampled.
    volatile uint32_t pc, lr, xpsr;  // Interrupted context of the latest core 1 SysTick.
    volatile uint32_t icsr, iser, ispr;
    volatile uint32_t ipsr_hist[48];  // Interrupted exception number per SysTick sample (0 = thread).
    volatile uint32_t pc_ring[16];    // Last 16 sampled PCs.
    volatile uint32_t hf_count, hf_core, hf_pc, hf_lr, hf_xpsr, hf_sp, hf_r0, hf_r1, hf_r2, hf_r3, hf_r12;
    volatile uint32_t gpio_isr_enter, gpio_isr_exit;  // on_demod_pin_change() entries/exits on core 1.
};
extern T1339Diag t1339_diag;

extern "C" void t1339_systick_c(uint32_t* frame);
extern "C" void t1339_hardfault_c(uint32_t* frame);
extern "C" void t1339_systick_entry();
extern "C" void t1339_hardfault_entry();
extern T1339Diag t1339_prev;
extern uint32_t t1339_prev_wd_reason, t1339_prev_chip_reset, t1339_prev_scratch[4];
void T1339SaveBoot();

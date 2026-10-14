#pragma once

// Per-packet ClearIrq(SyncwordValid) issued by uDMA, with no CPU in the per-packet path (t-0421 prototype).
//
// The SyncwordValid edge on LR2021 DIO5 (CC1314 DIO_13) is captured twice: GPT0A stores its time (mlat_capture),
// and GPT0B, fed from the same port event, requests uDMA channel 10. Channel 10 runs a memory scatter-gather list:
// NSS low, the 6-byte ClearIrq into the SPI0 TX FIFO, re-arm channel 11 and request it. Channel 11's list raises NSS,
// asks channel 12 to store the GPT0 counter in a ring (the clear time), pops the 6 received bytes from the SPI0 RX
// FIFO and re-arms channel 10.

// Bus sharing: the chain only runs while armed. Everything else that uses the SPI (drain sweep, IRQ-paced chain,
// synchronous commands) calls Disarm() first, which masks channel 10's request and waits for a chain in progress
// to finish. Arm() is called with the LR2021 idle (BUSY low); when DIO5 is already latched high (an edge came while
// disarmed), it starts the chain by software request.

#include <cstdint>

namespace mlat_clear_dma {

// mlat_capture must be running. clear_cmd: the 6-byte ClearIrq frame, kept valid until Stop().
void Start(const uint8_t* clear_cmd);
void Stop();
bool Running();

// Starts the chain on the next edge. Call with the bus free and BUSY low.
void Arm();
// Masks the trigger and waits for a chain in progress to end. False: it did not end in time (bus state unknown).
// reason: who takes the bus (bench statistics only).
bool Disarm(uint8_t reason = 0);
bool Armed();

// No effect since the GPT1A delay was dropped (kept for the bench knob).
void SetDelayTicks(uint32_t ticks);
uint32_t DelayTicks();

static constexpr uint16_t kRingLen = 64;
// Clear times (GPT0 counter just after NSS rose), oldest first. Same contract as mlat_capture::Read().
uint16_t ReadClears(uint32_t* out, bool* overrun);

struct Counters {
    uint32_t kicks = 0;              // Chains started by software request at Arm() (DIO5 latched while disarmed).
    uint32_t disarms = 0;            // Disarm() calls that masked an armed chain.
    uint32_t disarm_waits = 0;       // Disarm() found a chain in progress and waited for it.
    uint32_t disarm_timeouts = 0;    // Disarm() gave up waiting.
    uint32_t disarm_wait_max_cycles = 0;
    uint32_t rx_flushed_bytes = 0;   // Bytes left in the SPI0 RX FIFO after a chain (should stay 0).
    // Disarmed time until the next Arm(), by the reason of the disarm: count, total and max in CPU cycles.
    static constexpr uint8_t kReasons = 6;
    uint32_t off_count[kReasons] = {};
    uint64_t off_sum_cycles[kReasons] = {};
    uint32_t off_max_cycles[kReasons] = {};
};
const Counters& counters();
void ResetCounters();

}  // namespace mlat_clear_dma

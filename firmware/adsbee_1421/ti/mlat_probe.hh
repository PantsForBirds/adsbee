#pragma once
// MLAT bench probe (t-0354). Experiment only, never merged: edge-time capture of LR2021 DIO5 (CC1314 DIO_13) on
// GPT0A at 48 MHz, a raw LR2021 command passthrough, and a capture loop. Driven by AT+MLAT.

#include <cstddef>
#include <cstdint>

namespace mlat_probe {

struct Edge {
    uint32_t ticks;  // 24-bit GPT0A capture value (48 MHz, wraps every 349.5 ms).
    uint8_t level;   // DIO_13 level read in the capture ISR (1 = rising edge).
};

// Routes DIO_13 to GPT0A and starts the free-running 24-bit timer. Idempotent.
void Init();
bool IsInited();
// Current timer value (same counter the captures use).
uint32_t Now();
uint8_t PinLevel();
// Capture edge selection: 0 = rising, 1 = falling, 2 = both; +4 = one-shot (each capture needs Arm()).
void SetEdges(uint8_t mode);
// Clears a pending capture and enables the capture interrupt.
void Arm();
// Pops one captured edge. Returns false if none is pending.
bool PopEdge(Edge& edge);
uint32_t DroppedEdges();
uint32_t TotalEdges();
void Flush();

// Capture input: any CC1314 IOID (13 = LR2021 DIO5, 12 = LR2021 DIO6, 14 = LR2021 BUSY). Reboot to undo.
void SetSource(uint8_t ioid);
// With count_only the ISR keeps the statistics below and stores no edges.
void SetCountOnly(bool count_only);
struct Stats {
    uint32_t rising, falling;
    uint32_t high_min, high_max, low_min, low_max;  // Pulse widths in ticks (24-bit wrap).
    uint64_t high_sum, low_sum;
    uint32_t trig_pulses;
};
Stats GetStats();
// RX trigger loop (t-0406): DIO_12 (LR2021 DIO6, set to RX_TRIGGER on the chip first) driven by the CC1314.
// edges: bit 0 = pulse on a falling capture, bit 1 = on a rising capture. width = busy-loop count of the pulse.
void TrigEnable(bool enable);
void TrigLevel(bool high);
void TrigAuto(uint8_t edges, uint16_t width, uint16_t delay);

// True while AT+MLAT=HOLD,1 is in effect: the normal LR2021 servicing in ADSBee::UpdateLR2021 is frozen.
extern volatile bool hold;

}  // namespace mlat_probe

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
// Capture edge selection: 0 = rising, 1 = falling, 2 = both.
void SetEdges(uint8_t mode);
// Pops one captured edge. Returns false if none is pending.
bool PopEdge(Edge& edge);
uint32_t DroppedEdges();
uint32_t TotalEdges();
void Flush();

// True while AT+MLAT=HOLD,1 is in effect: the normal LR2021 servicing in ADSBee::UpdateLR2021 is frozen.
extern volatile bool hold;

}  // namespace mlat_probe

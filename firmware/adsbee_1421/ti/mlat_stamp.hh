#pragma once
// MLAT per-packet timestamp prototype (t-0364). Experiment only, not for release.
// LR2021 DIO5 (SyncwordValid IRQ) -> CC1314 DIO_13 -> GPT0A edge-time capture at 48 MHz. The LR2021 IRQ line is
// latched, so an edge is only seen for the first packet after the SyncwordValid bit was cleared.
//   Mode 0: stock (DIO5 not routed, no capture interrupt).
//   Mode 1: clear the bit over SPI right after every edge (when the bus is free).
//   Mode 2: sparse, no extra SPI: only the drain chain's GetAndClearIrq clears it.

#include <cstdint>

namespace mlat_stamp {

static constexpr uint32_t kTicksPerUs = 48;
static constexpr uint32_t kTickMask = 0x00FFFFFF;  // 24-bit capture counter, wraps every 349.5 ms.

struct Counters {
    volatile uint32_t edges = 0;           // Capture interrupts.
    volatile uint32_t ring_dropped = 0;    // Edges lost to a full ring.
    volatile uint32_t clears_posted = 0;   // Mode 1: ClearIrq frames posted from the capture interrupt.
    volatile uint32_t clears_deferred = 0; // Mode 1: bus was busy at the edge; cleared later.
    volatile uint32_t clears_thread = 0;   // Mode 1: deferred clears posted from the main loop.
    volatile uint32_t clears_stolen = 0;   // Mode 1: clears that interrupted a parked loop drain.
    volatile uint32_t isr_cycles = 0;      // CPU cycles spent in the capture interrupt.
    volatile uint32_t isr_max_cycles = 0;
    uint32_t stamped = 0;                  // Packets given a timestamp.
    uint32_t unstamped = 0;                // Packets left without one.
    uint32_t batches_ok = 0;
    uint32_t batches_mismatch = 0;         // Edge count did not fit the packet count: whole batch unstamped.
    uint32_t orphans = 0;                  // Edges dropped without a packet.
    uint32_t doubles = 0;                  // Two edges closer than a packet length (merged, packet unstamped).
    uint32_t rx_packets = 0;               // FIFO packets parsed.
    uint32_t valid = 0;                    // Valid decoded frames.
    uint32_t valid_stamped = 0;            // Valid decoded frames with a timestamp.
};
extern Counters c;

void Init();
void SetMode(uint8_t mode);  // Capture interrupt on for modes 1 and 2. The caller routes DIO5 on the LR2021.
uint8_t Mode();
uint32_t Now();  // Current 24-bit counter value.
void Flush();    // Drops all queued edges.
void ResetCounters();

// Pairs the next num_packets FIFO packets (oldest first) with captured edges. boundary_ticks: counter value when
// the batch was complete in the FIFO. exact_boundary: every packet complete by then is in the batch (loop drain
// level read); false for the 9-packet chain read. Writes one 64-bit 48 MHz timestamp per packet, 0 = unstamped.
// Log of the last valid frames for the bench (ICAO, downlink format, timestamp with bit 63 = unstamped).
struct LogEntry {
    uint32_t icao_df;  // ICAO in [23:0], downlink format in [31:24].
    uint64_t stamp;
};
static constexpr uint16_t kLogLen = 512;
void LogValid(uint32_t icao, uint16_t df, uint64_t stamp);
uint16_t LogCount();
const LogEntry& LogAt(uint16_t i);
void LogClear();

void PairBatch(uint32_t boundary_ticks, bool exact_boundary, uint16_t num_packets, uint64_t* stamps);

}  // namespace mlat_stamp

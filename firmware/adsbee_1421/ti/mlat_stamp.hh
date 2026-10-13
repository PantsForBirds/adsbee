#pragma once
// MLAT per-packet timestamp prototype v2 (t-0394). Experiment only, not for release.
//
// Edge: LR2021 DIO5 (SyncwordValid IRQ, latched) -> CC1314 DIO_13 -> GPT0A edge-time capture at 48 MHz, moved to a
// ring by uDMA (no interrupt per edge). The main-loop drain sweep clears the latch.
// Internal timestamp: GetTimestampValue(sync detected), read in the sweep, counted back from the request's NSS edge.
//
// A stamp is attached only when the packet is known: both sources mark one sync event, and a sync event's packet is
// complete in the FIFO between kDelayMin and kDelayMax later. A packet whose event may fall on either side of a FIFO
// read stays unstamped.
//   Mode 0: stock. 1: (c) edge. 2: (d) internal timestamp. 3: (c) + (d).

#include <cstdint>

namespace mlat_stamp {

static constexpr uint32_t kTicksPerUs = 48;
static constexpr uint32_t kTickMask = 0x00FFFFFF;  // 24-bit counter, wraps every 349.5 ms.
static constexpr uint8_t kModeEdge = 1, kModeTs = 2;

// Stamp word: bit 63 unstamped, [62:60] source (1 edge, 2 internal timestamp, 3 both agree), [59:56] packets in
// the FIFO read (15 = more, or a chain read), [55:0] time in 1/96 us.
static constexpr uint64_t kUnstamped = 1ull << 63;

struct Counters {
    uint32_t edges = 0;          // Captured DIO5 edges.
    uint32_t dma_rearms = 0;     // Ring halves handed back to the uDMA.
    uint32_t dma_overruns = 0;   // Both halves filled before a re-arm: capture restarted, edges lost.
    volatile uint32_t isr_edges = 0, isr_cycles = 0, isr_max_cycles = 0;  // Interrupt capture (fallback) only.
    uint32_t sweeps = 0;         // Completed loop sweeps (with or without packets).
    uint32_t reads1 = 0, reads2 = 0, reads3p = 0;  // FIFO reads by packet count.
    uint32_t chain_packets = 0;  // Packets read by the IRQ chain (never stamped).
    uint32_t clears = 0;         // ClearIrq frames sent by the sweep.
    uint32_t ts_reads = 0;       // GetTimestampValue reads.
    uint32_t stamped_edge = 0, stamped_ts = 0, stamped_both = 0;
    uint32_t un_none = 0;        // Unstamped: no event known for the packet.
    uint32_t un_ambiguous = 0;   // Unstamped: the event may belong to the read before or after.
    uint32_t un_multi = 0;       // Unstamped: several packets in the read and the event does not single one out.
    uint32_t rej_disagree = 0;   // Edge and internal timestamp point at different times: no stamp.
    uint32_t rej_orphan = 0;     // Event whose packet should be in the read, and the read is empty.
    uint32_t rej_extra = 0;      // More events than packets.
    uint32_t spin_cycles = 0, spin_max_cycles = 0, spins = 0;  // Busy-waits of the fast sweep.
    uint32_t frames = 0;         // SPI frames posted by the drains.
    uint32_t valid = 0, valid_stamped = 0;
    // Edge to "packet complete in the FIFO": bounds seen. lo = largest age of an event at a level read that did not
    // hold its packet; hi = smallest age of an event at a level read that did.
    uint32_t delay_lo_ticks = 0, delay_hi_ticks = 0xFFFFFFFF;
    int32_t agree_min = 0x7FFFFFFF, agree_max = -0x7FFFFFFF;  // Edge minus internal timestamp, 1/96 us.
};
extern Counters c;
extern uint32_t delay_min_ticks, delay_max_ticks;  // kDelayMin / kDelayMax, settable for tuning.
extern int32_t agree_center, agree_tol;            // Expected edge - timestamp difference and tolerance, 1/96 us.

bool SetMode(uint8_t mode, bool isr_capture);  // Starts or stops the capture. The caller configures the LR2021.
uint8_t Mode();
bool IsrCapture();
bool PinHigh();  // DIO5 level: the latch is set.
void ResetCounters();
void Poll();     // Main loop: moves captured edges out of the ring, re-arms the uDMA.

bool Inited();
uint32_t NowSlow();
// 24-bit counter now; 0 before the capture was first started (GPT0 has no clock then).
inline uint32_t Now() { return Inited() ? NowSlow() : 0; }

// A FIFO read was made whose contents are not paired (chain read, flush): nothing before now may be stamped later.
void Boundary();

// One loop sweep. num_packets whole packets were read (0: nothing read). level_a_ticks / level_b_ticks: the FIFO
// level was sampled between these. has_ts: ts_value (32 MHz ticks back from ts_nss_ticks) is valid.
// Writes one stamp word per packet.
struct Sweep {
    uint32_t level_a_ticks = 0, level_b_ticks = 0;
    bool has_ts = false;
    uint32_t ts_nss_ticks = 0, ts_value = 0;
};
void PairSweep(const Sweep& sweep, uint16_t num_packets, uint64_t* stamps);

// Per-sweep debug log (sweeps with packets or pending edges): ticks.
struct DebugEntry {
    uint8_t k, pending;
    uint16_t ab;             // level_b - level_a.
    int32_t age_a;           // level_a - oldest pending edge.
    int32_t since_boundary;  // level_a - boundary.
    int32_t ts_age;          // level_a - timestamp event.
};
static constexpr uint16_t kDbgLen = 200;
extern DebugEntry dbg[kDbgLen];
extern uint16_t dbg_count;

struct LogEntry {
    uint32_t icao_df;  // ICAO in [23:0], downlink format in [31:24].
    uint64_t stamp;
};
static constexpr uint16_t kLogLen = 512;
void LogValid(uint32_t icao, uint16_t df, uint64_t stamp);
uint16_t LogCount();
const LogEntry& LogAt(uint16_t i);
void LogClear();

}  // namespace mlat_stamp

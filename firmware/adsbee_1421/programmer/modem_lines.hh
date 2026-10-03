#pragma once

#include <stdint.h>

// Maps host modem-control lines (USB CDC SET_CONTROL_LINE_STATE) to the SYNC level and RESET_N pulses, like a
// USB-UART adapter (asserted line = pin low). Pure logic, host-tested in host_test/test_modem_lines.cc.
//   RTS asserted    -> SYNC low (awake)
//   RTS deasserted  -> SYNC high (sleep / backdoor armed), only while DTR is asserted
//   DTR assert edge -> RESET_N pulse, with SYNC latched from RTS at the edge
//
// Hosts drop DTR and RTS together when the port closes, so SYNC high requires DTR asserted and a closed port leaves
// the module awake. Exceptions, so a backdoor entry works from DTR low:
//   - While a reset is pending, SYNC holds the level latched at the DTR edge.
//   - For kBackdoorHoldMs after a reset with RTS deasserted, SYNC stays high whatever DTR does, so the boot ROM
//     samples it high.
class ModemLines {
   public:
    // Long enough for the CC13x4 boot ROM to sample the backdoor pin after RESET_N is released.
    static constexpr uint32_t kBackdoorHoldMs = 250;

    // Start of a pass-through session. SYNC stays low until the host changes a line.
    void Start();

    // Host line state outside pass-through: remembered for edge detection, never acted on.
    void Track(bool dtr, bool rts);

    // Host line state during pass-through.
    void OnLineState(bool dtr, bool rts);

    // A DTR assert edge has requested a reset pulse the caller has not performed yet.
    bool reset_pending() const { return reset_pending_; }
    // SYNC level (true = high) the pending reset was requested with.
    bool reset_sync_high() const { return reset_sync_high_; }
    // The caller performed the pending reset; RESET_N was released at now_ms.
    void OnResetDone(uint32_t now_ms);

    // Level to drive on SYNC (true = high). Call at least every few seconds so the hold expires before the ms
    // counter wraps.
    bool SyncHigh(uint32_t now_ms);

   private:
    bool dtr_ = false;
    bool rts_ = false;
    bool following_ = false;  // A line changed during this session: SYNC follows the lines.
    bool reset_pending_ = false;
    bool reset_sync_high_ = false;
    bool holding_ = false;  // Backdoor hold running since hold_start_ms_.
    uint32_t hold_start_ms_ = 0;
};

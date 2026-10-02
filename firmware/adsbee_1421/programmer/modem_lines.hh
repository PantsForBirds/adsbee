#pragma once

#include <stdint.h>

// Host modem-control lines (CDC SET_CONTROL_LINE_STATE) -> SYNC level and RESET_N pulses for the
// pass-through bridge. Pure logic with no SDK dependencies so it can be host-tested
// (host_test/test_modem_lines.cc); bridge.cc feeds it the TinyUSB callbacks and the clock.
//
// Adapter emulation (an asserted modem-control bit drives the physical pin low):
//   RTS asserted            -> SYNC low (awake)
//   RTS deasserted          -> SYNC high (sleep request / backdoor armed), but only while the
//                              port is open, see below
//   DTR assert edge         -> RESET_N pulse, with SYNC latched at the edge's RTS level
//
// Port close: Linux (HUPCL, the default), macOS and Windows drop DTR and RTS together when the
// port closes, which is RTS deasserted and would put the module to sleep until the next open.
// "Both lines low" can't be told apart from a host that is mid-way through a backdoor entry (the
// web console and the README's pyserial snippet deassert RTS with DTR low, then pulse DTR), and no
// other USB event marks a close. DTR is the CDC "terminal present" bit, though
// (tud_cdc_connected() is DTR), so SYNC high requires DTR asserted, except:
//   - While a reset is pending, SYNC holds the level latched at the DTR edge, so an edge and its
//     release arriving together still reset into what the edge asked for.
//   - For kBackdoorHoldMs after a reset pulse taken with RTS deasserted, SYNC stays high whatever
//     DTR does (asserting RTS still drops it), so the boot ROM samples it high. After that the ROM
//     bootloader is running and no longer looks at SYNC; the next DTR edge re-latches it.
// So a closed port leaves the module awake, a backdoor entry works from DTR low, and an open port
// (DTR asserted, as every terminal leaves it) can still put the module to sleep by deasserting RTS.
class ModemLines {
   public:
    // Long enough for the CC13x4 boot ROM to sample the backdoor pin after RESET_N is released.
    static constexpr uint32_t kBackdoorHoldMs = 250;

    // Start of a pass-through session. Line states recorded before this (Track()) only seed
    // edge detection: SYNC stays low until the host changes a line during the session.
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

    // Level to drive on SYNC (true = high). Call at least every few seconds so an expired hold
    // is cleared well before the ms counter wraps.
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

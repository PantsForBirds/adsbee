#pragma once

#include <stddef.h>
#include <stdint.h>

// Bounded writes to the USB host. With DTR high, TinyUSB's TX FIFO only drains when the host reads; a host that stops
// reading (a stopped reader, or a port closed with HUPCL off) would hold the main loop forever, and with it host data to
// the module and the line-coding requests that carry the reboot-to-BOOTSEL baud. Pure logic, host-tested in
// host_test/test_cdc_tx.cc.
class CdcTx {
   public:
    // No room in the FIFO for this long: the host isn't reading. From then on, writes drop instead of waiting until the
    // host takes bytes again.
    static constexpr uint32_t kStallMs = 100;

    // Writes `len` bytes, waiting while the host makes room, but never kStallMs without progress, and not at all while
    // the host isn't reading or isn't connected. Returns the bytes written; the rest are dropped and counted. Port:
    //   uint32_t Write(const uint8_t* data, uint32_t len)  bytes the FIFO took
    //   void Wait()                                        flush and service USB once
    //   bool Connected()                                   USB up and DTR asserted
    //   uint32_t NowMs()
    template <class Port>
    size_t Write(Port& port, const uint8_t* data, size_t len) {
        size_t written = 0;
        bool waiting = false;
        uint32_t wait_start_ms = 0;
        while (written < len) {
            uint32_t n = port.Write(data + written, (uint32_t)(len - written));
            if (n > 0) {
                written += n;
                stalled_ = false;
                waiting = false;
                continue;
            }
            if (stalled_ || !port.Connected()) break;
            uint32_t now_ms = port.NowMs();
            if (!waiting) {
                waiting = true;
                wait_start_ms = now_ms;
            } else if (now_ms - wait_start_ms >= kStallMs) {
                stalled_ = true;
                break;
            }
            port.Wait();
        }
        dropped_ += (uint32_t)(len - written);
        return written;
    }

    // Ends a stall once the host has read from the FIFO (`fifo_free` bytes free), even with nothing new to write.
    void OnFifoFree(uint32_t fifo_free) {
        if (fifo_free > 0) stalled_ = false;
    }

    // The host stopped reading; writes drop until it takes bytes again.
    bool stalled() const { return stalled_; }

    // Bytes dropped since the last call.
    uint32_t TakeDropped() {
        uint32_t dropped = dropped_;
        dropped_ = 0;
        return dropped;
    }

   private:
    bool stalled_ = false;
    uint32_t dropped_ = 0;
};

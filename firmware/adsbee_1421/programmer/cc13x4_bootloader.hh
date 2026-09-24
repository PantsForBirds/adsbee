#pragma once

#include <stddef.h>
#include <stdint.h>

// CC13x4 factory ROM serial bootloader protocol (TRM SWCU194 chapter 10) over target_uart.
// Port of CC13x4Bootloader in software/adsbee_1421_flasher/adsbee_1421_flasher.py.
class Cc13x4Bootloader {
   public:
    static const size_t kMaxDataChunk = 248;  // SEND_DATA payload cap (252) rounded to flash words.

    bool SyncBaud(uint32_t window_ms = 300);  // 0x55 0x55 auto-baud; true if the ROM ACKs.
    bool Ping();
    bool GetChipId(uint32_t* chip_id_out);
    bool BankErase();
    bool SectorErase(uint32_t addr);
    bool Download(uint32_t addr, uint32_t num_bytes);
    bool SendData(const uint8_t* data, size_t len);
    bool Crc32(uint32_t addr, uint32_t num_bytes, uint32_t* crc_out);
    bool ResetCmd();

    const char* LastError() const { return last_error_; }

   private:
    enum Command : uint8_t {
        kCmdPing = 0x20,
        kCmdDownload = 0x21,
        kCmdGetStatus = 0x23,
        kCmdSendData = 0x24,
        kCmdReset = 0x25,
        kCmdSectorErase = 0x26,
        kCmdCrc32 = 0x27,
        kCmdGetChipId = 0x28,
        kCmdBankErase = 0x2C,
    };
    static const uint8_t kAck = 0xCC;
    static const uint8_t kNack = 0x33;
    static const uint8_t kStatusSuccess = 0x40;

    // 1 = ACK, 0 = NACK, -1 = timeout/garbage. Skips 0x00 filler bytes.
    int WaitAck(uint32_t timeout_ms);
    bool SendPacket(const uint8_t* payload, size_t len, uint32_t ack_timeout_ms, int retries = 3);
    // Returns payload length or -1; ACKs/NACKs the packet per its checksum.
    int RecvPacket(uint8_t* buf, size_t max_len, uint32_t timeout_ms);
    bool CheckStatus(const char* context);

    const char* last_error_ = "";
};

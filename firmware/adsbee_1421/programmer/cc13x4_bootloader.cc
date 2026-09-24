#include "cc13x4_bootloader.hh"

#include <string.h>

#include "target_uart.hh"

static void PackBe32(uint8_t* out, uint32_t value) {
    out[0] = value >> 24;
    out[1] = value >> 16;
    out[2] = value >> 8;
    out[3] = value;
}

int Cc13x4Bootloader::WaitAck(uint32_t timeout_ms) {
    while (true) {
        int byte = TargetUartReadByteTimeout(timeout_ms);
        if (byte < 0) {
            last_error_ = "timed out waiting for ACK";
            return -1;
        }
        if (byte == 0x00) continue;  // Filler bytes are allowed before the ACK.
        if (byte == kAck) return 1;
        if (byte == kNack) return 0;
        last_error_ = "unexpected byte while waiting for ACK";
        return -1;
    }
}

bool Cc13x4Bootloader::SendPacket(const uint8_t* payload, size_t len, uint32_t ack_timeout_ms,
                                  int retries) {
    uint8_t packet[2 + kMaxDataChunk + 16];
    packet[0] = (uint8_t)(len + 2);
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) checksum += payload[i];
    packet[1] = checksum;
    memcpy(packet + 2, payload, len);

    for (int attempt = 0; attempt < retries; attempt++) {
        TargetUartWriteBlocking(packet, len + 2);
        int ack = WaitAck(ack_timeout_ms);
        if (ack == 1) return true;
        if (ack < 0) return false;  // Timeout: retrying blind would desync framing.
    }
    last_error_ = "command NACKed repeatedly";
    return false;
}

int Cc13x4Bootloader::RecvPacket(uint8_t* buf, size_t max_len, uint32_t timeout_ms) {
    int size = 0;
    while (size == 0) {  // Zero bytes are allowed filler before the size byte.
        size = TargetUartReadByteTimeout(timeout_ms);
        if (size < 0) {
            last_error_ = "timed out waiting for response packet";
            return -1;
        }
    }
    int checksum = TargetUartReadByteTimeout(timeout_ms);
    if (checksum < 0) {
        last_error_ = "timed out reading response checksum";
        return -1;
    }
    size_t data_len = (size_t)size - 2;
    if (data_len > max_len) {
        last_error_ = "response packet too large";
        return -1;
    }
    uint8_t sum = 0;
    for (size_t i = 0; i < data_len; i++) {
        int byte = TargetUartReadByteTimeout(timeout_ms);
        if (byte < 0) {
            last_error_ = "timed out reading response data";
            return -1;
        }
        buf[i] = (uint8_t)byte;
        sum += (uint8_t)byte;
    }
    uint8_t reply = (sum == checksum) ? kAck : kNack;
    TargetUartWriteBlocking(&reply, 1);
    if (reply == kNack) {
        last_error_ = "response packet checksum mismatch";
        return -1;
    }
    return (int)data_len;
}

bool Cc13x4Bootloader::CheckStatus(const char* context) {
    uint8_t cmd = kCmdGetStatus;
    if (!SendPacket(&cmd, 1, 1000)) return false;
    uint8_t status;
    if (RecvPacket(&status, 1, 1000) != 1) return false;
    if (status != kStatusSuccess) {
        last_error_ = context;
        return false;
    }
    return true;
}

bool Cc13x4Bootloader::SyncBaud(uint32_t window_ms) {
    TargetUartFlushInput();
    const uint8_t pattern[2] = {0x55, 0x55};
    TargetUartWriteBlocking(pattern, 2);
    return WaitAck(window_ms) == 1;
}

bool Cc13x4Bootloader::Ping() {
    uint8_t cmd = kCmdPing;
    return SendPacket(&cmd, 1, 1000);
}

bool Cc13x4Bootloader::GetChipId(uint32_t* chip_id_out) {
    uint8_t cmd = kCmdGetChipId;
    if (!SendPacket(&cmd, 1, 1000)) return false;
    uint8_t reply[4];
    if (RecvPacket(reply, sizeof(reply), 1000) != 4) return false;
    *chip_id_out = ((uint32_t)reply[0] << 24) | ((uint32_t)reply[1] << 16) |
                   ((uint32_t)reply[2] << 8) | reply[3];
    return true;
}

bool Cc13x4Bootloader::BankErase() {
    uint8_t cmd = kCmdBankErase;
    if (!SendPacket(&cmd, 1, 20000)) return false;
    return CheckStatus("bank erase failed");
}

bool Cc13x4Bootloader::SectorErase(uint32_t addr) {
    uint8_t payload[5] = {kCmdSectorErase};
    PackBe32(payload + 1, addr);
    if (!SendPacket(payload, sizeof(payload), 5000)) return false;
    return CheckStatus("sector erase failed");
}

bool Cc13x4Bootloader::Download(uint32_t addr, uint32_t num_bytes) {
    uint8_t payload[9] = {kCmdDownload};
    PackBe32(payload + 1, addr);
    PackBe32(payload + 5, num_bytes);
    if (!SendPacket(payload, sizeof(payload), 1000)) return false;
    return CheckStatus("download setup failed");
}

bool Cc13x4Bootloader::SendData(const uint8_t* data, size_t len) {
    uint8_t payload[1 + kMaxDataChunk];
    payload[0] = kCmdSendData;
    memcpy(payload + 1, data, len);
    if (!SendPacket(payload, 1 + len, 5000)) return false;
    return CheckStatus("program data failed");
}

bool Cc13x4Bootloader::Crc32(uint32_t addr, uint32_t num_bytes, uint32_t* crc_out) {
    uint8_t payload[13] = {kCmdCrc32};
    PackBe32(payload + 1, addr);
    PackBe32(payload + 5, num_bytes);
    PackBe32(payload + 9, 0);  // Repeat count 0: read each location once.
    if (!SendPacket(payload, sizeof(payload), 10000)) return false;
    uint8_t reply[4];
    if (RecvPacket(reply, sizeof(reply), 10000) != 4) return false;
    *crc_out = ((uint32_t)reply[0] << 24) | ((uint32_t)reply[1] << 16) |
               ((uint32_t)reply[2] << 8) | reply[3];
    return true;
}

bool Cc13x4Bootloader::ResetCmd() {
    uint8_t cmd = kCmdReset;
    return SendPacket(&cmd, 1, 1000);  // ACK only, no GET_STATUS (the chip resets).
}

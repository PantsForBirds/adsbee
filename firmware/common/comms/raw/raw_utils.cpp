#include "raw_utils.hh"

#include "stdio.h"

uint16_t BuildRawModeSFrame(const RawModeSPacket& packet, char* raw_frame_buf) {
    char data_buf[RawModeSPacket::kExtendedSquitterPacketLenBits / kBitsPerNibble + 1] = {'\0'};
    packet.PrintBuffer(data_buf, sizeof(data_buf));
    return snprintf(raw_frame_buf, kRawModeSFrameMaxNumChars, "#MDS*%s;(%d,%d,%d,%016llX)\r\n", data_buf, packet.source,
                    packet.sigs_dbm, packet.sigq_db,
                    (packet.flags & RawModeSPacket::kFlagNoMlatTimestamp) ? 0 : packet.mlat_48mhz_64bit_counts);
}

uint16_t BuildRawUATADSBFrame(const RawUATADSBPacket& packet, char* raw_frame_buf) {
    char data_buf[RawUATADSBPacket::kADSBMessageMaxSizeBytes * kNibblesPerByte + 1] = {'\0'};
    // Write bytes as uppercase.
    ByteBufferToHexString(data_buf, packet.buffer, packet.buffer_len_bytes, true);
    return snprintf(raw_frame_buf, kRawUATADSBFrameMaxNumChars, "#UAT*-%s;(%d,%d,%016llX)\r\n", data_buf,
                    packet.sigs_dbm, packet.sigq_bits, packet.mlat_48mhz_64bit_counts);
}

uint16_t BuildRawUATUplinkFrame(const RawUATUplinkPacket& packet, char* raw_frame_buf) {
    char data_buf[RawUATUplinkPacket::kUplinkMessageNumBytes * kNibblesPerByte + 1] = {'\0'};
    // Write bytes as uppercase.
    ByteBufferToHexString(data_buf, packet.encoded_message, packet.encoded_message_len_bytes, true);
    return snprintf(raw_frame_buf, kRawUATUplinkFrameMaxNumChars, "#UAT*+%s;(%d,%d,%016llX)\r\n", data_buf,
                    packet.sigs_dbm, packet.sigq_bits, packet.mlat_48mhz_64bit_counts);
}

uint16_t BuildRawADSLFrame(const RawADSLPacket& packet, char* raw_frame_buf) {
    char data_buf[RawADSLPacket::kMaxPacketLenBytes * kNibblesPerByte + 1] = {'\0'};
    // The shared slot holds up to 48 bytes; ADS-L packets are at most 32.
    uint16_t len_bytes = packet.buffer_len_bytes < RawADSLPacket::kMaxPacketLenBytes
                             ? packet.buffer_len_bytes
                             : RawADSLPacket::kMaxPacketLenBytes;
    ByteBufferToHexString(data_buf, packet.buffer, len_bytes, true);
    return snprintf(raw_frame_buf, kRawADSLFrameMaxNumChars, "#ADL*%s;(%d,%d,%d,%016llX)\r\n", data_buf,
                    packet.sigs_dbm, packet.sigq_bits, packet.channel == RawADSLPacket::kChannelUnknown ? -1 : packet.channel,
                    packet.mlat_48mhz_64bit_counts);
}

uint16_t BuildRawSubGHzFrame(const RawUATADSBPacket& packet, char* raw_frame_buf) {
    if (packet.protocol == RawUATADSBPacket::kProtocolADSL) {
        return BuildRawADSLFrame(RawADSLPacket(packet), raw_frame_buf);
    }
    return BuildRawUATADSBFrame(packet, raw_frame_buf);
}

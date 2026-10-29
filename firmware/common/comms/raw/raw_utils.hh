#pragma once

#include "adsl_packet.hh"
#include "mode_s_packet.hh"
#include "stdint.h"
#include "uat_packet.hh"

static const uint16_t kRawModeSFrameMaxNumChars = RawModeSPacket::kExtendedSquitterPacketLenBytes * 2 + 50;
static const uint16_t kRawUATADSBFrameMaxNumChars = RawUATADSBPacket::kADSBMessageMaxSizeBytes * 2 + 50;
static const uint16_t kRawUATUplinkFrameMaxNumChars = RawUATUplinkPacket::kUplinkMessageNumBytes * 2 + 50;

uint16_t BuildRawModeSFrame(const RawModeSPacket& packet, char* raw_frame_buf);
uint16_t BuildRawUATADSBFrame(const RawUATADSBPacket& packet, char* raw_frame_buf);
uint16_t BuildRawUATUplinkFrame(const RawUATUplinkPacket& packet, char* raw_frame_buf);
static const uint16_t kRawADSLFrameMaxNumChars = RawADSLPacket::kMaxPacketLenBytes * 2 + 50;
// "#ADL*<length byte .. CRC hex>;(<rssi dBm>,<repaired chips>,<channel 0=868.2 1=868.4>,<timestamp>)"
uint16_t BuildRawADSLFrame(const RawADSLPacket& packet, char* raw_frame_buf);
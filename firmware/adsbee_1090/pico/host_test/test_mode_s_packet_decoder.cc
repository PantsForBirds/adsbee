#include "gtest/gtest.h"
#include "mode_s_packet_decoder.hh"
#include "settings.hh"

static constexpr uint64_t kCountsPerMs = 48000;

TEST(ModeSPacketDecoder, HandleNoBitErrors) {
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});

    RawModeSPacket raw_packet((const char*)"8D40621D58C382D690C8AC2863A7");
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);

    DecodedModeSPacket decoded_packet;
    EXPECT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet));
    EXPECT_EQ(decoded_packet.icao_address, 0x40621Du);
    EXPECT_EQ(decoded_packet.crc_syndrome, 0u);
}

TEST(ModeSPacketDecoder, HandleSingleBitError) {
    ModeSPacketDecoder decoder_no_corrections(
        ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = false});
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});

    RawModeSPacket raw_packet((const char*)"8D40621D58C382D690C8AC2863A6");
    DecodedModeSPacket uncorrected(raw_packet);
    EXPECT_FALSE(uncorrected.is_valid);
    EXPECT_NE(uncorrected.crc_syndrome, 0u);

    decoder_no_corrections.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder_no_corrections.UpdateDecoderLoop();
    EXPECT_EQ(decoder_no_corrections.decoded_mode_s_packet_out_queue.Length(), 0);

    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);

    DecodedModeSPacket decoded_packet;
    EXPECT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet));
    EXPECT_TRUE(decoded_packet.is_valid);
    EXPECT_EQ(decoded_packet.icao_address, 0x40621Du);
    EXPECT_EQ(decoded_packet.crc_syndrome, 0u);

    uint16_t bit_flip_index = 0;
    EXPECT_TRUE(decoder.decoded_mode_s_packet_bit_flip_locations_out_queue.Dequeue(bit_flip_index));
    EXPECT_EQ(bit_flip_index, 111);  // Last bit of the packet was flipped (A7 -> A6).
}

TEST(ModeSPacketDecoder, DontCorrectBitErrorsInDownlinkFormat) {
    // Flipping the MSb of the DF field (bit index 0) turns a DF=17 into a DF=1. Only packets received as DF=17/18 are
    // accepted as ADS-B (DO-260B 2.2.4.3.4.7.3.a), so this must not be "corrected" back into a DF=17.
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    RawModeSPacket raw_packet((const char*)"0D40621D58C382D690C8AC2863A7");  // 8D -> 0D.
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 0);
    EXPECT_EQ(decoder.decoded_mode_s_packet_bit_flip_locations_out_queue.Length(), 0);
}

TEST(ModeSPacketDecoder, OnlyCorrectExtendedSquitters) {
    // DF=19 packet with a valid CRC (9D40621D58C382D690C8AC50B818) and its last bit flipped. Single bit correction is
    // only applied to DF=17/18.
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    RawModeSPacket raw_packet((const char*)"9D40621D58C382D690C8AC50B819");
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 0);

    // The uncorrupted DF=19 packet still goes through.
    decoder.raw_mode_s_packet_in_queue.Enqueue(RawModeSPacket((const char*)"9D40621D58C382D690C8AC50B818"));
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);
}

TEST(ModeSPacketDecoder, RejectDuplicateMessages) {
    // Packet decoder should not re-process the same message if it's caught by multiple state machines.
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    RawModeSPacket raw_packet((const char*)"8D40621D58C382D690C8AC2863A7");
    raw_packet.source = 0;
    raw_packet.mlat_48mhz_64bit_counts = 123456;
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);
    DecodedModeSPacket decoded_packet;
    EXPECT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet));
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 0);
    EXPECT_EQ(decoded_packet.icao_address, 0x40621Du);

    // Enqueue the same packet again from another source within the duplicate window, it should not be re-processed.
    raw_packet.source = 1;
    raw_packet.mlat_48mhz_64bit_counts = 123456 + kCountsPerMs / 2;  // 0.5ms later.
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 0);

    // A timestamp slightly earlier than the first copy (e.g. different jitter correction) is also a duplicate.
    raw_packet.source = 2;
    raw_packet.mlat_48mhz_64bit_counts = 123456 - 10;
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 0);

    // Enqueue the same packet again after the duplicate window, it should be re-processed (e.g. a repeated
    // interrogation reply is a distinct transmission).
    raw_packet.source = 1;
    raw_packet.mlat_48mhz_64bit_counts = 123456 + ModeSPacketDecoder::kDuplicatePacketWindow48MHzCounts;
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);
    EXPECT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet));
    EXPECT_EQ(decoded_packet.icao_address, 0x40621Du);
    EXPECT_EQ(decoded_packet.raw.source, 1);  // Should have the source.
}

TEST(ModeSPacketDecoder, AcceptDistinctPacketsFromSameICAO) {
    // Different messages from the same aircraft close together in time must both be passed through.
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    RawModeSPacket even_packet((const char*)"8D40621D58C382D690C8AC2863A7");
    even_packet.source = 0;
    even_packet.mlat_48mhz_64bit_counts = 1000000;
    RawModeSPacket odd_packet((const char*)"8D40621D58C386435CC412692AD6");
    odd_packet.source = 1;
    odd_packet.mlat_48mhz_64bit_counts = 1000000 + kCountsPerMs / 10;  // 100us later.

    decoder.raw_mode_s_packet_in_queue.Enqueue(even_packet);
    decoder.raw_mode_s_packet_in_queue.Enqueue(odd_packet);
    decoder.UpdateDecoderLoop();
    ASSERT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 2);

    DecodedModeSPacket decoded_packet;
    EXPECT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet));
    EXPECT_TRUE(decoded_packet.is_valid);
    EXPECT_EQ(decoded_packet.icao_address, 0x40621Du);
    EXPECT_EQ(decoded_packet.raw.source, 0);
    EXPECT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet));
    EXPECT_TRUE(decoded_packet.is_valid);
    EXPECT_EQ(decoded_packet.icao_address, 0x40621Du);
    EXPECT_EQ(decoded_packet.raw.source, 1);
}

TEST(ModeSPacketDecoder, RejectCorrectedDuplicate) {
    // A clean copy from one state machine followed by a single-bit-corrupted copy of the same transmission from
    // another state machine should only produce one packet, since the corrected copy is identical.
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    RawModeSPacket clean_packet((const char*)"8D40621D58C382D690C8AC2863A7");
    clean_packet.source = 0;
    clean_packet.mlat_48mhz_64bit_counts = 5000000;
    RawModeSPacket corrupted_packet((const char*)"8D40621D58C382D690C8AC2863A6");
    corrupted_packet.source = 1;
    corrupted_packet.mlat_48mhz_64bit_counts = 5000000 + 5;

    decoder.raw_mode_s_packet_in_queue.Enqueue(clean_packet);
    decoder.raw_mode_s_packet_in_queue.Enqueue(corrupted_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);
}

TEST(ModeSPacketDecoder, DebugMessagesGatedOnLogLevel) {
    SettingsManager::LogLevel original_log_level = settings_manager.settings.log_level;
    RawModeSPacket raw_packet((const char*)"8D40621D58C382D690C8AC2863A7");

    // Below kInfo: no per-packet debug messages are formatted or queued.
    settings_manager.settings.log_level = SettingsManager::LogLevel::kWarnings;
    ModeSPacketDecoder quiet_decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    quiet_decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    quiet_decoder.UpdateDecoderLoop();
    EXPECT_EQ(quiet_decoder.decoded_mode_s_packet_out_queue.Length(), 1);
    EXPECT_EQ(quiet_decoder.debug_message_out_queue.Length(), 0);

    // At kInfo: one debug message per packet.
    settings_manager.settings.log_level = SettingsManager::LogLevel::kInfo;
    ModeSPacketDecoder verbose_decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    verbose_decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);
    verbose_decoder.UpdateDecoderLoop();
    EXPECT_EQ(verbose_decoder.decoded_mode_s_packet_out_queue.Length(), 1);
    ASSERT_EQ(verbose_decoder.debug_message_out_queue.Length(), 1);
    ModeSPacketDecoder::DebugMessage message;
    EXPECT_TRUE(verbose_decoder.debug_message_out_queue.Dequeue(message));
    EXPECT_NE(strstr(message.message, "VALID"), nullptr);
    EXPECT_NE(strstr(message.message, "8D40621D58C382D690C8AC2863A7"), nullptr);

    settings_manager.settings.log_level = original_log_level;
}

TEST(ModeSPacketDecoder, ForwardAllCallReplyWithInterrogatorCode) {
    // DF=11 replies to interrogators with a nonzero II code are forwarded for ICAO confirmation, not dropped.
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    decoder.raw_mode_s_packet_in_queue.Enqueue(RawModeSPacket((const char*)"5D7C0B6DB05073"));
    decoder.UpdateDecoderLoop();
    ASSERT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);
    DecodedModeSPacket decoded_packet;
    EXPECT_TRUE(decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet));
    EXPECT_TRUE(decoded_packet.is_address_parity);
    EXPECT_EQ(decoded_packet.icao_address, 0x7C0B6Du);
}

TEST(ModeSPacketDecoder, RejectDuplicateSquittersWithTrailingBits) {
    // A 56-bit packet read out of the demodulator as 3 words keeps the bits received after the end of the message in
    // the low byte of its second word. They aren't part of the packet, so they mustn't defeat the duplicate filter.
    ModeSPacketDecoder decoder(ModeSPacketDecoder::PacketDecoderConfig{.enable_1090_error_correction = true});
    RawModeSPacket raw_packet((const char*)"5D7C0B6DB05076");
    ASSERT_EQ(raw_packet.buffer_len_bytes, 7);  // 56 bits.
    raw_packet.source = 0;
    raw_packet.mlat_48mhz_64bit_counts = 123456;
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet);

    RawModeSPacket raw_packet_trailing_bits = raw_packet;
    raw_packet_trailing_bits.buffer[1] |= 0xA5;  // Bits 56-63.
    raw_packet_trailing_bits.source = 1;
    raw_packet_trailing_bits.mlat_48mhz_64bit_counts = 123456 + kCountsPerMs / 4;
    decoder.raw_mode_s_packet_in_queue.Enqueue(raw_packet_trailing_bits);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 1);

    // A packet that differs within its 56 bits is not a duplicate.
    RawModeSPacket different_packet((const char*)"5D7C0B6DB05073");
    different_packet.source = 2;
    different_packet.mlat_48mhz_64bit_counts = 123456 + kCountsPerMs / 2;
    decoder.raw_mode_s_packet_in_queue.Enqueue(different_packet);
    decoder.UpdateDecoderLoop();
    EXPECT_EQ(decoder.decoded_mode_s_packet_out_queue.Length(), 2);
}

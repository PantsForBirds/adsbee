// On-target tests of the 1090 receive path (AT+TEST): its CPU cost in CPU cycles (utils/cycle_counter.hh), for
// synthetic DF17-mode captures through ADSBee::ParseLR2021RxFifo and PacketDecoder, and the receiver config
// fallback. Every cycle measurement runs with interrupts off, so min/avg/max are the functions' own cost.
#include <ti/drivers/dpl/HwiP.h>

#include "adsbee.hh"
#include "crc.hh"
#include "cycle_counter.hh"
#include "hardware_unit_tests.hh"
#include "lr2021_ook_adsb.hh"
#include "packet_decoder.hh"

class ADSBeeTestAccessor {
   public:
    static void Parse(const uint8_t* buf, uint16_t len) { adsbee.ParseLR2021RxFifo(buf, len, 0); }
    // Changes only what the parser and the decoder read; the radio keeps its configuration.
    static SettingsManager::R1090PreambleMode SwapMode(SettingsManager::R1090PreambleMode mode) {
        const SettingsManager::R1090PreambleMode old = adsbee.r1090_preamble_mode_;
        adsbee.r1090_preamble_mode_ = mode;
        return old;
    }
    static uint32_t& FramesSinceValid() { return adsbee.lr2021_frames_since_valid_; }
    static bool ApplyReceiverConfig() { return adsbee.ApplyReceiverConfig(); }
    static bool ReceiverConfigOk() { return adsbee.receiver_config_ok_; }
};

namespace {

static constexpr uint16_t kCaptureLenBytes = LR2021::kOokDF17PacketRxLenBytes;

struct CycleStats {
    uint32_t min = UINT32_MAX, max = 0, n = 0;
    uint64_t sum = 0;
    void Add(uint32_t c) {
        min = c < min ? c : min;
        max = c > max ? c : max;
        sum += c;
        n++;
    }
    void Print(const char* name) const {
        CONSOLE_PRINTF("RXCPU %-34s n=%4lu min=%6lu avg=%6lu max=%6lu cycles (max %lu us)\r\n", name,
                       (unsigned long)n, (unsigned long)min, (unsigned long)(n ? sum / n : 0), (unsigned long)max,
                       (unsigned long)(max / (CycleCounter::kCpuHz / 1000000u)));
    }
};

uint32_t rng_state = 1090;
uint32_t Rand() {  // xorshift32
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

bool GetBit(const uint8_t* buf, uint16_t i) { return (buf[i / 8] >> (7 - i % 8)) & 1u; }
void PutBit(uint8_t* buf, uint16_t i, bool v) {
    buf[i / 8] = static_cast<uint8_t>(v ? (buf[i / 8] | (0x80u >> (i % 8))) : (buf[i / 8] & ~(0x80u >> (i % 8))));
}

// What the DF17 detector captures from a valid DF17 frame with random ICAO and ME: message bits 4-111, then
// the capture slop (random bits).
void ValidCapture(uint8_t* cap) {
    uint8_t f[14];
    f[0] = static_cast<uint8_t>((17u << 3) | 5u);
    for (uint8_t k = 1; k < 11; k++) f[k] = static_cast<uint8_t>(Rand());
    const uint32_t p = crc24(f, 11);
    f[11] = static_cast<uint8_t>(p >> 16);
    f[12] = static_cast<uint8_t>(p >> 8);
    f[13] = static_cast<uint8_t>(p);
    for (uint16_t j = 0; j < kCaptureLenBytes * 8; j++) {
        const uint16_t i = j + LR2021OokAdsb::kDF17HeaderLenBits;
        PutBit(cap, j, i < 112 ? GetBit(f, i) : (Rand() & 1u));
    }
}

// A false trigger or a garbled frame: nothing decodes, and the decoder tries its single-bit correction.
void NoiseCapture(uint8_t* cap) {
    for (uint16_t k = 0; k < kCaptureLenBytes; k++) cap[k] = static_cast<uint8_t>(Rand());
}

void DrainDecoderQueues() {
    RawModeSPacket raw;
    while (packet_decoder.raw_mode_s_packet_queue.Dequeue(raw)) {
    }
    DecodedModeSPacket decoded;
    while (packet_decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded)) {
    }
}

}  // namespace

UTEST(RxCpu, CycleCounterRuns) {
    CycleCounter::Enable();
    const uint32_t a = CycleCounter::Now();
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < 1000; i++) sink = sink + i;
    const uint32_t cycles = CycleCounter::Since(a);
    ASSERT_GT(cycles, 1000u);
    CONSOLE_PRINTF("RXCPU empty loop of 1000: %lu cycles\r\n", (unsigned long)cycles);
}

UTEST(RxCpu, ParseAndDecodeCycles) {
    const SettingsManager::R1090PreambleMode old_mode =
        ADSBeeTestAccessor::SwapMode(SettingsManager::kR1090PreambleModeDF17);
    const uint32_t old_frames_since_valid = ADSBeeTestAccessor::FramesSinceValid();
    DrainDecoderQueues();
    for (int valid = 1; valid >= 0; valid--) {
        CycleStats parse, decode;
        uint32_t decoded_valid = 0;
        for (uint16_t n = 0; n < 200; n++) {
            uint8_t cap[kCaptureLenBytes];
            if (valid) {
                ValidCapture(cap);
            } else {
                NoiseCapture(cap);
            }
            const uintptr_t key = HwiP_disable();
            const uint32_t t0 = CycleCounter::Now();
            ADSBeeTestAccessor::Parse(cap, sizeof(cap));
            const uint32_t t1 = CycleCounter::Now();
            packet_decoder.Update();
            const uint32_t t2 = CycleCounter::Now();
            HwiP_restore(key);
            parse.Add((t1 - t0) & CycleCounter::kMask);
            decode.Add((t2 - t1) & CycleCounter::kMask);
            DecodedModeSPacket decoded;
            while (packet_decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded)) {
                decoded_valid += decoded.is_valid ? 1 : 0;
            }
            DrainDecoderQueues();  // Nothing synthetic reaches the aircraft dictionary or a report.
        }
        CONSOLE_PRINTF("RXCPU --- %s\r\n", valid ? "valid DF17 capture" : "noise capture (no match)");
        parse.Print("ParseLR2021RxFifo, one capture");
        decode.Print("PacketDecoder::Update, one packet");
        if (valid) {
            EXPECT_EQ(decoded_valid, 200u);  // The reconstruction puts the four consumed DF bits back.
        }
    }
    ADSBeeTestAccessor::FramesSinceValid() = old_frames_since_valid;
    ADSBeeTestAccessor::SwapMode(old_mode);
}

// A receiver config the chip rejects (CMD_PERR) is replaced by the factory config once, instead of being retried
// on every health-ladder backoff (a full reset and config each time, 8.7 ms of main loop, no reception).
UTEST(ReceiverConfig, RejectedConfigFallsBackOnce) {
    const SettingsManager::R1090PreambleMode old_mode = adsbee.GetR1090PreambleMode();
    const uint32_t old_fallbacks = adsbee.lr2021_config_fallback_count;
    const uint32_t old_fails = adsbee.lr2021_config_fail_count;

    ADSBeeTestAccessor::SwapMode(SettingsManager::kR1090PreambleModeModeS);
    adsbee.lr2021.test_detector_len_override = 11;  // Odd: the LR2021 answers CMD_PERR.
    const bool ok = ADSBeeTestAccessor::ApplyReceiverConfig();

    EXPECT_TRUE(ok);
    EXPECT_TRUE(ADSBeeTestAccessor::ReceiverConfigOk());
    EXPECT_EQ(adsbee.lr2021_config_fallback_count, old_fallbacks + 1);
    EXPECT_EQ(adsbee.lr2021_config_fail_count, old_fails);
    EXPECT_EQ(static_cast<int>(adsbee.GetR1090PreambleMode()),
              static_cast<int>(SettingsManager::kR1090PreambleModeDF17));
    EXPECT_EQ(adsbee.lr2021.last_stat().chip_mode, LR2021::ChipMode::kRx);

    adsbee.SetR1090PreambleMode(old_mode);  // Back to what the device ran before the test.
}

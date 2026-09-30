// On-target CPU cost of the DF17-mode receive path (AT+TEST), measured in CPU cycles (utils/cycle_counter.hh).
// Synthetic captures, no RF: the realignment (the first version from df17_recover_reference.hh and the
// current one), one capture through ADSBee::ParseLR2021RxFifo, and one packet through PacketDecoder.
// Every measurement runs with interrupts off, so min/avg/max are the function's own cost.
#include <ti/drivers/dpl/HwiP.h>

#include "adsbee.hh"
#include "crc.hh"
#include "cycle_counter.hh"
#include "df17_recover_reference.hh"
#include "hardware_unit_tests.hh"
#include "lr2021_ook_adsb.hh"
#include "packet_decoder.hh"

using namespace LR2021OokAdsb;

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
};

namespace {

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
        CONSOLE_PRINTF("DF17CPU %-34s n=%4lu min=%6lu avg=%6lu max=%6lu cycles (max %lu us)\r\n", name,
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

// A valid DF17 frame with random ICAO and ME and the given CA.
void RandomDF17Frame(uint8_t ca, uint8_t* f) {
    f[0] = static_cast<uint8_t>((17u << 3) | (ca & 7u));
    for (uint8_t k = 1; k < kModeSFrameLenBytes - 3; k++) f[k] = static_cast<uint8_t>(Rand());
    const uint32_t p = crc24(f, kModeSFrameLenBytes - 3);
    f[11] = static_cast<uint8_t>(p >> 16);
    f[12] = static_cast<uint8_t>(p >> 8);
    f[13] = static_cast<uint8_t>(p);
}

// What the detector captures when it starts at message bit `shift` (bits outside the frame random).
void CaptureAt(const uint8_t* frame, int shift, uint8_t* cap) {
    for (int j = 0; j < kModeSFrameLenBits; j++) {
        const int i = j + shift;
        SetMsgBit(cap, static_cast<uint16_t>(j),
                  (i < 0 || i >= kModeSFrameLenBits) ? (Rand() & 1u) : GetMsgBit(frame, static_cast<uint16_t>(i)));
    }
}

uint32_t Crc24Fn(const uint8_t* buf, uint16_t len) { return crc24(buf, len); }

enum CaptureKind { kNominal, kLateWithFlips, kEarlyWithFlips, kNoise };
void MakeCapture(CaptureKind kind, uint8_t* cap) {
    uint8_t f[kModeSFrameLenBytes];
    RandomDF17Frame(static_cast<uint8_t>(Rand()), f);
    switch (kind) {
        case kNominal:  // A clean capture: the fast path.
            CaptureAt(f, kDF17HeaderLenBits, cap);
            break;
        case kLateWithFlips:  // Shift 7 (the last one tried), CA bit 7 wrong: the last candidates.
            SetMsgBit(f, 7, !GetMsgBit(f, 7));
            CaptureAt(f, 7, cap);
            break;
        case kEarlyWithFlips:  // Shift -2 (the most candidates), all three CA bits wrong.
            for (uint16_t b = 5; b < 8; b++) SetMsgBit(f, b, !GetMsgBit(f, b));
            CaptureAt(f, -2, cap);
            break;
        case kNoise:  // A false trigger or a garbled frame: nothing matches, every candidate is tried.
            for (uint8_t k = 0; k < kModeSFrameLenBytes; k++) cap[k] = static_cast<uint8_t>(Rand());
            break;
    }
}

const char* KindName(CaptureKind kind) {
    switch (kind) {
        case kNominal:
            return "nominal (valid, fast path)";
        case kLateWithFlips:
            return "shift 7 + CA flip (valid)";
        case kEarlyWithFlips:
            return "shift -2 + 3 CA flips (valid)";
        default:
            return "noise (no match, worst case)";
    }
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

UTEST(DF17Cpu, CycleCounterRuns) {
    CycleCounter::Enable();
    const uint32_t a = CycleCounter::Now();
    volatile uint32_t sink = 0;
    for (uint32_t i = 0; i < 1000; i++) sink = sink + i;
    const uint32_t cycles = CycleCounter::Since(a);
    ASSERT_GT(cycles, 1000u);
    CONSOLE_PRINTF("DF17CPU empty loop of 1000: %lu cycles\r\n", (unsigned long)cycles);
}

UTEST(DF17Cpu, RealignmentCycles) {
    static constexpr uint16_t kCapturesPerKind = 300;
    uint32_t overhead = UINT32_MAX;
    for (int i = 0; i < 20; i++) {
        const uint32_t a = CycleCounter::Now();
        const uint32_t b = CycleCounter::Now();
        overhead = ((b - a) & CycleCounter::kMask) < overhead ? ((b - a) & CycleCounter::kMask) : overhead;
    }
    CONSOLE_PRINTF("DF17CPU timer overhead %lu cycles (subtracted)\r\n", (unsigned long)overhead);
    for (CaptureKind kind : {kNominal, kLateWithFlips, kEarlyWithFlips, kNoise}) {
        CycleStats before, after;
        uint32_t mismatches = 0;
        for (uint16_t n = 0; n < kCapturesPerKind; n++) {
            uint8_t cap[kModeSFrameLenBytes], out_before[kModeSFrameLenBytes], out_after[kModeSFrameLenBytes];
            MakeCapture(kind, cap);
            const uintptr_t key = HwiP_disable();
            uint32_t t0 = CycleCounter::Now();
            const int8_t s_before = LR2021OokAdsbReference::RecoverDF17Frame(cap, out_before, Crc24Fn);
            uint32_t t1 = CycleCounter::Now();
            const int8_t s_after = RecoverDF17Frame(cap, out_after, Crc24Fn);
            uint32_t t2 = CycleCounter::Now();
            HwiP_restore(key);
            before.Add(((t1 - t0) & CycleCounter::kMask) - overhead);
            after.Add(((t2 - t1) & CycleCounter::kMask) - overhead);
            bool same = s_before == s_after;
            for (uint8_t k = 0; k < kModeSFrameLenBytes; k++) same &= out_before[k] == out_after[k];
            mismatches += same ? 0 : 1;
        }
        CONSOLE_PRINTF("DF17CPU --- %s\r\n", KindName(kind));
        before.Print("realign, first version (2e47e61f)");
        after.Print("realign, syndrome version");
        EXPECT_EQ(mismatches, 0u);
    }
}

UTEST(DF17Cpu, ParseAndDecodeCycles) {
    const SettingsManager::R1090PreambleMode old_mode =
        ADSBeeTestAccessor::SwapMode(SettingsManager::kR1090PreambleModeDF17);
    const uint32_t old_frames_since_valid = ADSBeeTestAccessor::FramesSinceValid();
    DrainDecoderQueues();
    for (CaptureKind kind : {kNominal, kNoise}) {
        CycleStats parse, decode;
        for (uint16_t n = 0; n < 200; n++) {
            uint8_t cap[kModeSFrameLenBytes];
            MakeCapture(kind, cap);
            const uintptr_t key = HwiP_disable();
            uint32_t t0 = CycleCounter::Now();
            ADSBeeTestAccessor::Parse(cap, sizeof(cap));
            uint32_t t1 = CycleCounter::Now();
            packet_decoder.Update();
            uint32_t t2 = CycleCounter::Now();
            HwiP_restore(key);
            parse.Add((t1 - t0) & CycleCounter::kMask);
            decode.Add((t2 - t1) & CycleCounter::kMask);
            DrainDecoderQueues();  // Nothing synthetic reaches the aircraft dictionary or a report.
        }
        CONSOLE_PRINTF("DF17CPU --- %s\r\n", KindName(kind));
        parse.Print("ParseLR2021RxFifo, one capture");
        decode.Print("PacketDecoder::Update, one packet");
    }
    ADSBeeTestAccessor::FramesSinceValid() = old_frames_since_valid;
    ADSBeeTestAccessor::SwapMode(old_mode);
}

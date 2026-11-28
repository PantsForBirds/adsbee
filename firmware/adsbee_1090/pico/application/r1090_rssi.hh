#pragma once

#include <stdint.h>

#include "macros.hh"  // DivRound

/**
 * 1090MHz receiver: conversions from the RF power detector voltage (AD8313 behind two LNAs, read on the RSSI ADC input
 * through a 10k / 1nF low pass) to dBm at the antenna connector.
 *
 * The low pass (10us) averages the detector's log output over the pulses of a Mode S packet, so a packet sample sits
 * only a fraction of the pulse height above the noise floor. Static levels (noise floor, trigger level) are not
 * affected. The packet sample is taken about 43us into the message, when the low pass has settled: it reads the
 * same for 56 and 112 bit packets.
 *
 * One calibration serves every RF front end version: the detector, its low pass and the ADC path are the same on all
 * of them (the versions differ in the comparator network). It was measured with CW and Mode S packets on a 1090U
 * rev G (2026-10). Gain differences between boards and SAW filter types shift the reading by the same amount.
 */
namespace r1090_rssi {

static constexpr int kSlopeMVPerdBx10 = 176;      // [0.1 mV/dB] Below the first curve point, down to the floor.
static constexpr int kPacketAveragePercent = 53;  // Settled packet sample height / pulse height above the floor.
static constexpr int kMaxdBm = -42;               // [dBm] The detector is at full scale from here up.

// CW calibration points. The detector compresses above -50 dBm and saturates at about -44 dBm.
struct CurvePoint {
    int16_t mv;
    int16_t dbm;
};
static constexpr CurvePoint kCurve[] = {{1137, -80}, {1489, -60}, {1680, -50},    {1715, -48},
                                        {1740, -46}, {1759, -44}, {1770, kMaxdBm}};
static constexpr int kNumCurvePoints = sizeof(kCurve) / sizeof(kCurve[0]);

/**
 * Original conversion (AD8313 datasheet slope, 44dB LNA gain). Only kept for the AT+TL_OFFSET text, which the
 * settings UI parses.
 */
static constexpr int LegacyMilliVoltsTodBm(int mv) { return 60 * (mv - 1600) / 1000 - 44; }

/**
 * Converts a static detector voltage (noise floor, trigger level, CW) to dBm.
 * @param[in] mv Detector voltage, in mV.
 * @retval Power at the antenna connector, in dBm.
 */
static constexpr int DetectorMilliVoltsTodBm(int mv) {
    if (mv >= kCurve[kNumCurvePoints - 1].mv) {
        return kMaxdBm;
    }
    for (int i = kNumCurvePoints - 1; i > 0; i--) {
        const CurvePoint& lo = kCurve[i - 1];
        const CurvePoint& hi = kCurve[i];
        if (mv >= lo.mv) {
            return lo.dbm + DivRound((hi.dbm - lo.dbm) * (mv - lo.mv), hi.mv - lo.mv);
        }
    }
    return kCurve[0].dbm + DivRound((mv - kCurve[0].mv) * 10, kSlopeMVPerdBx10);
}

/**
 * Converts the RSSI ADC sample of a Mode S packet to dBm.
 * @param[in] sample_mv Sample taken mid-message, in mV.
 * @param[in] noise_floor_mv Detector voltage between packets, in mV.
 * @retval Pulse power at the antenna connector, in dBm.
 */
static constexpr int PacketMilliVoltsTodBm(int sample_mv, int noise_floor_mv) {
    int height_mv = sample_mv > noise_floor_mv ? sample_mv - noise_floor_mv : 0;
    return DetectorMilliVoltsTodBm(noise_floor_mv + height_mv * 100 / kPacketAveragePercent);
}

}  // namespace r1090_rssi

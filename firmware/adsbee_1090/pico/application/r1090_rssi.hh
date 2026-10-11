#pragma once

#include <stdint.h>

/**
 * 1090MHz receiver: conversions from the RF power detector voltage (AD8313 behind two LNAs, read on the RSSI ADC input
 * through a 10k / 1nF low pass) to dBm at the antenna connector, per RF front end version.
 *
 * The low pass (10us) averages the detector's log output over the pulses of a Mode S packet, so a packet sample sits
 * only a fraction of the pulse height above the noise floor. Static levels (noise floor, trigger level) are not
 * affected. The packet sample is taken about 43us into the message, when the low pass has settled: it reads the
 * same for 56 and 112 bit packets.
 */
namespace r1090_rssi {

// Matches SettingsManager::DeviceInfo::ADSBee1090RFFrontendVersion.
static constexpr uint8_t kFrontendV3 = 3;

// Front end v3, calibrated with CW and Mode S packets on a 1090U rev G (2026-10).
static constexpr int kV3SlopeMVPerdBx10 = 176;      // [0.1 mV/dB] Below the first curve point, down to the floor.
static constexpr int kV3PacketAveragePercent = 53;  // Settled packet sample height / pulse height above the floor.
static constexpr int kV3MaxdBm = -42;               // [dBm] The detector is at full scale from here up.

// CW calibration points. The detector compresses above -50 dBm and saturates at about -44 dBm.
struct CurvePoint {
    int16_t mv;
    int16_t dbm;
};
static constexpr CurvePoint kV3Curve[] = {{1137, -80}, {1489, -60}, {1680, -50},      {1715, -48},
                                          {1740, -46}, {1759, -44}, {1770, kV3MaxdBm}};
static constexpr int kV3NumCurvePoints = sizeof(kV3Curve) / sizeof(kV3Curve[0]);

// Integer division rounded to nearest (denominator > 0).
static constexpr int DivRound(int numerator, int denominator) {
    return (numerator + (numerator >= 0 ? denominator : -denominator) / 2) / denominator;
}

/**
 * Original conversion, used for front ends v1 and v2 (AD8313 datasheet slope, 44dB LNA gain from bench testing).
 */
static constexpr int LegacyMilliVoltsTodBm(int mv) { return 60 * (mv - 1600) / 1000 - 44; }

// The original conversion was tuned on a packet sample taken as the demodulation began, while the low pass was still
// rising: about 58 % of the settled height (measured on a 1090U rev G; the low pass is the same on every revision).
// Scaling the settled sample back keeps the v1 and v2 readings where they were.
static constexpr int kLegacyPacketSamplePercent = 58;

/**
 * Converts a static detector voltage (noise floor, trigger level, CW) to dBm.
 * @param[in] mv Detector voltage, in mV.
 * @param[in] frontend_version RF front end version.
 * @retval Power at the antenna connector, in dBm.
 */
static constexpr int DetectorMilliVoltsTodBm(int mv, uint8_t frontend_version) {
    if (frontend_version < kFrontendV3) {
        return LegacyMilliVoltsTodBm(mv);
    }
    if (mv >= kV3Curve[kV3NumCurvePoints - 1].mv) {
        return kV3MaxdBm;
    }
    for (int i = kV3NumCurvePoints - 1; i > 0; i--) {
        const CurvePoint& lo = kV3Curve[i - 1];
        const CurvePoint& hi = kV3Curve[i];
        if (mv >= lo.mv) {
            return lo.dbm + DivRound((hi.dbm - lo.dbm) * (mv - lo.mv), hi.mv - lo.mv);
        }
    }
    return kV3Curve[0].dbm + DivRound((mv - kV3Curve[0].mv) * 10, kV3SlopeMVPerdBx10);
}

/**
 * Converts the RSSI ADC sample of a Mode S packet to dBm.
 * @param[in] sample_mv Sample taken mid-message, in mV.
 * @param[in] noise_floor_mv Detector voltage between packets, in mV.
 * @param[in] frontend_version RF front end version.
 * @retval Pulse power at the antenna connector, in dBm.
 */
static constexpr int PacketMilliVoltsTodBm(int sample_mv, int noise_floor_mv, uint8_t frontend_version) {
    int height_mv = sample_mv > noise_floor_mv ? sample_mv - noise_floor_mv : 0;
    if (frontend_version < kFrontendV3) {
        return LegacyMilliVoltsTodBm(noise_floor_mv + height_mv * kLegacyPacketSamplePercent / 100);
    }
    return DetectorMilliVoltsTodBm(noise_floor_mv + height_mv * 100 / kV3PacketAveragePercent, frontend_version);
}

}  // namespace r1090_rssi

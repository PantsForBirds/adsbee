#pragma once

#include <cstdint>

// The CC1312/CC1314 modem stamps some UAT syncs half a bit (0.47 us) late. Late stamps only happen when the sync
// lands in a ~1.3 us slice of a 10/3 us cycle (80 clocks of the 24 MHz modem clock) that starts with the receive
// command. Receive commands therefore start at an exact radio timer (RAT, 4 MHz) time, which puts every sync stamp
// at a known phase of that cycle.
namespace UATRxStart {

// Phase classes of 1/12 us: the 10/3 us cycle is 40/3 RAT ticks.
static constexpr uint8_t kNumPhaseClasses = 40;

struct ChipTiming {
    // Start lead after the RAT read that sets the start time. A command that the RF core gets too late ends with
    // ERROR_PAST_START; it is then restarted immediately, and its stamps are imprecise (a restart lands at another
    // phase).
    uint32_t lead_rat;
    // The RF core starts receiving late, without an error, when the start time is less than this after
    // RF_postCmd() returns.
    uint32_t min_margin_rat;
    // Bit n set when phase class n gives a precise stamp. Uplinks (the other sync word) have their own slice.
    uint64_t precise_adsb_classes;
    uint64_t precise_uplink_classes;
};

// From bench calibration with each chip's own RF core firmware. RF_postCmd() returns 34-42 us after the RAT read on
// the CC1314 and 36-45 us on the CC1312.
// CC1312: ADS-B classes 22-39 and 0-2, uplink classes 30-39 and 0-8.
static constexpr ChipTiming kCC1312 = {50 * 4, 32, 0xFFFFC00007ULL, 0xFFC00001FFULL};
// CC1314: ADS-B classes 19-37, uplink classes 22-39 and 0-3.
static constexpr ChipTiming kCC1314 = {48 * 4, 40, 0x3FFFF80000ULL, 0xFFFFC0000FULL};

/**
 * Phase of a sync stamp within the modem cycle.
 * @param[in] sync_rat Sync stamp in RAT ticks.
 * @param[in] start_rat Start time of the receive command that got the packet, in RAT ticks.
 * @retval Phase class, 0 to kNumPhaseClasses - 1.
 */
inline uint8_t PhaseClass(uint32_t sync_rat, uint32_t start_rat) {
    return static_cast<uint8_t>((sync_rat - start_rat) % kNumPhaseClasses * 3 % kNumPhaseClasses);
}

/**
 * @param[in] precise_classes Bit n set when phase class n gives a precise stamp.
 * @param[in] phase_class Phase class from PhaseClass().
 * @retval True if a stamp in this phase class is precise.
 */
inline bool IsPrecise(uint64_t precise_classes, uint8_t phase_class) {
    return phase_class < kNumPhaseClasses && ((precise_classes >> phase_class) & 1);
}

/**
 * @param[in] start_rat Start time of the posted receive command.
 * @param[in] posted_rat RAT read right after RF_postCmd() returned.
 * @param[in] min_margin_rat ChipTiming::min_margin_rat.
 * @retval True if the command starts exactly at start_rat.
 */
inline bool StartIsExact(uint32_t start_rat, uint32_t posted_rat, uint32_t min_margin_rat) {
    return static_cast<int32_t>(start_rat - posted_rat) >= static_cast<int32_t>(min_margin_rat);
}

}  // namespace UATRxStart

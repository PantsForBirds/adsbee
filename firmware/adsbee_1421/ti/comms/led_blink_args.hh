#pragma once

#include <cstdint>
#include <string_view>

// AT+LED_BLINK=<duration_ms>,<led>[,<led>...] arguments, parsed the same way as on the ADSBee 1090.
namespace LEDBlinkArgs {

static constexpr uint32_t kMaxDurationMs = 60'000;  // Same limit as the 1090 (ObjectDictionary::kLEDBlinkMaxDurationMs).

struct Selection {
    bool led_1090 = false;
    bool led_subg = false;
};

enum Result : uint8_t {
    kOK = 0,
    kNoNetworkLED,  // NETWORK is valid on the 1090, which has an ESP32 LED; the 1421 has none.
    kInvalidLED
};

inline bool DurationIsValid(uint32_t duration_ms) { return duration_ms >= 1 && duration_ms <= kMaxDurationMs; }

/**
 * Parses LED names (1090, SUBG, ALL; case sensitive like the 1090). All names are checked before anything blinks.
 * @param[in] names LED name arguments.
 * @param[in] num_names Number of names.
 * @param[out] selection LEDs to blink. Only valid when the result is kOK.
 * @param[out] bad_index Index of the rejected name when the result is not kOK.
 */
inline Result ParseLEDs(const std::string_view* names, uint16_t num_names, Selection& selection, uint16_t& bad_index) {
    selection = Selection();
    for (uint16_t i = 0; i < num_names; i++) {
        std::string_view name = names[i];
        if (name == "1090") {
            selection.led_1090 = true;
        } else if (name == "SUBG") {
            selection.led_subg = true;
        } else if (name == "ALL") {
            selection.led_1090 = selection.led_subg = true;
        } else {
            bad_index = i;
            return name == "NETWORK" ? kNoNetworkLED : kInvalidLED;
        }
    }
    return kOK;
}

}  // namespace LEDBlinkArgs

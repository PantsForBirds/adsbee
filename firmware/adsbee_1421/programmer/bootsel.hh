#pragma once

#include <stdint.h>

// Reads the BOOTSEL button. Briefly stalls flash access with interrupts off; keep calls to a
// modest cadence (tens of ms) so UART RX interrupts are not starved.
bool GetBootselButton();

// Poll cadence for PollBootsel(). Tens of ms: fast enough to feel responsive, slow enough that
// GetBootselButton()'s flash stall does not starve UART RX interrupts.
static const uint32_t kBootselPollMs = 50;

// How long BOOTSEL must be held to count as a long press rather than a tap.
static const uint32_t kBootselLongPressMs = 3000;

// Gesture reported by PollBootsel().
enum class BootselEvent {
    kNone,
    kShortPress,  // Pressed and released inside kBootselLongPressMs. Reported on release.
    kLongPress,   // Held for kBootselLongPressMs. Reported once, as soon as the threshold is
                  // crossed and while the button is still down, so the user gets immediate
                  // feedback and the subsequent release is swallowed rather than read as a tap.
};

// Debounced BOOTSEL gesture detector, shared by the main state machine's wait loops and the
// pass-through bridge so a given gesture means the same thing everywhere. Keeps its own state
// across calls; poll it on a single cadence of tens of ms (see GetBootselButton()) and never from
// two loops at once. Returns kNone until a gesture completes.
BootselEvent PollBootsel();

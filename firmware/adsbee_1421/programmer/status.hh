#pragma once

// WS2812 status LED plus human-readable status text on the USB CDC port.
//
// LED legend:
//   yellow blink  waiting for a device (bootloader entry retry loop)
//   orange        forced-reflash armed (BOOTSEL held at power-up)
//   white blink   settings-erase armed (BOOTSEL held 3 s); runs at the next bootloader entry
//   cyan          CRC check against the baked image
//   magenta blink erase + program
//   blue blink    post-program CRC verify
//   blue          console baud negotiation
//   green         pass-through at 1 Mbaud
//   red           error (before automatic retry)

enum class Status {
    kWaitingForDevice,
    kForcedFlashArmed,
    kSettingsEraseArmed,
    kCrcCheck,
    kFlashing,
    kVerifying,
    kNegotiating,
    kPassthrough,
    kError,
};

void StatusInit();
void StatusSet(Status status);
void StatusUpdate();  // Advances blink phase; call from every wait/poll loop.

// printf to the USB CDC port; silently dropped when no host terminal is connected. Used only
// outside pass-through, so status text never mixes with bridged traffic.
void CdcPrintf(const char* format, ...) __attribute__((format(printf, 1, 2)));

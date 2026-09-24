#include "status.hh"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "ws2812.pio.h"  // Generated from ws2812.pio by pico_generate_pio_header().

// The RP2040-Zero's only LED is an addressable WS2812 on GP16 (same driver as the
// adsbee-lp1090 sync_pulser jig).
static const uint kLedPin = PICO_DEFAULT_WS2812_PIN;
static const uint8_t kLedBrightness = 16;  // Out of 255. These are painfully bright at full scale.
static const uint32_t kBlinkPeriodMs = 250;

static PIO led_pio = pio0;
static uint led_sm = 0;

static Status current_status = Status::kWaitingForDevice;
static bool last_led_on = true;

static void LedSetRgb(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 wants GRB, MSB first, left-aligned in the 32-bit FIFO word.
    uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    pio_sm_put_blocking(led_pio, led_sm, grb << 8u);
}

static void ApplyLed(bool blink_on) {
    const uint8_t kB = kLedBrightness;
    switch (current_status) {
        case Status::kWaitingForDevice:  // Yellow blink.
            if (blink_on) LedSetRgb(kB, kB, 0); else LedSetRgb(0, 0, 0);
            break;
        case Status::kForcedFlashArmed:  // Orange.
            LedSetRgb(kB, kB / 2, 0);
            break;
        case Status::kSettingsEraseArmed:  // White blink -- destructive action pending.
            if (blink_on) LedSetRgb(kB, kB, kB); else LedSetRgb(0, 0, 0);
            break;
        case Status::kCrcCheck:  // Cyan.
            LedSetRgb(0, kB, kB);
            break;
        case Status::kFlashing:  // Magenta blink.
            if (blink_on) LedSetRgb(kB, 0, kB); else LedSetRgb(0, 0, 0);
            break;
        case Status::kVerifying:  // Blue blink.
            if (blink_on) LedSetRgb(0, 0, kB); else LedSetRgb(0, 0, 0);
            break;
        case Status::kNegotiating:  // Blue.
            LedSetRgb(0, 0, kB);
            break;
        case Status::kPassthrough:  // Green.
            LedSetRgb(0, kB, 0);
            break;
        case Status::kError:  // Red.
            LedSetRgb(kB, 0, 0);
            break;
    }
}

void StatusInit() {
    uint offset = pio_add_program(led_pio, &ws2812_program);
    ws2812_program_init(led_pio, led_sm, offset, kLedPin, 800000, false);
    ApplyLed(true);
}

void StatusSet(Status status) {
    current_status = status;
    last_led_on = true;
    ApplyLed(true);
}

void StatusUpdate() {
    bool blink_on = (to_ms_since_boot(get_absolute_time()) / kBlinkPeriodMs) % 2 == 0;
    if (blink_on != last_led_on) {
        last_led_on = blink_on;
        ApplyLed(blink_on);
    }
}

void CdcPrintf(const char* format, ...) {
    if (!tud_cdc_connected()) return;
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len <= 0) return;
    if ((size_t)len >= sizeof(buf)) len = sizeof(buf) - 1;  // vsnprintf truncated.

    const char* pos = buf;
    while (len > 0) {
        uint32_t wrote = tud_cdc_write(pos, (uint32_t)len);
        pos += wrote;
        len -= (int)wrote;
        tud_cdc_write_flush();
        tud_task();
        if (!tud_cdc_connected()) return;  // Host went away mid-message.
    }
}

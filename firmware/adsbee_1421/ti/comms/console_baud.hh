#pragma once

#include <stdint.h>

// Console baud rates the ADSBee 1421 accepts, shared by the CC1314 firmware, the ADSBee 1421 Programmer and its host
// tests. Both chips use a PL011 UART: divisor = UARTCLK / (16 * baud), with a 6-bit fraction.
//   - kMax: the divisor's integer part must be at least 1 (48 MHz clock -> 3 Mbaud).
//   - kMin: below 9600 the console can't keep up with its report output.
namespace ConsoleBaud {

static constexpr uint32_t kMin = 9600;
static constexpr uint32_t kMax = 3000000;
static constexpr uint32_t kMaxErrorPpm = 20000;  // 2%.
static constexpr uint32_t kCC1314UartClockHz = 48000000;

// Divisor in 1/64 units, rounded as both the TI driverlib and the pico-sdk do.
constexpr uint32_t Pl011Divisor64(uint32_t uart_clock_hz, uint32_t baud) {
    return static_cast<uint32_t>(((uint64_t)uart_clock_hz * 8u / baud + 1u) / 2u);
}

// Rate a PL011 actually generates for `baud`, or 0 if the divisor is out of range.
constexpr uint32_t Pl011ActualBaud(uint32_t uart_clock_hz, uint32_t baud) {
    if (baud == 0) return 0;
    uint32_t divisor = Pl011Divisor64(uart_clock_hz, baud);
    if (divisor < 64u || divisor / 64u > 0xFFFFu) return 0;
    return static_cast<uint32_t>((uint64_t)uart_clock_hz * 4u / divisor);
}

// |actual - nominal| / nominal in parts per million (UINT32_MAX if actual is 0).
constexpr uint32_t ErrorPpm(uint32_t nominal, uint32_t actual) {
    if (nominal == 0 || actual == 0) return UINT32_MAX;
    uint64_t diff = actual > nominal ? actual - nominal : nominal - actual;
    return static_cast<uint32_t>(diff * 1000000u / nominal);
}

constexpr uint32_t CC1314ActualBaud(uint32_t baud) { return Pl011ActualBaud(kCC1314UartClockHz, baud); }

// True if the ADSBee 1421 console accepts `baud`.
constexpr bool IsSupported(uint32_t baud) {
    return baud >= kMin && baud <= kMax && ErrorPpm(baud, CC1314ActualBaud(baud)) <= kMaxErrorPpm;
}

}  // namespace ConsoleBaud

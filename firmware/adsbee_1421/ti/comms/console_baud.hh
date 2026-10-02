#pragma once

#include <stdint.h>

// Console baud rates the ADSBee 1421 accepts (AT+BAUD_RATE=CONSOLE,<baud>, the saved setting applied at boot), and
// the PL011 divider arithmetic behind them. Pure constexpr with no SDK dependencies: the CC1314 firmware, the ADSBee
// 1421 Programmer (which renegotiates the console to its host's rate) and the Programmer's host tests share it.
//
// The CC1314 console (UART2CC26X2) and the Programmer's RP2040 UART are both ARM PL011s: 16x oversampling and a baud
// divisor of UARTCLK / (16 * baud) held as a 16-bit integer part and a 6-bit fraction. Any rate is accepted that the
// CC1314 can generate within kMaxErrorPpm and that lies in [kMin, kMax]:
//   - kMax: the divisor's integer part must be at least 1, so the CC1314's 48 MHz UART clock tops out at 3 Mbaud.
//   - kMin: the hardware goes down to ~46 baud, but the console's report output can't keep up below a few kB/s and
//     its TX waits grow with the byte time. 9600 is the lowest common rate; it also keeps the 1200 baud that pymavlink
//     opens ports at (and the Programmer passes straight through) out of the console range.
// Within that range the 6-bit fraction keeps the CC1314's rounding error below 0.8%, so kMaxErrorPpm only guards
// the arithmetic. The previous whitelist {115200, 230400, 460800, 921600, 1000000} lies inside the range, so saved
// settings stay valid.
namespace ConsoleBaud {

static constexpr uint32_t kMin = 9600;
static constexpr uint32_t kMax = 3000000;
static constexpr uint32_t kMaxErrorPpm = 20000;  // 2%.
static constexpr uint32_t kCC1314UartClockHz = 48000000;  // UART2CC26X2 clocks the PL011 from the 48 MHz CPU clock.

// Fractional divisor in 1/64 units, rounded to nearest: driverlib UARTConfigSetExpClk() (CC13x4) and pico-sdk
// uart_set_baudrate() (RP2040) both compute it this way. 64-bit so any clock and baud are safe.
constexpr uint32_t Pl011Divisor64(uint32_t uart_clock_hz, uint32_t baud) {
    return static_cast<uint32_t>(((uint64_t)uart_clock_hz * 8u / baud + 1u) / 2u);
}

// Rate a PL011 actually generates for `baud`, or 0 if the divisor is out of range (integer part 0 or > 65535). The
// CC1314 driverlib does not clamp, so an out-of-range divisor means the rate can't be generated at all.
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

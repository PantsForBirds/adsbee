// Tests for the console baud range (ti/comms/console_baud.hh).
#include <stdint.h>
#include <stdio.h>

#include <initializer_list>

#include "console_baud.hh"
#include "gtest/gtest.h"
#include "host_line_coding.hh"

// ---- Baud arithmetic ----

// Independent copies of the driverlib (CC1314) and pico-sdk (RP2040) baud divider code.
static uint32_t TiActual(uint32_t clk, uint32_t baud) {
    uint32_t div = (((clk * 8) / baud) + 1) / 2;
    uint32_t ibrd = div / 64, fbrd = div % 64;
    if (ibrd == 0 || ibrd > 0xFFFF) return 0;
    return (uint32_t)((uint64_t)clk * 4 / (64 * ibrd + fbrd));
}
static uint32_t PicoActual(uint32_t clk, uint32_t baud) {
    uint32_t div = (8 * clk / baud) + 1;
    uint32_t ibrd = div >> 7, fbrd;
    if (ibrd == 0) {
        ibrd = 1;
        fbrd = 0;
    } else if (ibrd >= 65535) {
        ibrd = 65535;
        fbrd = 0;
    } else {
        fbrd = (div & 0x7f) >> 1;
    }
    return (4 * clk) / (64 * ibrd + fbrd);
}

// PL011 divider matches driverlib and pico-sdk.
TEST(ConsoleBaud, DividerMatchesDrivers) {
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 997) {
        EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(baud), TiActual(ConsoleBaud::kCC1314UartClockHz, baud));
        EXPECT_EQ(ProgrammerActualBaud(baud), PicoActual(kProgrammerUartClockHz, baud));
    }
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(115200), 115176u);  // Divisor 1667/64.
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(3000000), 3000000u);
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(3000001), 3000000u);  // Rounds to the same divisor.
    EXPECT_EQ(ConsoleBaud::CC1314ActualBaud(3100000), 0u);        // Integer part 0: can't be generated.
    EXPECT_EQ(ConsoleBaud::Pl011ActualBaud(48000000, 45u), 0u);   // Integer part > 65535.
    EXPECT_EQ(ConsoleBaud::Pl011ActualBaud(48000000, 0u), 0u);
}

// Console accepts 9600 to 3000000 baud within 2%, and the Programmer's UART matches each.
TEST(ConsoleBaud, ConsoleRange) {
    // The old whitelist stays valid, so saved settings keep working.
    for (uint32_t baud : {115200u, 230400u, 460800u, 921600u, 1000000u}) {
        EXPECT_TRUE(ConsoleBaud::IsSupported(baud));
    }
    for (uint32_t baud : {9600u, 19200u, 38400u, 57600u, 76800u, 250000u, 500000u, 2000000u, 3000000u, 123457u}) {
        EXPECT_TRUE(ConsoleBaud::IsSupported(baud));
    }
    for (uint32_t baud : {0u, 300u, 1200u, 9599u, 3000001u, 4000000u, 0xDEADBEEu, 0xFFFFFFFFu}) {
        EXPECT_FALSE(ConsoleBaud::IsSupported(baud));
    }
    // Every rate in range is well inside 2% on both ends.
    uint32_t worst_cc1314 = 0, worst_link = 0;
    for (uint32_t baud = ConsoleBaud::kMin; baud <= ConsoleBaud::kMax; baud += 101) {
        uint32_t cc1314 = ConsoleBaud::CC1314ActualBaud(baud);
        uint32_t e = ConsoleBaud::ErrorPpm(baud, cc1314);
        if (e > worst_cc1314) worst_cc1314 = e;
        e = ConsoleBaud::ErrorPpm(cc1314, ProgrammerActualBaud(baud));
        if (e > worst_link) worst_link = e;
        EXPECT_TRUE(ConsoleBaud::IsSupported(baud));
    }
    printf("  worst CC1314 error %u ppm, worst CC1314 vs Programmer mismatch %u ppm\n", (unsigned)worst_cc1314,
           (unsigned)worst_link);
    EXPECT_TRUE(worst_cc1314 < 8000);
    EXPECT_TRUE(worst_link < 11000);  // ~1%, near 3 Mbaud.
}

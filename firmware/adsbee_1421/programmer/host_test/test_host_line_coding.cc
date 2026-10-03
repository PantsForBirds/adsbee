// ClassifyHostBaud(): which host line-coding bauds reboot the ADSBee 1421 Programmer into BOOTSEL, which are
// ignored, and that the magic baud is the ADSBee 1090's.
#include "gtest/gtest.h"
#include "host_line_coding.hh"

// Parsed from firmware/adsbee_1090/pico/CMakeLists.txt by host_test/CMakeLists.txt.
#ifndef ADSBEE_1090_RESET_MAGIC_BAUD_RATE
#error "ADSBEE_1090_RESET_MAGIC_BAUD_RATE not defined"
#endif

TEST(HostLineCoding, MagicBaudMatchesAdsbee1090) {
    EXPECT_EQ(kRebootToBootselBaud, (uint32_t)ADSBEE_1090_RESET_MAGIC_BAUD_RATE);
    EXPECT_EQ(kRebootToBootselBaud, 233495534u);
    EXPECT_EQ(ClassifyHostBaud(ADSBEE_1090_RESET_MAGIC_BAUD_RATE), HostBaudAction::kRebootToBootsel);
}

// Every other rate is ignored: the Programmer's UART follows the module's rate.
TEST(HostLineCoding, EveryOtherRateIsIgnored) {
    // Includes 1200 (pico-sdk's default magic baud, used by pymavlink), which must not reboot.
    const uint32_t rates[] = {0, 300, 1200, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 3000000};
    for (uint32_t baud : rates) {
        EXPECT_EQ(ClassifyHostBaud(baud), HostBaudAction::kIgnore) << "baud " << baud;
    }
}

// Only the exact magic baud reboots.
TEST(HostLineCoding, NeighborsAreIgnored) {
    EXPECT_EQ(ClassifyHostBaud(kRebootToBootselBaud - 1), HostBaudAction::kIgnore);
    EXPECT_EQ(ClassifyHostBaud(kRebootToBootselBaud + 1), HostBaudAction::kIgnore);
    EXPECT_EQ(ClassifyHostBaud(0xFFFFFFFFu), HostBaudAction::kIgnore);
}

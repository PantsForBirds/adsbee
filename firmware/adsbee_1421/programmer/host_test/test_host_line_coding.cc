// ClassifyHostBaud(): which host line-coding bauds reboot the ADSBee 1421 Programmer into BOOTSEL, which are
// forwarded to the target UART, and that the magic baud is the ADSBee 1090's.
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

TEST(HostLineCoding, BaudZeroIsIgnored) { EXPECT_EQ(ClassifyHostBaud(0), HostBaudAction::kIgnore); }

// pico-sdk's default magic baud; pymavlink and friends open ports at it. It must not reboot the Programmer.
TEST(HostLineCoding, Baud1200IsForwarded) { EXPECT_EQ(ClassifyHostBaud(1200), HostBaudAction::kApply); }

TEST(HostLineCoding, ConsoleAndCommonRatesAreForwarded) {
    // kConsoleBaudCandidates and kBootloaderBaud (board.hh, which needs the SDK; bridge.cc static_asserts them too),
    // plus rates terminals commonly default to.
    const uint32_t rates[] = {1000000, 921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 300, 3000000};
    for (uint32_t baud : rates) {
        EXPECT_EQ(ClassifyHostBaud(baud), HostBaudAction::kApply) << "baud " << baud;
    }
}

// Only the exact magic baud reboots.
TEST(HostLineCoding, NeighborsAreForwarded) {
    EXPECT_EQ(ClassifyHostBaud(kRebootToBootselBaud - 1), HostBaudAction::kApply);
    EXPECT_EQ(ClassifyHostBaud(kRebootToBootselBaud + 1), HostBaudAction::kApply);
    EXPECT_EQ(ClassifyHostBaud(0xFFFFFFFFu), HostBaudAction::kApply);
}

// Host tests for ClassifyHostBaud(): which host line-coding bauds reboot the ADSBee 1421 Programmer into BOOTSEL,
// which are forwarded to the target UART, and that the magic baud is the ADSBee 1090's.
#include <stdint.h>
#include <stdio.h>

#include "host_line_coding.hh"

static int failures = 0;

#define EXPECT(cond)                                                     \
    do {                                                                 \
        if (!(cond)) {                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            failures++;                                                  \
        }                                                                \
    } while (0)

// Parsed from firmware/adsbee_1090/pico/CMakeLists.txt by host_test/CMakeLists.txt.
#ifndef ADSBEE_1090_RESET_MAGIC_BAUD_RATE
#error "ADSBEE_1090_RESET_MAGIC_BAUD_RATE not defined"
#endif

static void TestMatchesAdsbee1090() {
    printf("magic baud matches the ADSBee 1090\n");
    EXPECT(kRebootToBootselBaud == (uint32_t)ADSBEE_1090_RESET_MAGIC_BAUD_RATE);
    EXPECT(kRebootToBootselBaud == 233495534u);
    EXPECT(ClassifyHostBaud(ADSBEE_1090_RESET_MAGIC_BAUD_RATE) == HostBaudAction::kRebootToBootsel);
}

static void TestNotABaud() {
    printf("baud 0 is ignored\n");
    EXPECT(ClassifyHostBaud(0) == HostBaudAction::kIgnore);
}

// pico-sdk's default magic baud; pymavlink and friends open ports at it. It must not reboot the Programmer.
static void Test1200IsForwarded() {
    printf("1200 baud is forwarded to the module\n");
    EXPECT(ClassifyHostBaud(1200) == HostBaudAction::kApply);
}

static void TestConsoleAndCommonRatesAreForwarded() {
    printf("console, bootloader and common rates are forwarded\n");
    // kConsoleBaudCandidates and kBootloaderBaud (board.hh, which needs the SDK; bridge.cc static_asserts them too),
    // plus rates terminals commonly default to.
    const uint32_t rates[] = {1000000, 921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 300, 3000000};
    for (uint32_t baud : rates) EXPECT(ClassifyHostBaud(baud) == HostBaudAction::kApply);
}

static void TestNeighboursAreForwarded() {
    printf("only the exact magic baud reboots\n");
    EXPECT(ClassifyHostBaud(kRebootToBootselBaud - 1) == HostBaudAction::kApply);
    EXPECT(ClassifyHostBaud(kRebootToBootselBaud + 1) == HostBaudAction::kApply);
    EXPECT(ClassifyHostBaud(0xFFFFFFFFu) == HostBaudAction::kApply);
}

int main() {
    TestMatchesAdsbee1090();
    TestNotABaud();
    Test1200IsForwarded();
    TestConsoleAndCommonRatesAreForwarded();
    TestNeighboursAreForwarded();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}

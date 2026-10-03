#include <string>
#include <vector>

#include "comms/at_console_guard.hh"
#include "cpp_at.hh"
#include "gtest/gtest.h"

namespace {

constexpr uint16_t kBufLen = 1200;  // Same as CommsManager::kATCommandBufMaxLen.
using Assembler = ATLineAssembler<kBufLen>;

// Feeds bytes through an assembler the way CommsManager::UpdateAT() does and returns the lines it would parse.
std::vector<std::string> Feed(Assembler &a, const std::string &bytes, uint32_t timestamp_ms = 0) {
    std::vector<std::string> lines;
    for (char c : bytes) {
        if (a.Push(c, timestamp_ms) == Assembler::kLine) {
            lines.emplace_back(a.line());
        }
    }
    return lines;
}

// A slice of a firmware image around the help text for AT+BOOT_USB_UF2: NUL-terminated strings between machine code.
const std::string kImageSlice = std::string(
    "\x08\xb5\x03\x4b\x00\x22\x1a\x60\n"
    "Reboot ADSBee into RP2040 USB Bootloader.\r\n\tDo not use unless you have a USB connection to the ADSBee\r\n"
    "\tAT+BOOT_USB_UF2=1DEADBEE\0\0\0"
    "Test watchdog by blocking for timeout_sec+1 seconds.\r\n\tAT+WATCHDOG=TEST\0"
    "\x70\x47\x00\xbf\x10\n",
    sizeof("\x08\xb5\x03\x4b\x00\x22\x1a\x60\n"
           "Reboot ADSBee into RP2040 USB Bootloader.\r\n\tDo not use unless you have a USB connection to the ADSBee\r\n"
           "\tAT+BOOT_USB_UF2=1DEADBEE\0\0\0"
           "Test watchdog by blocking for timeout_sec+1 seconds.\r\n\tAT+WATCHDOG=TEST\0"
           "\x70\x47\x00\xbf\x10\n") -
        1);

bool AnyLineHasATCommand(const std::vector<std::string> &lines) {
    for (const auto &l : lines) {
        if (l.find("AT+") != std::string::npos) return true;
    }
    return false;
}

}  // namespace

TEST(ATConsoleGuard, TextLinesPass) {
    ATConsoleGuard guard;
    Assembler a(guard);
    auto lines = Feed(a, "AT+SETTINGS?DUMP\r\nAT+WIFI_STA=1,caf\xc3\xa9,p\xc3\xa4ss\tword\r\n");
    ASSERT_EQ(lines.size(), 2u);
    EXPECT_EQ(lines[0], "AT+SETTINGS?DUMP\r\n");
    EXPECT_EQ(lines[1], "AT+WIFI_STA=1,caf\xc3\xa9,p\xc3\xa4ss\tword\r\n");  // UTF-8 and tabs are text.
}

TEST(ATConsoleGuard, BinaryLinesAreDropped) {
    ATConsoleGuard guard;
    Assembler a(guard);
    // An embedded NUL must not truncate the line to a clean-looking command.
    EXPECT_TRUE(Feed(a, std::string("\tAT+BOOT_USB_UF2=1DEADBEE\0\x01\x02\n", 30)).empty());
    EXPECT_TRUE(Feed(a, "AT+REBOOT\x1b[0m\r\n").empty());  // Control characters.
    EXPECT_TRUE(Feed(a, "AT+REBOOT\x7f\r\n").empty());
    // The assembler recovers for the next text line.
    auto lines = Feed(a, "AT+UPTIME\r\n");
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "AT+UPTIME\r\n");
}

TEST(ATConsoleGuard, ImageSliceNeverYieldsACommand) {
    ATConsoleGuard guard;
    Assembler a(guard);
    // Whatever byte the misparse starts at, no line with an AT command may come out.
    for (size_t start = 0; start < kImageSlice.size(); start++) {
        a.Clear();
        EXPECT_FALSE(AnyLineHasATCommand(Feed(a, kImageSlice.substr(start)))) << "start " << start;
    }
}

TEST(ATConsoleGuard, ShortPayloadRemainderIsDiscarded) {
    ATConsoleGuard guard;
    Assembler a(guard);
    std::string payload = std::string(40, 'x') + kImageSlice;
    guard.BeginBinaryPayload(payload.size());
    EXPECT_TRUE(guard.PayloadInProgress());
    // The WRITE timed out after reading 40 bytes; the rest of the payload arrives afterwards.
    guard.EndBinaryPayload(40, 1000);
    EXPECT_FALSE(guard.PayloadInProgress());
    EXPECT_EQ(guard.DiscardRemainingBytes(), kImageSlice.size());
    uint32_t t = 1000;
    for (char c : kImageSlice) {
        EXPECT_EQ(a.Push(c, t += 1), Assembler::kDiscarded);
    }
    EXPECT_EQ(guard.DiscardRemainingBytes(), 0u);
    // The sender's next command is parsed normally.
    auto lines = Feed(a, "AT+OTA=ERASE,1000,12288\r\n", t);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines[0], "AT+OTA=ERASE,1000,12288\r\n");
}

TEST(ATConsoleGuard, TimedOutPayloadCountsAsOutstanding) {
    ATConsoleGuard guard;
    Assembler a(guard);
    guard.BeginBinaryPayload(12288);
    guard.EndBinaryPayload(-1, 0);  // ATReadConsole() returned -1: timed out.
    EXPECT_EQ(guard.DiscardRemainingBytes(), 12288u);
    EXPECT_EQ(a.Push('A', 10), Assembler::kDiscarded);
}

TEST(ATConsoleGuard, DiscardEndsWhenSenderGoesQuiet) {
    ATConsoleGuard guard;
    Assembler a(guard);
    guard.BeginBinaryPayload(12288);
    guard.EndBinaryPayload(-1, 5000);
    EXPECT_EQ(a.Push('\x01', 5000 + ATConsoleGuard::kDiscardIdleTimeoutMs), Assembler::kDiscarded);
    // More than kDiscardIdleTimeoutMs without payload bytes: the sender gave up, so new input is commands again.
    uint32_t t = 5000 + 2 * ATConsoleGuard::kDiscardIdleTimeoutMs + 1;
    auto lines = Feed(a, "AT+OTA=GET_PARTITION\r\n", t);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(guard.DiscardRemainingBytes(), 0u);
}

TEST(ATConsoleGuard, CompletePayloadLeavesNothingToDiscard) {
    ATConsoleGuard guard;
    Assembler a(guard);
    guard.BeginBinaryPayload(20);
    guard.EndBinaryPayload(20, 0);
    EXPECT_EQ(guard.DiscardRemainingBytes(), 0u);
    EXPECT_EQ(Feed(a, "AT+OTA=VERIFY\r\n").size(), 1u);
}

TEST(ATConsoleGuard, OverflowDropsLine) {
    ATConsoleGuard guard;
    Assembler a(guard);
    std::string long_line(kBufLen + 10, 'a');
    int overflows = 0;
    for (char c : long_line) overflows += a.Push(c, 0) == Assembler::kOverflow;
    EXPECT_EQ(overflows, 1);
}

// End to end with the real parser: a misparsed image slice must not run AT+BOOT_USB_UF2 or AT+WATCHDOG.
static int boot_usb_calls = 0;
static int watchdog_calls = 0;
static bool BootCallback(const CppAT::ATCommandDef_t &def, char op, const std::string_view args[], uint16_t num_args) {
    boot_usb_calls++;
    return true;
}
static bool WatchdogCallback(const CppAT::ATCommandDef_t &def, char op, const std::string_view args[],
                             uint16_t num_args) {
    watchdog_calls++;
    return true;
}

TEST(ATConsoleGuard, ParserNeverSeesImageCommands) {
    const CppAT::ATCommandDef_t commands[] = {
        {.command = "BOOT_USB_UF2", .min_args = 0, .max_args = 1, .help_string = "", .callback = BootCallback},
        {.command = "WATCHDOG", .min_args = 0, .max_args = 1, .help_string = "", .callback = WatchdogCallback},
    };
    CppAT parser(commands, sizeof(commands) / sizeof(commands[0]), true);
    ATConsoleGuard guard;
    Assembler a(guard);
    boot_usb_calls = watchdog_calls = 0;
    for (size_t start = 0; start < kImageSlice.size(); start++) {
        a.Clear();
        for (const auto &l : Feed(a, kImageSlice.substr(start))) parser.ParseMessage(l);
    }
    EXPECT_EQ(boot_usb_calls, 0);
    EXPECT_EQ(watchdog_calls, 0);
    // The same text typed as a command still works.
    for (const auto &l : Feed(a, "AT+WATCHDOG=TEST\r\n")) parser.ParseMessage(l);
    EXPECT_EQ(watchdog_calls, 1);
}

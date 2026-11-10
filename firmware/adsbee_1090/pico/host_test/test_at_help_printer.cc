#include <string>
#include <vector>

#include "comms/at_help_printer.hh"
#include "gtest/gtest.h"

namespace {

std::string out;
bool Write(const char* buf, size_t len) {
    out.append(buf, len);
    return true;
}

std::string LongHelp(size_t len) {
    std::string s;
    for (size_t i = 0; i < len; i++) s.push_back('a' + i % 26);
    return s;
}

// Longer than either product's printf buffer (500 B on the 1090, 1000 B on the 1421).
const std::string kLongHelp = LongHelp(1928);

void CallbackHelp() { out += "\tfrom callback\r\n"; }

const CppAT::ATCommandDef_t kCommands[] = {
    {.command = "ALPHA", .help_string = "AT+ALPHA?\r\n\tQuery alpha."},
    {.command = "LONG", .help_string = kLongHelp},
    {.command = "CB", .help_callback = CallbackHelp},
    {.command = "ZULU", .help_string = "AT+ZULU=<x>"},
};
const uint16_t kNumCommands = sizeof(kCommands) / sizeof(kCommands[0]);

std::string Expected() {
    return std::string("AT Command Help Menu:\r\n") + "ALPHA: \r\n\tAT+ALPHA?\r\n\tQuery alpha.\r\n" + "LONG: \r\n\t" +
           kLongHelp + "\r\n" + "CB: \r\n\tfrom callback\r\n" + "ZULU: \r\n\tAT+ZULU=<x>\r\n";
}

}  // namespace

TEST(ATHelpPrinter, PrintAllListsEveryCommandWhole) {
    out.clear();
    ATHelpPrinter printer(kCommands, kNumCommands);
    printer.PrintAll(Write);
    EXPECT_EQ(out, Expected());
    EXPECT_FALSE(printer.Active());
}

TEST(ATHelpPrinter, PrintNextOneEntryPerCallWithinBound) {
    out.clear();
    ATHelpPrinter printer(kCommands, kNumCommands);
    EXPECT_FALSE(printer.Active());
    EXPECT_EQ(printer.NextEntryMaxBytes(), 0u);
    printer.Start();
    uint16_t calls = 0;
    while (printer.Active()) {
        size_t bound = printer.NextEntryMaxBytes();
        size_t before = out.size();
        printer.PrintNext(Write);
        EXPECT_LE(out.size() - before, bound);
        if (!kCommands[calls].help_callback) {
            EXPECT_EQ(out.size() - before, bound);  // Exact for help strings.
        }
        calls++;
    }
    EXPECT_EQ(calls, kNumCommands);
    EXPECT_EQ(out, Expected());
    EXPECT_FALSE(printer.PrintNext(Write));
    EXPECT_EQ(out, Expected());
}

TEST(ATHelpPrinter, StartRestartsAndStopAbandons) {
    out.clear();
    ATHelpPrinter printer(kCommands, kNumCommands);
    printer.Start();
    printer.PrintNext(Write);
    printer.PrintNext(Write);
    printer.Stop();
    EXPECT_FALSE(printer.Active());
    EXPECT_FALSE(printer.PrintNext(Write));

    out.clear();
    printer.Start();
    while (printer.PrintNext(Write)) {
    }
    EXPECT_EQ(out, Expected());
}

TEST(ATHelpPrinter, EmptyListPrintsHeaderOnly) {
    out.clear();
    ATHelpPrinter printer(kCommands, 0);
    printer.PrintAll(Write);
    EXPECT_EQ(out, "AT Command Help Menu:\r\n");
}

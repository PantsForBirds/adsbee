// CdcTextFinish(): what CdcPrintf() sends for messages that fit its buffer and for ones that don't.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "cdc_text.hh"
#include "gtest/gtest.h"

// Mirrors CdcPrintf(): format into a kCdcTextMax buffer, return what would be written to the port.
static std::string Format(const char* format, ...) __attribute__((format(printf, 1, 2)));
static std::string Format(const char* format, ...) {
    char buf[kCdcTextMax];
    va_list args;
    va_start(args, format);
    int formatted = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    return std::string(buf, CdcTextFinish(buf, sizeof(buf), formatted));
}

TEST(CdcText, MessagesThatFitAreSentUnchanged) {
    EXPECT_EQ(Format("SBL sync at %lu baud.\r\n", 1000000ul), "SBL sync at 1000000 baud.\r\n");
    std::string exact(kCdcTextMax - 3, 'x');
    EXPECT_EQ(Format("%s\r\n", exact.c_str()), exact + "\r\n");  // kCdcTextMax - 1 bytes: the most that fits.
}

TEST(CdcText, TruncatedMessageEndsInEllipsis) {
    std::string longer(kCdcTextMax, 'y');
    std::string out = Format("%s tail\r\n", longer.c_str());
    EXPECT_EQ(out.size(), kCdcTextMax - 1);
    EXPECT_EQ(out.substr(out.size() - 5), "...\r\n");
    EXPECT_EQ(out.substr(0, 10), std::string(10, 'y'));
    std::string one_over(kCdcTextMax - 2, 'z');  // Plus "\r\n": exactly one byte too long.
    out = Format("%s\r\n", one_over.c_str());
    EXPECT_EQ(out.size(), kCdcTextMax - 1);
    EXPECT_EQ(out.substr(out.size() - 5), "...\r\n");
}

TEST(CdcText, EncodingErrorsAndTinyBuffers) {
    char buf[4] = "abc";
    EXPECT_EQ(CdcTextFinish(buf, sizeof(buf), -1), 0u);
    EXPECT_EQ(CdcTextFinish(buf, sizeof(buf), 0), 0u);
    EXPECT_EQ(CdcTextFinish(buf, 0, 5), 0u);
    EXPECT_EQ(CdcTextFinish(buf, sizeof(buf), 10), 3u);  // Too small for the mark: left as vsnprintf cut it.
    EXPECT_STREQ(buf, "abc");
}

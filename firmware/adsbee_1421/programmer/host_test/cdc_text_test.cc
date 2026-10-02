// Host tests for CdcTextFinish(): what CdcPrintf() sends for messages that fit its buffer and for ones that don't.
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "cdc_text.hh"

static int failures = 0;

#define EXPECT(cond)                                                     \
    do {                                                                 \
        if (!(cond)) {                                                   \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            failures++;                                                  \
        }                                                                \
    } while (0)

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

static void TestFits() {
    printf("messages that fit are sent unchanged\n");
    EXPECT(Format("SBL sync at %lu baud.\r\n", 1000000ul) == "SBL sync at 1000000 baud.\r\n");
    std::string exact(kCdcTextMax - 3, 'x');
    EXPECT(Format("%s\r\n", exact.c_str()) == exact + "\r\n");  // kCdcTextMax - 1 bytes: the most that fits.
}

static void TestTruncatedIsMarked() {
    printf("a message that doesn't fit ends in ...\\r\\n\n");
    std::string longer(kCdcTextMax, 'y');
    std::string out = Format("%s tail\r\n", longer.c_str());
    EXPECT(out.size() == kCdcTextMax - 1);
    EXPECT(out.compare(out.size() - 5, 5, "...\r\n") == 0);
    EXPECT(out.compare(0, 10, std::string(10, 'y')) == 0);
    std::string one_over(kCdcTextMax - 2, 'z');  // Plus "\r\n": exactly one byte too long.
    out = Format("%s\r\n", one_over.c_str());
    EXPECT(out.size() == kCdcTextMax - 1 && out.compare(out.size() - 5, 5, "...\r\n") == 0);
}

static void TestEdgeCases() {
    printf("encoding errors and tiny buffers\n");
    char buf[4] = "abc";
    EXPECT(CdcTextFinish(buf, sizeof(buf), -1) == 0);
    EXPECT(CdcTextFinish(buf, sizeof(buf), 0) == 0);
    EXPECT(CdcTextFinish(buf, 0, 5) == 0);
    EXPECT(CdcTextFinish(buf, sizeof(buf), 10) == 3);  // Too small for the mark: left as vsnprintf cut it.
    EXPECT(strcmp(buf, "abc") == 0);
}

int main() {
    TestFits();
    TestTruncatedIsMarked();
    TestEdgeCases();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all passed\n");
    return 0;
}

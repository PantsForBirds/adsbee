#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include "comms/chunked_printf_sink.hh"
#include "gtest/gtest.h"

namespace {

std::vector<std::string> writes;
bool write_ok = true;
bool Write(const char* buf, size_t len) {
    writes.emplace_back(buf, len);
    return write_ok;
}
using Sink = ChunkedPrintfSink<decltype(&Write), 16>;

// Stands in for the Pico SDK's vfctprintf on the host: same output, one character at a time.
int TestFctPrintf(void (*out)(char, void*), void* arg, const char* format, ...) {
    va_list args;
    va_start(args, format);
    char buf[4096];
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    for (int i = 0; i < len; i++) out(buf[i], arg);
    return len;
}

std::string Joined() {
    std::string s;
    for (const std::string& w : writes) s += w;
    return s;
}

}  // namespace

TEST(ChunkedPrintfSink, LongFormattedOutputComesOutWholeInChunks) {
    writes.clear();
    write_ok = true;
    // Longer than the 1090's 500 B console printf buffer.
    std::string long_arg(1200, 'x');
    Sink sink(Write);
    int len = TestFctPrintf(Sink::Putc, &sink, "AT+LONG=%s,%d,%.2f\r\n", long_arg.c_str(), -42, 3.14159);
    EXPECT_TRUE(sink.Flush());
    std::string expected = "AT+LONG=" + long_arg + ",-42,3.14\r\n";
    EXPECT_EQ(len, static_cast<int>(expected.length()));
    EXPECT_EQ(Joined(), expected);
    for (size_t i = 0; i < writes.size(); i++) {
        if (i + 1 < writes.size()) {
            EXPECT_EQ(writes[i].length(), 16u);
        } else {
            EXPECT_GT(writes[i].length(), 0u);
            EXPECT_LE(writes[i].length(), 16u);
        }
    }
}

TEST(ChunkedPrintfSink, ShortOutputIsOneWriteOnFlush) {
    writes.clear();
    write_ok = true;
    Sink sink(Write);
    TestFctPrintf(Sink::Putc, &sink, "OK\r\n");
    EXPECT_TRUE(writes.empty());
    EXPECT_TRUE(sink.Flush());
    ASSERT_EQ(writes.size(), 1u);
    EXPECT_EQ(writes[0], "OK\r\n");
    EXPECT_TRUE(sink.Flush());  // Nothing left: no empty write.
    EXPECT_EQ(writes.size(), 1u);
}

TEST(ChunkedPrintfSink, ExactChunkMultipleHasNoEmptyWrite) {
    writes.clear();
    write_ok = true;
    Sink sink(Write);
    TestFctPrintf(Sink::Putc, &sink, "%s", std::string(32, 'a').c_str());
    EXPECT_TRUE(sink.Flush());
    ASSERT_EQ(writes.size(), 2u);
    EXPECT_EQ(Joined(), std::string(32, 'a'));
}

TEST(ChunkedPrintfSink, FailedWriteIsReportedByFlush) {
    writes.clear();
    write_ok = false;
    Sink sink(Write);
    TestFctPrintf(Sink::Putc, &sink, "%s", std::string(20, 'a').c_str());
    write_ok = true;
    EXPECT_FALSE(sink.Flush());  // The first chunk's write failed.
    EXPECT_EQ(Joined(), std::string(20, 'a'));
}

#include <arpa/inet.h>

#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "ipv6_config.hh"

namespace {

// Splits "a,b,c" into AT arguments.
const char* Parse(const char* arg_str, IPv6Config& out) {
    static std::string storage;
    static std::vector<std::string_view> args;
    storage = arg_str;
    args.clear();
    size_t pos = 0;
    while (true) {
        size_t comma = storage.find(',', pos);
        args.push_back(std::string_view(storage).substr(pos, comma == std::string::npos ? std::string::npos : comma - pos));
        if (comma == std::string::npos) break;
        pos = comma + 1;
    }
    return ParseIPv6ConfigArgs(args.data(), args.size(), out);
}

std::string Addr(const uint8_t a[16]) {
    char buf[IPv6Config::kAddrStrMaxLen + 1];
    FormatIPv6Addr(a, buf, sizeof(buf));
    return buf;
}

bool ParseStr(const char* s, uint8_t out[16]) { return ParseIPv6Addr(s, strlen(s), out); }

}  // namespace

TEST(IPv6Config, ParsesTheUsualForms) {
    uint8_t a[16];
    ASSERT_TRUE(ParseStr("2001:db8::50", a));
    EXPECT_EQ(Addr(a), "2001:db8::50");
    ASSERT_TRUE(ParseStr("2001:0DB8:0000:0000:0000:0000:0000:0050", a));
    EXPECT_EQ(Addr(a), "2001:db8::50");
    ASSERT_TRUE(ParseStr("::", a));
    EXPECT_TRUE(IPv6IsZero(a));
    ASSERT_TRUE(ParseStr("::1", a));
    EXPECT_TRUE(IPv6IsLoopback(a));
    ASSERT_TRUE(ParseStr("fe80::", a));
    EXPECT_TRUE(IPv6IsLinkLocal(a));
    ASSERT_TRUE(ParseStr("1:2:3:4:5:6:7:8", a));
    EXPECT_EQ(Addr(a), "1:2:3:4:5:6:7:8");
    ASSERT_TRUE(ParseStr("1::8", a));
    ASSERT_TRUE(ParseStr("1:2:3:4:5:6:7::", a));
    EXPECT_EQ(Addr(a), "1:2:3:4:5:6:7:0");  // RFC 5952: a single zero group is not compressed.
    ASSERT_TRUE(ParseStr("::2:3:4:5:6:7:8", a));
    EXPECT_EQ(Addr(a), "0:2:3:4:5:6:7:8");
    ASSERT_TRUE(ParseStr("::ffff:192.0.2.1", a));
    EXPECT_EQ(a[10], 0xFF);
    EXPECT_EQ(a[12], 192);
    EXPECT_EQ(a[15], 1);
    ASSERT_TRUE(ParseStr("64:ff9b::10.0.0.1", a));

    for (const char* bad : {"", ":", ":::", "1:2:3:4:5:6:7", "1:2:3:4:5:6:7:8:9", "1::2::3", "12345::", "g::1",
                            "1:2:3:4:5:6:7:8::", "::1:2:3:4:5:6:7:8", ":1::", "1::2:", "fe80::1%eth0", "[::1]",
                            "2001:db8::/64", "::1.2.3", "1.2.3.4", "::1.2.3.4:5", " ::1", "::1 ", "1:2:3:4:5:6:7:1.2.3.4"}) {
        uint8_t b[16];
        memset(b, 7, sizeof(b));
        EXPECT_FALSE(ParseStr(bad, b)) << bad;
        EXPECT_EQ(b[0], 7) << bad;  // Unchanged on failure.
    }
    // Only the first len characters count.
    ASSERT_TRUE(ParseIPv6Addr("2001:db8::1/64", 11, a));
    EXPECT_EQ(Addr(a), "2001:db8::1");
}

TEST(IPv6Config, FormatsLikeInetNtop) {
    const char* samples[] = {"2001:db8::50",    "2001:db8:0:1::",  "::",       "::1",          "fe80::1:2",
                             "2001:db8:0:0:1:0:0:1", "1:0:0:2:0:0:0:3", "ff02::1", "fd12:3456:789a:1::1",
                             "1:2:3:4:5:6:7:8", "0:1:2:3:4:5:6:0", "2001:db8::1:0:0:1"};
    for (const char* s : samples) {
        uint8_t a[16];
        ASSERT_EQ(inet_pton(AF_INET6, s, a), 1) << s;
        char expected[INET6_ADDRSTRLEN];
        ASSERT_NE(inet_ntop(AF_INET6, a, expected, sizeof(expected)), nullptr);
        char got[IPv6Config::kAddrStrMaxLen + 1];
        FormatIPv6Addr(a, got, sizeof(got), false);
        EXPECT_STREQ(got, expected) << s;
        uint8_t back[16];
        ASSERT_TRUE(ParseStr(got, back)) << got;
        EXPECT_EQ(memcmp(a, back, 16), 0) << got;
    }
    uint8_t zero[16] = {};
    EXPECT_EQ(Addr(zero), "");
}

TEST(IPv6Config, ParsesAuto) {
    IPv6Config c;
    c.is_static = true;
    c.prefix_len = 64;
    c.ip[0] = 0x20;
    EXPECT_EQ(Parse("AUTO", c), nullptr);
    EXPECT_FALSE(c.is_static);
    EXPECT_EQ(c.prefix_len, 0);
    EXPECT_TRUE(IPv6IsZero(c.ip));
    c.is_static = true;
    EXPECT_EQ(Parse("auto", c), nullptr);
    EXPECT_FALSE(c.is_static);
    EXPECT_NE(Parse("AUTO,", c), nullptr);
    EXPECT_NE(Parse("SLAAC", c), nullptr);
    EXPECT_NE(Parse("DHCP", c), nullptr);
}

TEST(IPv6Config, ParsesStaticAddresses) {
    IPv6Config c;
    ASSERT_EQ(Parse("2001:db8:1::50/64,fe80::1,2001:4860:4860::8888,2606:4700:4700::1111", c), nullptr);
    EXPECT_TRUE(c.is_static);
    EXPECT_EQ(c.prefix_len, 64);
    EXPECT_EQ(Addr(c.ip), "2001:db8:1::50");
    EXPECT_EQ(Addr(c.gateway), "fe80::1");
    EXPECT_EQ(Addr(c.dns[0]), "2001:4860:4860::8888");
    EXPECT_EQ(Addr(c.dns[1]), "2606:4700:4700::1111");

    // Address only; empty gateway and DNS mean none.
    ASSERT_EQ(Parse("fd00:42::60/48", c), nullptr);
    EXPECT_EQ(c.prefix_len, 48);
    EXPECT_TRUE(IPv6IsZero(c.gateway));
    ASSERT_EQ(Parse("fd00:42::60/64,,,fd00:42::53", c), nullptr);
    EXPECT_TRUE(IPv6IsZero(c.dns[0]));
    EXPECT_EQ(Addr(c.dns[1]), "fd00:42::53");
    // On-prefix gateway.
    EXPECT_EQ(Parse("2001:db8:1::50/64,2001:db8:1::1", c), nullptr);
    // /128 and /1 are allowed.
    EXPECT_EQ(Parse("2001:db8::50/128", c), nullptr);
    EXPECT_EQ(Parse("2001:db8::50/1", c), nullptr);
    // Link-local DNS servers are unicast too.
    EXPECT_EQ(Parse("2001:db8::50/64,,fe80::53", c), nullptr);
}

TEST(IPv6Config, RejectsBadConfigurationsAndLeavesTheOldOne) {
    IPv6Config c;
    ASSERT_EQ(Parse("2001:db8::50/64", c), nullptr);
    const IPv6Config before = c;
    struct Case {
        const char* args;
        const char* error_part;
    } cases[] = {
        {"", "AUTO or"},
        {"2001:db8::50/64,fe80::1,::53,::54,extra", "AUTO or"},
        {"2001:db8::50", "prefix length"},
        {"2001:db8::50/", "1 to 128"},
        {"2001:db8::50/0", "1 to 128"},
        {"2001:db8::50/129", "1 to 128"},
        {"2001:db8::50/64x", "1 to 128"},
        {"2001:db8::50/0064", "1 to 128"},
        {"2001:db8::zz/64", "not valid"},
        {"::/64", "global"},
        {"::1/128", "global"},
        {"ff02::1/64", "global"},
        {"fe80::50/64", "global"},
        {"::ffff:10.0.0.1/96", "global"},
        {"fec0::1/64", "global"},
        {"2001:db8::/64", "anycast"},
        {"2001:db8:1::50/64,2001:db8:2::1", "Gateway must"},
        {"2001:db8:1::50/64,2001:db8:1::", "Gateway must"},
        {"2001:db8:1::50/64,ff02::2", "Gateway must"},
        {"2001:db8:1::50/64,2001:db8:1::50", "differ"},
        {"2001:db8:1::50/64,nope", "Gateway must be an IPv6"},
        {"2001:db8:1::50/64,,ff02::fb", "DNS server must be a unicast"},
        {"2001:db8:1::50/64,,::1", "DNS server must be a unicast"},
        {"2001:db8:1::50/64,,,8.8.8.8", "DNS server must be an IPv6"},
    };
    for (const Case& tc : cases) {
        const char* error = Parse(tc.args, c);
        ASSERT_NE(error, nullptr) << tc.args;
        EXPECT_NE(strstr(error, tc.error_part), nullptr) << tc.args << ": " << error;
        EXPECT_EQ(memcmp(&c, &before, sizeof(c)), 0) << tc.args;
    }
}

TEST(IPv6Config, ValidatesStoredConfigurations) {
    IPv6Config c;
    EXPECT_EQ(ValidateIPv6Config(c), nullptr);
    c.dns[1][0] = 0x20;  // AUTO with an address left behind.
    EXPECT_NE(ValidateIPv6Config(c), nullptr);
    c = IPv6Config();
    c.prefix_len = 64;
    EXPECT_NE(ValidateIPv6Config(c), nullptr);
    c = IPv6Config();
    c.is_static = true;
    ASSERT_TRUE(ParseStr("2001:db8::50", c.ip));
    EXPECT_NE(ValidateIPv6Config(c), nullptr);  // Prefix length 0.
    c.prefix_len = 200;
    EXPECT_NE(ValidateIPv6Config(c), nullptr);
    c.prefix_len = 64;
    EXPECT_EQ(ValidateIPv6Config(c), nullptr);
}

TEST(IPv6Config, PrefixHelpers) {
    uint8_t a[16], b[16];
    ASSERT_TRUE(ParseStr("2001:db8:1:2::1", a));
    ASSERT_TRUE(ParseStr("2001:db8:1:3::1", b));
    EXPECT_TRUE(IPv6PrefixMatch(a, b, 48));
    EXPECT_TRUE(IPv6PrefixMatch(a, b, 63));
    EXPECT_FALSE(IPv6PrefixMatch(a, b, 64));
    EXPECT_TRUE(IPv6PrefixMatch(a, b, 0));
    EXPECT_FALSE(IPv6HostBitsZero(a, 64));
    EXPECT_TRUE(IPv6HostBitsZero(a, 128));
    ASSERT_TRUE(ParseStr("2001:db8:1:3::", a));
    EXPECT_TRUE(IPv6HostBitsZero(a, 64));
    EXPECT_FALSE(IPv6HostBitsZero(a, 63));
}

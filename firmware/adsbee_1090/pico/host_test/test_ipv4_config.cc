#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "ipv4_config.hh"

namespace {

// Splits "a,b,c" into AT arguments.
const char* Parse(const char* arg_str, IPv4Config& out) {
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
    return ParseIPv4ConfigArgs(args.data(), args.size(), out);
}

std::string Addr(const uint8_t a[4]) {
    char buf[IPv4Config::kAddrStrMaxLen + 1];
    FormatIPv4Addr(a, buf, sizeof(buf));
    return buf;
}

}  // namespace

TEST(IPv4Config, ParsesDottedQuads) {
    uint8_t a[4] = {9, 9, 9, 9};
    EXPECT_TRUE(ParseIPv4Addr("192.168.1.50", 12, a));
    EXPECT_EQ(Addr(a), "192.168.1.50");
    EXPECT_TRUE(ParseIPv4Addr("0.0.0.0", 7, a));
    EXPECT_TRUE(ParseIPv4Addr("255.255.255.255", 15, a));
    EXPECT_EQ(Addr(a), "255.255.255.255");
    for (const char* bad : {"", "1.2.3", "1.2.3.4.5", "256.1.1.1", "1.2.3.4 ", " 1.2.3.4", "1..2.3", "1.2.3.", "a.b.c.d",
                            "1.2.3.0004", "-1.2.3.4", "1.2.3.4/24", "1,2,3,4"}) {
        uint8_t b[4] = {7, 7, 7, 7};
        EXPECT_FALSE(ParseIPv4Addr(bad, strlen(bad), b)) << bad;
        EXPECT_EQ(b[0], 7) << bad;  // Unchanged on failure.
    }
    // Only the first len characters count.
    EXPECT_TRUE(ParseIPv4Addr("10.0.0.1,junk", 8, a));
    EXPECT_EQ(Addr(a), "10.0.0.1");
}

TEST(IPv4Config, ParsesNetmasksAsQuadsOrPrefixLengths) {
    uint8_t m[4];
    EXPECT_TRUE(ParseIPv4Netmask("255.255.255.0", 13, m));
    EXPECT_EQ(IPv4NetmaskPrefixLen(m), 24);
    EXPECT_TRUE(ParseIPv4Netmask("24", 2, m));
    EXPECT_EQ(Addr(m), "255.255.255.0");
    EXPECT_TRUE(ParseIPv4Netmask("/20", 3, m));
    EXPECT_EQ(Addr(m), "255.255.240.0");
    EXPECT_TRUE(ParseIPv4Netmask("1", 1, m));
    EXPECT_EQ(Addr(m), "128.0.0.0");
    EXPECT_TRUE(ParseIPv4Netmask("255.255.255.252", 15, m));
    for (const char* bad : {"", "/", "0", "31", "32", "33", "255.255.255.254", "255.255.255.255", "0.0.0.0",
                            "255.0.255.0", "255.255.255.1", "24a", "/2/4", "255.255.255"}) {
        EXPECT_FALSE(ParseIPv4Netmask(bad, strlen(bad), m)) << bad;
    }
    uint8_t holes[4] = {255, 0, 255, 0};
    EXPECT_EQ(IPv4NetmaskPrefixLen(holes), -1);
}

TEST(IPv4Config, ParsesDHCP) {
    IPv4Config c;
    c.is_static = true;
    c.ip[0] = 10;
    EXPECT_EQ(Parse("DHCP", c), nullptr);
    EXPECT_FALSE(c.is_static);
    EXPECT_TRUE(IPv4IsZero(c.ip));
    c.is_static = true;
    EXPECT_EQ(Parse("dhcp", c), nullptr);
    EXPECT_FALSE(c.is_static);
    EXPECT_NE(Parse("DHCPX", c), nullptr);
    EXPECT_NE(Parse("DHCP,1.2.3.4", c), nullptr);
}

TEST(IPv4Config, ParsesStaticConfigurations) {
    IPv4Config c;
    ASSERT_EQ(Parse("192.168.1.50,255.255.255.0,192.168.1.1,1.1.1.1,9.9.9.9", c), nullptr);
    EXPECT_TRUE(c.is_static);
    EXPECT_EQ(Addr(c.ip), "192.168.1.50");
    EXPECT_EQ(Addr(c.netmask), "255.255.255.0");
    EXPECT_EQ(Addr(c.gateway), "192.168.1.1");
    EXPECT_EQ(Addr(c.dns[0]), "1.1.1.1");
    EXPECT_EQ(Addr(c.dns[1]), "9.9.9.9");

    // Gateway and DNS can be empty or left out.
    ASSERT_EQ(Parse("10.1.2.3,8", c), nullptr);
    EXPECT_EQ(Addr(c.netmask), "255.0.0.0");
    EXPECT_TRUE(IPv4IsZero(c.gateway));
    EXPECT_TRUE(IPv4IsZero(c.dns[0]));
    ASSERT_EQ(Parse("10.1.2.3,8,,", c), nullptr);
    ASSERT_EQ(Parse("10.1.2.3,8,,,8.8.4.4", c), nullptr);
    EXPECT_TRUE(IPv4IsZero(c.dns[0]));
    EXPECT_EQ(Addr(c.dns[1]), "8.8.4.4");
    // 169.254.x.x and an empty-gateway /30 are fine.
    EXPECT_EQ(Parse("169.254.10.20,16", c), nullptr);
    EXPECT_EQ(Parse("192.168.7.1,30,192.168.7.2", c), nullptr);
}

TEST(IPv4Config, RejectsBadStaticConfigurations) {
    IPv4Config c;
    ASSERT_EQ(Parse("192.168.1.50,24,192.168.1.1", c), nullptr);
    const IPv4Config before = c;
    struct Case {
        const char* args;
        const char* error_part;
    };
    const Case cases[] = {
        {"", "Use IPV4,DHCP"},
        {"192.168.1.50", "Use IPV4,DHCP"},
        {"192.168.1.50,24,192.168.1.1,1.1.1.1,9.9.9.9,8.8.8.8", "Use IPV4,DHCP"},
        {"192.168.1.500,24", "IP address must be a dotted quad"},
        {",24", "IP address must be a dotted quad"},
        {"192.168.1.50,", "Netmask"},
        {"192.168.1.50,255.0.255.0", "Netmask"},
        {"192.168.1.50,32", "Netmask"},
        {"192.168.1.50,24,gateway", "Gateway must be a dotted quad"},
        {"192.168.1.50,24,,dns", "DNS server must be a dotted quad"},
        {"0.0.0.0,24", "unicast"},
        {"0.1.2.3,8", "unicast"},
        {"127.0.0.2,8", "unicast"},
        {"224.0.0.5,8", "unicast"},
        {"255.255.255.255,24", "unicast"},
        {"192.168.1.0,24", "network or broadcast"},
        {"192.168.1.255,24", "network or broadcast"},
        {"192.168.1.50,24,192.168.2.1", "Gateway must be a host on"},
        {"192.168.1.50,24,192.168.1.255", "Gateway must be a host on"},
        {"192.168.1.50,24,192.168.1.0", "Gateway must be a host on"},
        {"192.168.1.50,24,192.168.1.50", "differ"},
        {"192.168.1.50,24,192.168.1.1,255.255.255.255", "DNS server must be a unicast"},
        {"192.168.1.50,24,192.168.1.1,,224.0.0.251", "DNS server must be a unicast"},
    };
    for (const Case& t : cases) {
        const char* error = Parse(t.args, c);
        ASSERT_NE(error, nullptr) << t.args;
        EXPECT_NE(strstr(error, t.error_part), nullptr) << t.args << ": " << error;
        EXPECT_EQ(memcmp(&c, &before, sizeof(c)), 0) << t.args;  // Unchanged on error.
    }
}

TEST(IPv4Config, ValidatesStoredConfigurations) {
    IPv4Config c;
    EXPECT_EQ(ValidateIPv4Config(c), nullptr);  // DHCP.
    c.gateway[3] = 1;
    EXPECT_NE(ValidateIPv4Config(c), nullptr);  // DHCP with an address left in.
    c = IPv4Config();
    c.is_static = true;
    EXPECT_NE(ValidateIPv4Config(c), nullptr);  // Static with nothing set.
    IPv4FromU32(0x0A000005, c.ip);
    IPv4FromU32(0xFFFF0000, c.netmask);
    EXPECT_EQ(ValidateIPv4Config(c), nullptr);
}

TEST(IPv4Config, FormatsZeroAsEmpty) {
    uint8_t zero[4] = {};
    char buf[IPv4Config::kAddrStrMaxLen + 1] = "x";
    FormatIPv4Addr(zero, buf, sizeof(buf));
    EXPECT_STREQ(buf, "");
    FormatIPv4Addr(zero, buf, sizeof(buf), false);
    EXPECT_STREQ(buf, "0.0.0.0");
}

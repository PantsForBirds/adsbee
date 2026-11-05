#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"
#include "static_ip_config.hh"

namespace {

// Splits "a,b,c" into AT arguments, as cppAT does after the operator.
const char* Parse(const char* arg_str, IPv4Config& v4, IPv6Config& v6, IPFamily& family) {
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
    return ParseStaticIPArgs(args.data(), args.size(), v4, v6, family);
}

}  // namespace

TEST(StaticIPConfig, FamilyPicksTheConfiguration) {
    IPv4Config v4;
    IPv6Config v6;
    IPFamily family = IPFamily::kIPv6;
    ASSERT_EQ(Parse("IPV4,192.168.1.50,24,192.168.1.1", v4, v6, family), nullptr);
    EXPECT_EQ(family, IPFamily::kIPv4);
    EXPECT_TRUE(v4.is_static);
    EXPECT_FALSE(v6.is_static);

    ASSERT_EQ(Parse("IPV6,2001:db8::50/64,fe80::1,::ffff:8.8.8.8", v4, v6, family), nullptr);
    EXPECT_EQ(family, IPFamily::kIPv6);
    EXPECT_TRUE(v6.is_static);
    EXPECT_TRUE(v4.is_static);  // Untouched.

    // Case-insensitive, like AT+IP_MODE.
    ASSERT_EQ(Parse("ipv4,dhcp", v4, v6, family), nullptr);
    EXPECT_FALSE(v4.is_static);
    ASSERT_EQ(Parse("Ipv6,Auto", v4, v6, family), nullptr);
    EXPECT_FALSE(v6.is_static);
}

TEST(StaticIPConfig, MissingOrUnknownFamilyListsTheForms) {
    IPv4Config v4;
    IPv6Config v6;
    IPFamily first;
    ASSERT_EQ(Parse("IPV6,2001:db8::50/64", v4, v6, first), nullptr);
    const IPv6Config before6 = v6;
    IPFamily family = IPFamily::kIPv4;
    for (const char* bad : {"", "DHCP", "AUTO", "192.168.1.50,24", "2001:db8::50/64", "IPV5,DHCP", "IP,DHCP", "IPV44,DHCP",
                            "DUAL,DHCP", ",IPV4,DHCP"}) {
        const char* error = Parse(bad, v4, v6, family);
        ASSERT_NE(error, nullptr) << bad;
        EXPECT_NE(strstr(error, "IPV4 or IPV6"), nullptr) << bad << ": " << error;
        for (const char* form : {"IPV4,DHCP", "IPV4,<ip>,<netmask>", "IPV6,AUTO", "IPV6,<addr>/<prefix>"}) {
            EXPECT_NE(strstr(error, form), nullptr) << bad << ": " << error;
        }
        EXPECT_EQ(memcmp(&v6, &before6, sizeof(v6)), 0) << bad;
        EXPECT_FALSE(v4.is_static) << bad;
    }
}

TEST(StaticIPConfig, FamilyErrorsComeFromThatFamilysParser) {
    IPv4Config v4;
    IPv6Config v6;
    IPFamily family;
    const char* error = Parse("IPV4", v4, v6, family);
    ASSERT_NE(error, nullptr);
    EXPECT_NE(strstr(error, "IPV4,DHCP or IPV4,<ip>"), nullptr) << error;
    error = Parse("IPV6", v4, v6, family);
    ASSERT_NE(error, nullptr);
    EXPECT_NE(strstr(error, "IPV6,AUTO or IPV6,<addr>"), nullptr) << error;
    error = Parse("IPV4,10.0.0.1,24,10.1.0.1", v4, v6, family);
    ASSERT_NE(error, nullptr);
    EXPECT_NE(strstr(error, "Gateway"), nullptr) << error;
    error = Parse("IPV6,fe80::50/64", v4, v6, family);
    ASSERT_NE(error, nullptr);
    EXPECT_NE(strstr(error, "global"), nullptr) << error;
    // The address is no longer the first argument, so "::" forms reach the parser intact (cppAT only strips
    // non-alphanumerics right after '=') and get the right error.
    error = Parse("IPV6,::1/128", v4, v6, family);
    ASSERT_NE(error, nullptr);
    EXPECT_NE(strstr(error, "global"), nullptr) << error;
    EXPECT_FALSE(v4.is_static);
    EXPECT_FALSE(v6.is_static);
}

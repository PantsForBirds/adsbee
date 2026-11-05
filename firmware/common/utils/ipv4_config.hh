#ifndef IPV4_CONFIG_HH_
#define IPV4_CONFIG_HH_

#include <cstddef>
#include <cstdint>
#include <cstdio>

// IPv4 addressing of the Ethernet or WiFi station interface: DHCP, or a static address (AT+ETHERNET_IP,
// AT+WIFI_STA_IP). Addresses are octets in dotted-quad order; 0.0.0.0 means not set.
struct IPv4Config {
    static constexpr uint16_t kNumDNSServers = 2;
    static constexpr uint16_t kAddrStrMaxLen = 15;  // "255.255.255.255"
    static constexpr int kMinPrefixLen = 1;
    static constexpr int kMaxPrefixLen = 30;  // /31 and /32 have no room for a gateway.

    bool is_static = false;  // False: DHCP, and every address is 0.0.0.0.
    uint8_t ip[4] = {};
    uint8_t netmask[4] = {};
    uint8_t gateway[4] = {};
    uint8_t dns[kNumDNSServers][4] = {};
};

inline uint32_t IPv4ToU32(const uint8_t a[4]) {
    return (uint32_t(a[0]) << 24) | (uint32_t(a[1]) << 16) | (uint32_t(a[2]) << 8) | uint32_t(a[3]);
}

inline void IPv4FromU32(uint32_t v, uint8_t a[4]) {
    a[0] = v >> 24;
    a[1] = v >> 16;
    a[2] = v >> 8;
    a[3] = v;
}

inline bool IPv4IsZero(const uint8_t a[4]) { return IPv4ToU32(a) == 0; }

// Writes "a.b.c.d", or "" for 0.0.0.0 when zero_is_empty is set. buf_len should be at least kAddrStrMaxLen + 1.
inline void FormatIPv4Addr(const uint8_t a[4], char* buf, size_t buf_len, bool zero_is_empty = true) {
    if (zero_is_empty && IPv4IsZero(a)) {
        if (buf_len > 0) buf[0] = '\0';
        return;
    }
    snprintf(buf, buf_len, "%u.%u.%u.%u", a[0], a[1], a[2], a[3]);
}

// Parses a dotted quad: four decimal parts of 0-255 with at most three digits each, nothing else.
inline bool ParseIPv4Addr(const char* str, size_t len, uint8_t out[4]) {
    uint8_t octets[4];
    size_t i = 0;
    for (int part = 0; part < 4; part++) {
        if (part > 0) {
            if (i >= len || str[i] != '.') return false;
            i++;
        }
        uint16_t value = 0;
        size_t num_digits = 0;
        while (i < len && str[i] >= '0' && str[i] <= '9' && num_digits < 3) {
            value = value * 10 + (str[i] - '0');
            i++;
            num_digits++;
        }
        if (num_digits == 0 || value > 255) return false;
        octets[part] = value;
    }
    if (i != len) return false;
    for (int part = 0; part < 4; part++) out[part] = octets[part];
    return true;
}

// Prefix length of a contiguous netmask, or -1 if the mask has holes.
inline int IPv4NetmaskPrefixLen(const uint8_t mask[4]) {
    uint32_t m = IPv4ToU32(mask);
    int prefix_len = 0;
    while (prefix_len < 32 && (m & (0x80000000u >> prefix_len))) prefix_len++;
    uint32_t expected = prefix_len == 0 ? 0 : 0xFFFFFFFFu << (32 - prefix_len);
    return m == expected ? prefix_len : -1;
}

// Parses a netmask as a dotted quad or a prefix length ("255.255.255.0", "24" or "/24"), /1 to /30.
inline bool ParseIPv4Netmask(const char* str, size_t len, uint8_t out[4]) {
    size_t start = (len > 0 && str[0] == '/') ? 1 : 0;
    if (len > start && len - start <= 2) {
        int prefix_len = 0;
        for (size_t i = start; i < len; i++) {
            if (str[i] < '0' || str[i] > '9') return false;
            prefix_len = prefix_len * 10 + (str[i] - '0');
        }
        if (prefix_len < IPv4Config::kMinPrefixLen || prefix_len > IPv4Config::kMaxPrefixLen) return false;
        IPv4FromU32(0xFFFFFFFFu << (32 - prefix_len), out);
        return true;
    }
    uint8_t mask[4];
    if (start != 0 || !ParseIPv4Addr(str, len, mask)) return false;
    int prefix_len = IPv4NetmaskPrefixLen(mask);
    if (prefix_len < IPv4Config::kMinPrefixLen || prefix_len > IPv4Config::kMaxPrefixLen) return false;
    for (int i = 0; i < 4; i++) out[i] = mask[i];
    return true;
}

// Not 0.x.x.x, loopback, multicast, reserved or broadcast.
inline bool IPv4IsUnicast(const uint8_t a[4]) { return a[0] != 0 && a[0] != 127 && a[0] < 224; }

/**
 * Checks a configuration.
 * @retval nullptr if it can be applied, else an error message.
 */
inline const char* ValidateIPv4Config(const IPv4Config& c) {
    if (!c.is_static) {
        bool all_zero = IPv4IsZero(c.ip) && IPv4IsZero(c.netmask) && IPv4IsZero(c.gateway);
        for (uint16_t i = 0; i < IPv4Config::kNumDNSServers; i++) all_zero &= IPv4IsZero(c.dns[i]);
        return all_zero ? nullptr : "DHCP configuration has addresses set.";
    }
    int prefix_len = IPv4NetmaskPrefixLen(c.netmask);
    if (prefix_len < IPv4Config::kMinPrefixLen || prefix_len > IPv4Config::kMaxPrefixLen) {
        return "Netmask must be contiguous, from /1 to /30, e.g. 255.255.255.0 or 24.";
    }
    uint32_t mask = IPv4ToU32(c.netmask);
    uint32_t ip = IPv4ToU32(c.ip);
    if (!IPv4IsUnicast(c.ip)) {
        return "IP address must be a unicast address (not 0.x.x.x, 127.x.x.x, multicast or broadcast).";
    }
    if ((ip & ~mask) == 0 || (ip & ~mask) == ~mask) {
        return "IP address is the network or broadcast address of its subnet.";
    }
    if (!IPv4IsZero(c.gateway)) {
        uint32_t gw = IPv4ToU32(c.gateway);
        if ((gw & mask) != (ip & mask) || (gw & ~mask) == 0 || (gw & ~mask) == ~mask) {
            return "Gateway must be a host on the IP address's subnet, or empty for none.";
        }
        if (gw == ip) return "Gateway must differ from the IP address.";
    }
    for (uint16_t i = 0; i < IPv4Config::kNumDNSServers; i++) {
        if (!IPv4IsZero(c.dns[i]) && !IPv4IsUnicast(c.dns[i])) {
            return "DNS server must be a unicast address, or empty for none.";
        }
    }
    return nullptr;
}

/**
 * Parses AT+ETHERNET_IP / AT+WIFI_STA_IP arguments: DHCP, or <ip>,<netmask>[,<gateway>[,<dns1>[,<dns2>]]].
 * Empty gateway and DNS arguments mean none. StrView needs data() and size() (std::string_view).
 * @retval nullptr on success (out is set), else an error message (out is unchanged).
 */
template <typename StrView>
const char* ParseIPv4ConfigArgs(const StrView args[], uint16_t num_args, IPv4Config& out) {
    static const char* const kUsage = "Use DHCP or <ip>,<netmask>[,<gateway>[,<dns1>[,<dns2>]]].";
    IPv4Config c;
    if (num_args == 1 && args[0].size() == 4) {
        const char* s = args[0].data();
        const char* dhcp = "DHCP";
        bool match = true;
        for (int i = 0; i < 4; i++) match &= (s[i] >= 'a' && s[i] <= 'z' ? s[i] - 'a' + 'A' : s[i]) == dhcp[i];
        if (match) {
            out = c;
            return nullptr;
        }
    }
    if (num_args < 2 || num_args > 2 + IPv4Config::kNumDNSServers + 1) return kUsage;
    c.is_static = true;
    if (!ParseIPv4Addr(args[0].data(), args[0].size(), c.ip)) {
        return "IP address must be a dotted quad, e.g. 192.168.1.50.";
    }
    if (!ParseIPv4Netmask(args[1].data(), args[1].size(), c.netmask)) {
        return "Netmask must be contiguous, from /1 to /30, e.g. 255.255.255.0 or 24.";
    }
    if (num_args > 2 && args[2].size() > 0 && !ParseIPv4Addr(args[2].data(), args[2].size(), c.gateway)) {
        return "Gateway must be a dotted quad, or empty for none.";
    }
    for (uint16_t i = 0; i < IPv4Config::kNumDNSServers; i++) {
        uint16_t arg = 3 + i;
        if (num_args > arg && args[arg].size() > 0 && !ParseIPv4Addr(args[arg].data(), args[arg].size(), c.dns[i])) {
            return "DNS server must be a dotted quad, or empty for none.";
        }
    }
    const char* error = ValidateIPv4Config(c);
    if (error != nullptr) return error;
    out = c;
    return nullptr;
}

#endif /* IPV4_CONFIG_HH_ */

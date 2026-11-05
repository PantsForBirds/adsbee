#ifndef IPV6_CONFIG_HH_
#define IPV6_CONFIG_HH_

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "ipv4_config.hh"  // ParseIPv4Addr for embedded IPv4 (::ffff:1.2.3.4).

// IPv6 addressing of the Ethernet or WiFi station interface: AUTO (link-local + SLAAC), or a static address (AT+
// ETHERNET_IP6, AT+WIFI_STA_IP6). Addresses are 16 bytes in network order; :: means not set.
struct IPv6Config {
    static constexpr uint16_t kNumDNSServers = 2;
    static constexpr uint16_t kAddrStrMaxLen = 45;  // "ffff:ffff:ffff:ffff:ffff:ffff:255.255.255.255"
    static constexpr int kMinPrefixLen = 1;
    static constexpr int kMaxPrefixLen = 128;

    bool is_static = false;   // False: AUTO, prefix_len is 0 and every address is ::.
    uint8_t prefix_len = 0;
    uint8_t ip[16] = {};
    uint8_t gateway[16] = {};
    uint8_t dns[kNumDNSServers][16] = {};
};

inline bool IPv6IsZero(const uint8_t a[16]) {
    for (int i = 0; i < 16; i++) {
        if (a[i] != 0) return false;
    }
    return true;
}

inline bool IPv6IsLoopback(const uint8_t a[16]) {
    for (int i = 0; i < 15; i++) {
        if (a[i] != 0) return false;
    }
    return a[15] == 1;
}

inline bool IPv6IsMulticast(const uint8_t a[16]) { return a[0] == 0xFF; }
inline bool IPv6IsLinkLocal(const uint8_t a[16]) { return a[0] == 0xFE && (a[1] & 0xC0) == 0x80; }  // fe80::/10
inline bool IPv6IsGlobal(const uint8_t a[16]) { return (a[0] & 0xE0) == 0x20; }                     // 2000::/3
inline bool IPv6IsUniqueLocal(const uint8_t a[16]) { return (a[0] & 0xFE) == 0xFC; }                // fc00::/7

// True if the first prefix_len bits of a and b match.
inline bool IPv6PrefixMatch(const uint8_t a[16], const uint8_t b[16], int prefix_len) {
    for (int i = 0; i < 16 && prefix_len > 0; i++, prefix_len -= 8) {
        uint8_t mask = prefix_len >= 8 ? 0xFF : static_cast<uint8_t>(0xFF << (8 - prefix_len));
        if ((a[i] & mask) != (b[i] & mask)) return false;
    }
    return true;
}

// True if every bit after the first prefix_len bits is zero.
inline bool IPv6HostBitsZero(const uint8_t a[16], int prefix_len) {
    for (int bit = prefix_len; bit < 128; bit++) {
        if (a[bit / 8] & (0x80 >> (bit % 8))) return false;
    }
    return true;
}

// Writes the RFC 5952 text form (lowercase, longest run of two or more zero groups as ::), or "" for :: when
// zero_is_empty is set. buf_len should be at least kAddrStrMaxLen + 1.
inline void FormatIPv6Addr(const uint8_t a[16], char* buf, size_t buf_len, bool zero_is_empty = true) {
    if (buf_len == 0) return;
    buf[0] = '\0';
    if (zero_is_empty && IPv6IsZero(a)) return;
    uint16_t groups[8];
    for (int i = 0; i < 8; i++) groups[i] = (uint16_t(a[2 * i]) << 8) | a[2 * i + 1];
    int best_start = -1, best_len = 0;
    for (int i = 0; i < 8;) {
        if (groups[i] != 0) {
            i++;
            continue;
        }
        int j = i;
        while (j < 8 && groups[j] == 0) j++;
        if (j - i > best_len && j - i >= 2) {
            best_start = i;
            best_len = j - i;
        }
        i = j;
    }
    size_t len = 0;
    for (int i = 0; i < 8 && len < buf_len; i++) {
        if (i == best_start) {
            len += snprintf(buf + len, buf_len - len, i == 0 ? "::" : ":");
            i += best_len - 1;
            continue;
        }
        len += snprintf(buf + len, buf_len - len, "%x%s", groups[i], i < 7 ? ":" : "");
    }
}

// Parses an IPv6 address in the usual text forms: eight groups of 1-4 hex digits, one "::" for a run of zero groups,
// and an optional dotted quad as the last 32 bits. No zone index (%eth0), no brackets, no prefix.
inline bool ParseIPv6Addr(const char* str, size_t len, uint8_t out[16]) {
    uint16_t groups[8] = {};
    int num_groups = 0;
    int gap = -1;  // Group index where "::" sits.
    size_t i = 0;
    if (len >= 2 && str[0] == ':' && str[1] == ':') {
        gap = 0;
        i = 2;
    } else if (len >= 1 && str[0] == ':') {
        return false;
    }
    while (i < len) {
        if (num_groups >= 8) return false;
        // Embedded IPv4: a '.' before the next ':'.
        size_t end = i;
        while (end < len && str[end] != ':') end++;
        bool is_v4 = false;
        for (size_t k = i; k < end; k++) is_v4 |= str[k] == '.';
        if (is_v4) {
            uint8_t v4[4];
            if (end != len || num_groups > 6 || !ParseIPv4Addr(str + i, end - i, v4)) return false;
            groups[num_groups++] = (uint16_t(v4[0]) << 8) | v4[1];
            groups[num_groups++] = (uint16_t(v4[2]) << 8) | v4[3];
            i = end;
            break;
        }
        size_t num_digits = end - i;
        if (num_digits == 0 || num_digits > 4) return false;
        uint16_t value = 0;
        for (size_t k = i; k < end; k++) {
            char c = str[k];
            uint8_t d;
            if (c >= '0' && c <= '9') d = c - '0';
            else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
            else return false;
            value = (value << 4) | d;
        }
        groups[num_groups++] = value;
        i = end;
        if (i == len) break;
        // At a ':'. "::" marks the gap; a single ':' must be followed by another group.
        i++;
        if (i < len && str[i] == ':') {
            if (gap >= 0) return false;
            gap = num_groups;
            i++;
        } else if (i == len) {
            return false;  // Trailing single ':'.
        }
    }
    if (gap < 0) {
        if (num_groups != 8) return false;
    } else {
        if (num_groups > 7) return false;  // "::" must stand for at least one group.
        int tail = num_groups - gap;
        for (int k = 0; k < tail; k++) groups[7 - k] = groups[num_groups - 1 - k];
        for (int k = gap; k < 8 - tail; k++) groups[k] = 0;
    }
    for (int k = 0; k < 8; k++) {
        out[2 * k] = groups[k] >> 8;
        out[2 * k + 1] = groups[k] & 0xFF;
    }
    return true;
}

/**
 * Checks a configuration.
 * @retval nullptr if it can be applied, else an error message.
 */
inline const char* ValidateIPv6Config(const IPv6Config& c) {
    if (!c.is_static) {
        bool all_zero = c.prefix_len == 0 && IPv6IsZero(c.ip) && IPv6IsZero(c.gateway);
        for (uint16_t i = 0; i < IPv6Config::kNumDNSServers; i++) all_zero &= IPv6IsZero(c.dns[i]);
        return all_zero ? nullptr : "AUTO configuration has addresses set.";
    }
    if (c.prefix_len < IPv6Config::kMinPrefixLen || c.prefix_len > IPv6Config::kMaxPrefixLen) {
        return "Prefix length must be from 1 to 128, usually 64.";
    }
    if (!IPv6IsGlobal(c.ip) && !IPv6IsUniqueLocal(c.ip)) {
        return "IPv6 address must be a global (2000::/3) or unique local (fc00::/7) unicast address.";
    }
    if (c.prefix_len < 127 && IPv6HostBitsZero(c.ip, c.prefix_len)) {
        return "IPv6 address is its prefix's subnet-router anycast address (all host bits zero).";
    }
    if (!IPv6IsZero(c.gateway)) {
        bool link_local = IPv6IsLinkLocal(c.gateway);
        bool on_prefix = (IPv6IsGlobal(c.gateway) || IPv6IsUniqueLocal(c.gateway)) &&
                         IPv6PrefixMatch(c.gateway, c.ip, c.prefix_len) &&
                         !(c.prefix_len < 127 && IPv6HostBitsZero(c.gateway, c.prefix_len));
        if (!link_local && !on_prefix) {
            return "Gateway must be a link-local (fe80::) address or a host on the address's prefix, or empty for "
                   "none.";
        }
        bool same = true;
        for (int i = 0; i < 16; i++) same &= c.gateway[i] == c.ip[i];
        if (same) return "Gateway must differ from the IPv6 address.";
    }
    for (uint16_t i = 0; i < IPv6Config::kNumDNSServers; i++) {
        const uint8_t* d = c.dns[i];
        if (!IPv6IsZero(d) && (IPv6IsMulticast(d) || IPv6IsLoopback(d))) {
            return "DNS server must be a unicast IPv6 address, or empty for none.";
        }
    }
    return nullptr;
}

/**
 * Parses AT+ETHERNET_IP6 / AT+WIFI_STA_IP6 arguments: AUTO, or <addr>/<prefix>[,<gateway>[,<dns1>[,<dns2>]]].
 * Empty gateway and DNS arguments mean none. StrView needs data() and size() (std::string_view).
 * @retval nullptr on success (out is set), else an error message (out is unchanged).
 */
template <typename StrView>
const char* ParseIPv6ConfigArgs(const StrView args[], uint16_t num_args, IPv6Config& out) {
    static const char* const kUsage = "Use AUTO or <addr>/<prefix>[,<gateway>[,<dns1>[,<dns2>]]].";
    IPv6Config c;
    if (num_args == 1 && args[0].size() == 4) {
        const char* s = args[0].data();
        const char* kAuto = "AUTO";
        bool match = true;
        for (int i = 0; i < 4; i++) match &= (s[i] >= 'a' && s[i] <= 'z' ? s[i] - 'a' + 'A' : s[i]) == kAuto[i];
        if (match) {
            out = c;
            return nullptr;
        }
    }
    if (num_args < 1 || num_args > 2 + IPv6Config::kNumDNSServers || args[0].size() == 0) return kUsage;
    c.is_static = true;
    const char* a = args[0].data();
    size_t a_len = args[0].size();
    size_t slash = 0;
    while (slash < a_len && a[slash] != '/') slash++;
    if (slash == a_len) return "Give the prefix length after the address, e.g. 2001:db8::50/64.";
    if (!ParseIPv6Addr(a, slash, c.ip)) {
        return "IPv6 address is not valid, e.g. 2001:db8::50/64.";
    }
    size_t num_digits = a_len - slash - 1;
    if (num_digits == 0 || num_digits > 3) return "Prefix length must be from 1 to 128, usually 64.";
    int prefix_len = 0;
    for (size_t i = slash + 1; i < a_len; i++) {
        if (a[i] < '0' || a[i] > '9') return "Prefix length must be from 1 to 128, usually 64.";
        prefix_len = prefix_len * 10 + (a[i] - '0');
    }
    if (prefix_len < IPv6Config::kMinPrefixLen || prefix_len > IPv6Config::kMaxPrefixLen) {
        return "Prefix length must be from 1 to 128, usually 64.";
    }
    c.prefix_len = prefix_len;
    if (num_args > 1 && args[1].size() > 0 && !ParseIPv6Addr(args[1].data(), args[1].size(), c.gateway)) {
        return "Gateway must be an IPv6 address, or empty for none.";
    }
    for (uint16_t i = 0; i < IPv6Config::kNumDNSServers; i++) {
        uint16_t arg = 2 + i;
        if (num_args > arg && args[arg].size() > 0 && !ParseIPv6Addr(args[arg].data(), args[arg].size(), c.dns[i])) {
            return "DNS server must be an IPv6 address, or empty for none.";
        }
    }
    const char* error = ValidateIPv6Config(c);
    if (error != nullptr) return error;
    out = c;
    return nullptr;
}

#endif /* IPV6_CONFIG_HH_ */

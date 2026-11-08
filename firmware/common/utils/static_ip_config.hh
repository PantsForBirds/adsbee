#ifndef STATIC_IP_CONFIG_HH_
#define STATIC_IP_CONFIG_HH_

#include "ipv4_config.hh"
#include "ipv6_config.hh"

// AT+ETHERNET_IP / AT+WIFI_STA_IP: the address family first, then that family's arguments.
enum class IPFamily : uint8_t { kIPv4, kIPv6 };

// Matches IPV4 or IPV6, case-insensitively. StrView needs data() and size().
template <typename StrView>
bool ParseIPFamily(const StrView& arg, IPFamily& family) {
    if (arg.size() != 4) return false;
    const char* s = arg.data();
    char up[4];
    for (int i = 0; i < 4; i++) up[i] = s[i] >= 'a' && s[i] <= 'z' ? s[i] - 'a' + 'A' : s[i];
    if (up[0] != 'I' || up[1] != 'P' || up[2] != 'V') return false;
    if (up[3] == '4') {
        family = IPFamily::kIPv4;
        return true;
    }
    if (up[3] == '6') {
        family = IPFamily::kIPv6;
        return true;
    }
    return false;
}

/**
 * Parses AT+ETHERNET_IP / AT+WIFI_STA_IP arguments: IPV4,<IPv4 args> or IPV6,<IPv6 args>. Writes only the named family's
 * configuration.
 * @retval nullptr on success (family and that configuration are set), else an error message (nothing is changed).
 */
template <typename StrView>
const char* ParseStaticIPArgs(const StrView args[], uint16_t num_args, IPv4Config& ipv4, IPv6Config& ipv6,
                              IPFamily& family) {
    IPFamily f;
    if (num_args < 1 || !ParseIPFamily(args[0], f)) {
        return "First argument must be IPV4 or IPV6. Use IPV4,DHCP | IPV4,<ip>,<netmask>[,<gateway>[,<dns1>[,<dns2>]]]"
               " | IPV6,AUTO | IPV6,<addr>/<prefix>[,<gateway>[,<dns1>[,<dns2>]]].";
    }
    const char* error = f == IPFamily::kIPv4 ? ParseIPv4ConfigArgs(args + 1, num_args - 1, ipv4)
                                             : ParseIPv6ConfigArgs(args + 1, num_args - 1, ipv6);
    if (error == nullptr) family = f;
    return error;
}

#endif /* STATIC_IP_CONFIG_HH_ */

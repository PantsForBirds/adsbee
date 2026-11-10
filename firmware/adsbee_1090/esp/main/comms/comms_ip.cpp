#include "beast/beast_utils.hh"  // For beast reporting.
#include "comms.hh"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "hardware_capabilities.hh"  // Internal-RAM heap guards (PSRAM must not mask an internal shortage).
#include "esp_mac.h"
#include "esp_netif_net_stack.h"  // esp_netif_get_netif_impl
#include "crc.hh"
#include "feed_datagram.hh"
#include "hal.hh"
#include "heap_diagnostics.hh"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/nd6.h"
#include "lwip/netif.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mdns.h"
#include "task_priorities.hh"

static const uint32_t kTCPSocketReconnectIntervalMs = 10000;
// How long a non-blocking feed connect may stay pending before it is abandoned (see ConnectFeedSocket()).
static const uint32_t kTCPSocketConnectTimeoutMs = 5000;

// Heap thresholds for back-pressure: while free memory is below them, feed sockets are given no new data (reports are
// dropped) so that sends can't run the heap out.
// Set high enough to leave room for WebSocket allocations (~2KB) and other system needs.
static const uint32_t kHeapBackPressureThresholdBytes = 20480;
// DMA-capable memory threshold. WiFi TX buffers require internal DMA memory which is more constrained. When WiFi is
// enabled we must reserve enough DMA-capable SRAM for esp_wifi's TX path; but on an Ethernet-only unit esp_wifi_init()
// is never called, so those buffers are never allocated and reserving 16 KB for them is a false back-pressure trip
// (it starved the WAN queue on the PSRAM-less S3 running Bluetooth Remote ID). Use the lower Ethernet-only threshold
// when both WiFi AP and STA are disabled.
static const uint32_t kDMABackPressureThresholdBytes = 16384;
static const uint32_t kDMABackPressureThresholdEthernetOnlyBytes = 10240;

// Unsent bytes kept per feed when its socket is full, allocated only while a feed is behind. With PSRAM this rides
// out a TCP retransmission timeout at about 900 messages per second; without PSRAM it holds the tail of a cut batch
// and one more batch, and is only allocated while the heap has room to spare.
static const uint16_t kFeedBacklogBytesPSRAM = 32768;
static const uint16_t kFeedBacklogBytesInternal = 4096;
static const uint32_t kFeedBacklogMinInternalFreeBytes = 32768;
// A burst can fill a healthy feed's socket and backlog for a moment. IPWANTask then waits for room instead of dropping,
// but only out of a small per-feed budget that refills while the feed is caught up, so a feed that stays behind delays
// the others by at most the budget once, and by at most 1 / kFeedWaitBudgetRefillDivisor of the time after that.
static const uint16_t kFeedWaitBudgetMs = 100;
static const uint32_t kFeedWaitBudgetRefillDivisor = 10;
// A feed that takes no data for this long is closed and reconnected.
static const uint32_t kFeedStallTimeoutMs = 10000;
static const uint32_t kFeedDropLogIntervalMs = 10000;
// Queue wait of IPWANTask while a feed backlog is waiting to be flushed.
static const uint32_t kFeedBacklogFlushIntervalMs = 5;
// Bytes a server sends back on a Beast feed are read in chunks of this size, a bounded number per pass.
static const uint16_t kFeedRecvChunkBytes = 64;
static const uint16_t kFeedRecvMaxChunksPerPass = 16;
// While the capabilities answer can still arrive, a feed's socket is read on every pass. After that it is only drained
// (pings, a late answer) at this interval: a recv() per feed per pass cost CPU time the 5-feed throughput needs.
static const uint32_t kFeedRecvDrainIntervalMs = 1000;

// #define ENABLE_TCP_SOCKET_RATE_LIMITING
#ifdef ENABLE_TCP_SOCKET_RATE_LIMITING
static const uint32_t kSendBufRateLimitBytesPerSecond = 100000;  // 100 kB/s
static const uint32_t kSendBufRateLimitIntervalMs = 10;
static const uint32_t kSendBufRateLimitBytesPerInterval =
    kSendBufRateLimitBytesPerSecond / (1000 / kSendBufRateLimitIntervalMs);
static const uint32_t kRateLimitDelayDurationMs = 1;
#endif

static const uint32_t kTCPKeepAliveEnable = 1;
static const uint32_t kTCPKeepAliveIdleSecondsBeforeStartingProbe = 120;
static const uint32_t kTCPKeepAliveIntervalSecondsBetweenProbes = 30;
static const uint32_t kTCPKeepAliveMaxFailedProbesBeforeDisconnect = 3;
static const uint32_t kTCPReuseAddrEnable =
    1;  // Allow reuse of local addresses and sockets that are in the TIME_WAIT state.

/** "Pass-Through" functions used to access member functions in callbacks. **/
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.IPEventHandler(arg, event_base, event_id, event_data);
}
void ip6_link_up_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    comms_manager.IP6LinkUpEventHandler(arg, event_base, event_id, event_data);
}
void ip_wan_task(void* pvParameters) { comms_manager.IPWANTask(pvParameters); }
/** End "Pass-Through" functions. **/

// Resolves a feed URI (hostname, IPv4 or IPv6 literal) to an address of the given family (AF_INET, AF_INET6 or
// AF_UNSPEC) and socket type. The caller frees *res with freeaddrinfo().
static bool ResolveFeedURI(const char* uri, uint16_t port, int family, int socktype, struct addrinfo** res) {
    struct addrinfo hints = {};
    hints.ai_family = family;
    hints.ai_socktype = socktype;
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%u", port);

    *res = nullptr;
    int err = getaddrinfo(uri, port_str, &hints, res);
    if (err != 0 || *res == nullptr) {
        CONSOLE_ERROR("ResolveFeedURI", "Lookup of %s (%s) failed: %d", uri,
                      family == AF_INET ? "IPv4" : (family == AF_INET6 ? "IPv6" : "IPv4/IPv6"), err);
        if (*res != nullptr) {
            freeaddrinfo(*res);
            *res = nullptr;
        }
        return false;
    }
    return true;
}

// FeedSendBuffer write callback: hands bytes to a feed's TCP socket without waiting. ctx is the socket.
static int FeedSocketWrite(void* ctx, const uint8_t* buf, uint16_t buf_len) {
    int num_sent = send(static_cast<int>(reinterpret_cast<intptr_t>(ctx)), buf, buf_len, MSG_DONTWAIT);
    if (num_sent >= 0) {
        return num_sent;
    }
    // Socket send buffer full (the server or the network is slow), an earlier write still being queued, or lwIP briefly
    // out of buffers: try again later.
    return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS || errno == ENOBUFS || errno == ENOMEM)
               ? 0
               : -1;
}

static void* FeedBacklogAlloc(size_t num_bytes) {
    if (HardwareCapabilities::HasPSRAM()) {
        return heap_caps_malloc(num_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    // Internal RAM is shared with WiFi, lwIP and the web server: a feed without a backlog drops whole batches.
    if (HardwareCapabilities::GetInternalFreeBytes() < kFeedBacklogMinInternalFreeBytes) {
        return nullptr;
    }
    return heap_caps_malloc(num_bytes, MALLOC_CAP_8BIT);
}

bool CommsManager::IPInit() {
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));
    ip_event_handler_was_initialized_ = true;

    // raw_packets_buf is allocated on the heap instead of the stack, which keeps this task's stack small.
    xTaskCreate(ip_wan_task, "ip_wan_task", kIPWANTaskStackSizeBytes, &ip_wan_task_handle, kIPWANTaskPriority, NULL);

    // Initialize mDNS service.
    esp_err_t err = mdns_init();
    if (err) {
        CONSOLE_ERROR("CommsManager::IPInit", "MDNS Init failed: %d\n", err);
        return false;
    }

    // Set hostname.
    err = mdns_hostname_set(settings_manager.settings.core_network_settings.hostname);
    if (err) {
        CONSOLE_ERROR("CommsManager::IPInit", "Failed setting MDNS hostname to %s: %d\n",
                      settings_manager.settings.core_network_settings.hostname, err);
        return false;
    }
    // Set default instance.
    mdns_instance_name_set("ADSBee 1090");
    HeapDiagnostics::Mark("ip_mdns");

    CONSOLE_INFO("CommsManager::IPInit", "MDNS initialized with hostname %s.\n",
                 settings_manager.settings.core_network_settings.hostname);
    return true;
}

// Static IPv6 routing, read by the lwIP hooks below. Written only in the TCP/IP task (see ApplyStaticIPv6()).
struct StaticIP6Route {
    struct netif* netif = nullptr;  // nullptr: slot unused.
    ip6_addr_t ip = {};
    uint8_t prefix_len = 0;
    bool has_gateway = false;
    ip6_addr_t gateway = {};
};
static StaticIP6Route static_ip6_routes[2];  // Ethernet, WiFi station.

static bool StaticIP6RouteIsUp(const StaticIP6Route& r) {
    return r.netif != nullptr && netif_is_up(r.netif) && netif_is_link_up(r.netif);
}

static bool IP6OnPrefix(const ip6_addr_t* addr, const StaticIP6Route& r) {
    return IPv6PrefixMatch(reinterpret_cast<const uint8_t*>(addr->addr), reinterpret_cast<const uint8_t*>(r.ip.addr),
                           r.prefix_len);
}

// lwIP picks an interface for an unscoped destination (CONFIG_LWIP_HOOK_IP6_ROUTE_CUSTOM). Called before lwIP's own
// rules. Returns nullptr to leave the choice to lwIP.
extern "C" struct netif* lwip_hook_ip6_route(const ip6_addr_t* src, const ip6_addr_t* dest) {
    for (const StaticIP6Route& r : static_ip6_routes) {
        if (!StaticIP6RouteIsUp(r)) continue;
        // Replies from the static address, and destinations on its prefix (lwIP only knows /64), go out its interface.
        if ((src != nullptr && ip6_addr_eq(src, &r.ip)) || IP6OnPrefix(dest, r)) return r.netif;
    }
    // Destinations lwIP can route itself: on an interface's /64, or behind a router learned from advertisements.
    struct netif* netif;
    NETIF_FOREACH(netif) {
        if (!netif_is_up(netif) || !netif_is_link_up(netif)) continue;
        for (int i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
            if (ip6_addr_isvalid(netif_ip6_addr_state(netif, i)) && ip6_addr_net_eq(dest, netif_ip6_addr(netif, i)) &&
                (netif_ip6_addr_isstatic(netif, i) || ip6_addr_nethost_eq(dest, netif_ip6_addr(netif, i)))) {
                return nullptr;
            }
        }
    }
    if (nd6_find_route(dest) != nullptr) return nullptr;
    // Else the first interface with a static gateway, Ethernet before the WiFi station.
    for (const StaticIP6Route& r : static_ip6_routes) {
        if (StaticIP6RouteIsUp(r) && r.has_gateway) return r.netif;
    }
    return nullptr;
}

// lwIP asks for the next hop of an off-link destination before it picks a router from advertisements
// (CONFIG_LWIP_HOOK_ND6_GET_GW_CUSTOM). Returns nullptr to use those routers.
extern "C" const ip6_addr_t* lwip_hook_nd6_get_gw(struct netif* netif, const ip6_addr_t* dest) {
    for (const StaticIP6Route& r : static_ip6_routes) {
        if (r.netif != netif) continue;
        if (IP6OnPrefix(dest, r)) return dest;  // On-link: a prefix shorter than lwIP's /64.
        if (r.has_gateway) return &r.gateway;
    }
    return nullptr;
}

struct StaticIP6RouteUpdate {
    int slot;
    StaticIP6Route route;
};

static esp_err_t SetStaticIP6Route(void* ctx) {
    const StaticIP6RouteUpdate* update = static_cast<const StaticIP6RouteUpdate*>(ctx);
    static_ip6_routes[update->slot] = update->route;
    // The address stays static: SLAAC never gives it a lifetime, and lwIP takes its /64 as on-link.
    struct netif* netif = update->route.netif;
    for (int i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
        if (ip6_addr_eq(netif_ip6_addr(netif, i), &update->route.ip)) {
            netif_ip6_addr_set_valid_life(netif, i, IP6_ADDR_LIFE_STATIC);
            netif_ip6_addr_set_pref_life(netif, i, IP6_ADDR_LIFE_STATIC);
        }
    }
    // Next-hop decisions cached for the old configuration are stale.
    nd6_clear_destination_cache();
    return ESP_OK;
}

void CommsManager::ApplyIPModeToNetifConfig(esp_netif_inherent_config_t& base, const IPv6Config& ipv6) {
    uint32_t flags = base.flags;
    if (ip_mode == SettingsManager::kIPModeIPv6) {
        flags &= ~ESP_NETIF_DHCP_CLIENT;
    } else {
        flags |= ESP_NETIF_DHCP_CLIENT;
    }
    if (ip_mode == SettingsManager::kIPModeIPv4 || ipv6.is_static) {
        // A static IPv6 address replaces SLAAC: no addresses from router advertisements (routers are still learned).
        flags &= ~ESP_NETIF_FLAG_IPV6_AUTOCONFIG_ENABLED;
    } else {
        flags |= ESP_NETIF_FLAG_IPV6_AUTOCONFIG_ENABLED;
    }
    base.flags = static_cast<esp_netif_flags_t>(flags);
}

void CommsManager::ApplyStaticIPv4(esp_netif_t* netif, const IPv4Config& config) {
    if (netif == nullptr || !config.is_static || ip_mode == SettingsManager::kIPModeIPv6) {
        return;
    }
    const char* desc = esp_netif_get_desc(netif);
    esp_err_t err = esp_netif_dhcpc_stop(netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        CONSOLE_ERROR("CommsManager::ApplyStaticIPv4", "Failed to stop the DHCP client on %s: %s", desc,
                      esp_err_to_name(err));
        return;
    }

    // Setting the address clears the DNS servers, which all interfaces share: put them back, then the static ones.
    esp_netif_dns_type_t dns_types[] = {ESP_NETIF_DNS_MAIN, ESP_NETIF_DNS_BACKUP, ESP_NETIF_DNS_FALLBACK};
    esp_netif_dns_info_t old_dns[3] = {};
    for (uint16_t i = 0; i < 3; i++) {
        esp_netif_get_dns_info(netif, dns_types[i], &old_dns[i]);
    }

    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr = ESP_IP4TOADDR(config.ip[0], config.ip[1], config.ip[2], config.ip[3]);
    ip_info.netmask.addr = ESP_IP4TOADDR(config.netmask[0], config.netmask[1], config.netmask[2], config.netmask[3]);
    ip_info.gw.addr = ESP_IP4TOADDR(config.gateway[0], config.gateway[1], config.gateway[2], config.gateway[3]);
    err = esp_netif_set_ip_info(netif, &ip_info);
    if (err != ESP_OK) {
        CONSOLE_ERROR("CommsManager::ApplyStaticIPv4", "Failed to set the static IPv4 address on %s: %s", desc,
                      esp_err_to_name(err));
        return;
    }

    for (uint16_t i = 0; i < 3; i++) {
        if (!ESP_IP_IS_ANY(old_dns[i].ip)) {
            esp_netif_set_dns_info(netif, dns_types[i], &old_dns[i]);
        }
    }
    ApplyStaticDNS(netif);
    CONSOLE_INFO("CommsManager::ApplyStaticIPv4", "%s uses static IPv4 address " IPSTR ".", desc,
                 IP2STR(&ip_info.ip));
}

void CommsManager::ApplyStaticIPv6(esp_netif_t* netif, const IPv6Config& config) {
    if (netif == nullptr || !config.is_static || ip_mode == SettingsManager::kIPModeIPv4) {
        return;
    }
    StaticIP6RouteUpdate update = {.slot = netif == ethernet_netif_ ? 0 : 1, .route = {}};
    const char* desc = esp_netif_get_desc(netif);
    esp_ip6_addr_t addr = {};
    memcpy(addr.addr, config.ip, sizeof(addr.addr));
    // Preferred at once, without duplicate address detection: lwIP only runs DAD for addresses it forms itself.
    esp_err_t err = esp_netif_add_ip6_address(netif, addr, true);
    if (err != ESP_OK) {
        CONSOLE_ERROR("CommsManager::ApplyStaticIPv6", "Failed to add the static IPv6 address on %s: %s", desc,
                      esp_err_to_name(err));
        return;
    }

    StaticIP6Route& r = update.route;
    r.netif = static_cast<struct netif*>(esp_netif_get_netif_impl(netif));
    memcpy(r.ip.addr, config.ip, sizeof(r.ip.addr));
    ip6_addr_assign_zone(&r.ip, IP6_UNICAST, r.netif);
    r.prefix_len = config.prefix_len;
    r.has_gateway = !IPv6IsZero(config.gateway);
    memcpy(r.gateway.addr, config.gateway, sizeof(r.gateway.addr));
    ip6_addr_assign_zone(&r.gateway, IP6_UNICAST, r.netif);
    err = esp_netif_tcpip_exec(SetStaticIP6Route, &update);
    if (err != ESP_OK) {
        CONSOLE_ERROR("CommsManager::ApplyStaticIPv6", "Failed to set the static IPv6 route on %s: %s", desc,
                      esp_err_to_name(err));
    }
    ApplyStaticDNS(netif);

    char addr_str[IPv6Config::kAddrStrMaxLen + 1], gw_str[IPv6Config::kAddrStrMaxLen + 1];
    FormatIPv6Addr(config.ip, addr_str, sizeof(addr_str));
    FormatIPv6Addr(config.gateway, gw_str, sizeof(gw_str));
    CONSOLE_INFO("CommsManager::ApplyStaticIPv6", "%s uses static IPv6 address %s/%u, gateway %s.", desc, addr_str,
                 config.prefix_len, r.has_gateway ? gw_str : "from router advertisements");
}

void CommsManager::ApplyStaticDNS(esp_netif_t* netif) {
    if (netif == nullptr) {
        return;
    }
    const bool is_ethernet = netif == ethernet_netif_;
    const IPv4Config& v4 = is_ethernet ? ethernet_ipv4 : wifi_sta_ipv4;
    const IPv6Config& v6 = is_ethernet ? ethernet_ipv6 : wifi_sta_ipv6;
    esp_netif_dns_info_t servers[3] = {};
    uint16_t num_servers = 0;
    if (ip_mode != SettingsManager::kIPModeIPv6 && v4.is_static) {
        for (uint16_t i = 0; i < IPv4Config::kNumDNSServers && num_servers < 3; i++) {
            const uint8_t* a = v4.dns[i];
            if (IPv4IsZero(a)) continue;
            servers[num_servers].ip.type = ESP_IPADDR_TYPE_V4;
            servers[num_servers++].ip.u_addr.ip4.addr = ESP_IP4TOADDR(a[0], a[1], a[2], a[3]);
        }
    }
    if (ip_mode != SettingsManager::kIPModeIPv4 && v6.is_static) {
        for (uint16_t i = 0; i < IPv6Config::kNumDNSServers && num_servers < 3; i++) {
            if (IPv6IsZero(v6.dns[i])) continue;
            esp_netif_dns_info_t& dns = servers[num_servers++];
            dns.ip.type = ESP_IPADDR_TYPE_V6;
            memcpy(dns.ip.u_addr.ip6.addr, v6.dns[i], sizeof(dns.ip.u_addr.ip6.addr));
            // A link-local server is reached through this interface.
            dns.ip.u_addr.ip6.zone = IPv6IsLinkLocal(v6.dns[i]) ? esp_netif_get_netif_impl_index(netif) : 0;
        }
    }
    esp_netif_dns_type_t types[] = {ESP_NETIF_DNS_MAIN, ESP_NETIF_DNS_BACKUP, ESP_NETIF_DNS_FALLBACK};
    for (uint16_t i = 0; i < num_servers; i++) {
        esp_err_t err = esp_netif_set_dns_info(netif, types[i], &servers[i]);
        if (err != ESP_OK) {
            CONSOLE_ERROR("CommsManager::ApplyStaticDNS", "Failed to set DNS server %u on %s: %s", i + 1,
                          esp_netif_get_desc(netif), esp_err_to_name(err));
        }
    }
}

void CommsManager::IP6LinkUpEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id,
                                         void* event_data) {
    if (ip_mode == SettingsManager::kIPModeIPv4) {
        return;
    }
    // Registered after the default handlers, which bring the netif up on these events.
    esp_netif_t* netif = nullptr;
    if (event_base == ETH_EVENT && event_id == ETHERNET_EVENT_CONNECTED) {
        netif = ethernet_netif_;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        netif = wifi_sta_netif_;
    }
    if (netif == nullptr) {
        return;
    }
    esp_err_t err = esp_netif_create_ip6_linklocal(netif);
    if (err != ESP_OK) {
        CONSOLE_ERROR("CommsManager::IP6LinkUpEventHandler", "Failed to create IPv6 link-local address on %s: %s",
                      esp_netif_get_desc(netif), esp_err_to_name(err));
    }
    ApplyStaticIPv6(netif, netif == ethernet_netif_ ? ethernet_ipv6 : wifi_sta_ipv6);
}

void CommsManager::GetIP6AddrStrs(esp_netif_t* netif, char* link_local, char* global, size_t len) {
    link_local[0] = '\0';
    global[0] = '\0';
    if (netif == nullptr) {
        return;
    }
    esp_ip6_addr_t addrs[LWIP_IPV6_NUM_ADDRESSES];
    int num_addrs = esp_netif_get_all_ip6(netif, addrs);  // Preferred (usable) addresses only.
    for (int i = 0; i < num_addrs; i++) {
        esp_ip6_addr_type_t type = esp_netif_ip6_get_addr_type(&addrs[i]);
        char* dest = nullptr;
        if (type == ESP_IP6_ADDR_IS_LINK_LOCAL && link_local[0] == '\0') {
            dest = link_local;
        } else if ((type == ESP_IP6_ADDR_IS_GLOBAL || type == ESP_IP6_ADDR_IS_UNIQUE_LOCAL) && global[0] == '\0') {
            dest = global;
        }
        if (dest != nullptr) {
            inet_ntop(AF_INET6, addrs[i].addr, dest, len);
        }
    }
}

void CommsManager::IPEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    switch (event_id) {
        case IP_EVENT_AP_STAIPASSIGNED: {
            // A new station has connected to the ADSBee's softAP network.
            ip_event_ap_staipassigned_t* event = (ip_event_ap_staipassigned_t*)event_data;

            char client_mac_str[SettingsManager::Settings::kMACAddrStrLen + 1] = {0};
            char client_ip_str[SettingsManager::Settings::kIPAddrStrLen + 1] = {0};
            snprintf(client_mac_str, SettingsManager::Settings::kMACAddrStrLen, MACSTR, MAC2STR(event->mac));
            client_mac_str[SettingsManager::Settings::kMACAddrStrLen] = '\0';
            snprintf(client_ip_str, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&event->ip));
            client_ip_str[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler",
                         "WiFi Access Point assigned IP address to client. IP: %s, MAC: %s", client_ip_str,
                         client_mac_str);

            WiFiAddClient(event->ip, event->mac);
            break;
        }
        case IP_EVENT_STA_GOT_IP: {
            // The ADSBee has connected to an external network as a WiFi station.
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            const esp_netif_ip_info_t* ip_info = &event->ip_info;

            wifi_sta_has_ip_ = true;
            wifi_sta_failed_mask_ = 0;  // Connected: earlier failed attempts no longer matter.
            snprintf(wifi_sta_ip, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->ip));
            wifi_sta_ip[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(wifi_sta_netmask, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->netmask));
            wifi_sta_netmask[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(wifi_sta_gateway, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->gw));
            wifi_sta_gateway[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler",
                         "WiFi Station got IP Address. IP: %s, Netmask: %s, Gateway: %s", wifi_sta_ip, wifi_sta_netmask,
                         wifi_sta_gateway);
            ApplyStaticDNS(wifi_sta_netif_);  // DHCP may have replaced the static IPv6 DNS servers.
            break;
        }
        case IP_EVENT_ETH_GOT_IP: {
            // The ADSBee's Ethernet interface has connected to an external network.
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            const esp_netif_ip_info_t* ip_info = &event->ip_info;

            ethernet_has_ip_ = true;
            snprintf(ethernet_ip, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->ip));
            ethernet_ip[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(ethernet_netmask, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->netmask));
            ethernet_netmask[SettingsManager::Settings::kIPAddrStrLen] = '\0';
            snprintf(ethernet_gateway, SettingsManager::Settings::kIPAddrStrLen, IPSTR, IP2STR(&ip_info->gw));
            ethernet_gateway[SettingsManager::Settings::kIPAddrStrLen] = '\0';

            CONSOLE_INFO("CommsManager::IPEventHandler", "Ethernet got IP address. IP: %s, Netmask: %s, Gateway: %s",
                         ethernet_ip, ethernet_netmask, ethernet_gateway);
            ApplyStaticDNS(ethernet_netif_);  // DHCP may have replaced the static IPv6 DNS servers.
            break;
        }
        case IP_EVENT_GOT_IP6: {
            // An IPv6 address finished duplicate address detection (link-local, or SLAAC from a router advertisement).
            ip_event_got_ip6_t* event = (ip_event_got_ip6_t*)event_data;
            esp_ip6_addr_type_t type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
            bool routable = type == ESP_IP6_ADDR_IS_GLOBAL || type == ESP_IP6_ADDR_IS_UNIQUE_LOCAL;
            if (routable && event->esp_netif == ethernet_netif_) {
                ethernet_has_ip6_ = true;
            } else if (routable && event->esp_netif == wifi_sta_netif_) {
                wifi_sta_has_ip6_ = true;
            }
            char addr_str[ObjectDictionary::kIP6AddrStrLen + 1];
            inet_ntop(AF_INET6, event->ip6_info.ip.addr, addr_str, sizeof(addr_str));
            CONSOLE_INFO("CommsManager::IPEventHandler", "%s got IPv6 address %s (%s).",
                         esp_netif_get_desc(event->esp_netif), addr_str, routable ? "global" : "link-local");
            break;
        }
        case IP_EVENT_ETH_LOST_IP: {
            // The ADSBee's Ethernet interface has disconnected from an external network.
            ethernet_has_ip_ = false;
            CONSOLE_INFO("CommsManager::IPEventHandler", "Ethernet lost IP address.");
            break;
        }
    }
}

void CommsManager::IPWANTask(void* pvParameters) {
    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task started.");

    uint8_t* raw_packets_buf = (uint8_t*)heap_caps_malloc(CompositeArray::RawPackets::kMaxLenBytes, MALLOC_CAP_8BIT);
    if (raw_packets_buf == nullptr) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to allocate raw packets buffer on heap. Task exiting.");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        // Don't try establishing socket connections until the ESP32 has been assigned an IP address.
        while (!HasIP()) {
            vTaskDelay(1);  // Delay for 1 tick.
        }

        UpdateFeedMetrics();
        UpdateFeedSendBuffers();
        UpdateMQTTClient();
        PublishMQTTMetrics();
        LogWANQueueOverflows();

        uint16_t num_active_report_sinks = 0;
        ReportSinkInfo active_report_sinks[SettingsManager::Settings::kMaxNumFeeds];
        // Whether RemoteIDManager queues packets for the feeds; ReportBeast sends them to the feeds that agreed on it.
        bool feeds_take_remote_id = false;

        // Maintain socket connections and build list of acive report sinks.
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            // Iterate through feeds, open/close and send message as required. The feeds_enabled master switch gates
            // all outbound feed connections: when it's off no feed sockets are opened, regardless of the per-feed
            // feed_is_active[] flags.
            if (!settings_manager.settings.feeds_enabled || !settings_manager.settings.feed_is_active[i]) {
                // Feed is not active, ensure socket is closed.
                if (feed_sock_is_connected_[i] || feed_sock_is_connecting_[i]) {
                    // Need to close the socket connection.
                    CloseFeedSocket(i);
                }
                continue;
            } else {
                // Feed is active, ensure socket is open and add to active sinks if connected.
                uint32_t config_crc = FeedConfigCRC(i);
                if (feed_sock_is_connected_[i] || feed_sock_is_connecting_[i]) {
                    if (config_crc != feed_config_crcs_[i]) {
                        CONSOLE_INFO("CommsManager::IPWANTask", "Settings of feed %d changed, reconnecting.", i);
                        CloseFeedSocket(i);
                        feed_sock_last_connect_timestamp_ms_[i] = 0;  // Reconnect without waiting.
                    }
                }
                if (!feed_sock_is_connected_[i] && !feed_sock_is_connecting_[i]) {
                    feed_config_crcs_[i] = config_crc;
                    feed_transports_in_use_[i] = settings_manager.settings.feed_transports[i];
                }
                // An MQTT feed follows the state of the shared client.
                bool is_mqtt = feed_transports_in_use_[i] == SettingsManager::kFeedTransportMQTT;
                if ((!feed_sock_is_connected_[i] || is_mqtt) && !ConnectFeedSocket(i)) {
                    // Need to open the socket connection, but failed to do so.
                    continue;  // Failed to connect, try again later.
                }
                // Before this pass's frames: a capabilities answer takes effect from them on.
                ReceiveFeedSocket(i);
                // Socket is connected, add to active sinks.
                ReportSinkInfo& sink = active_report_sinks[num_active_report_sinks++];
                sink = {i, settings_manager.settings.feed_protocols[i], feed_caps_[i].bits()};
                feeds_take_remote_id |= (sink.beast_caps & BeastReporter::kBeastCapRemoteID) != 0;
            }
        }
        feeds_take_remote_id_.store(feeds_take_remote_id, std::memory_order_relaxed);

        // Gather packet(s) to send.
        /**
         * Architecture note: We send packets in the queue as a buffer, since the buffer in the other task gets
         * deallocated. Here, we can unpack the buffer into a CompositeArray object with pointers because we know it
         * won't go out of scope till we are done with it.
         */
        CompositeArray::RawPackets reporting_composite_array;
        // Wake up sooner while a backlog waits for its socket.
        bool backlog_pending = false;
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            backlog_pending |= feed_sock_is_connected_[i] && feed_send_buffers_[i].BacklogLenBytes() > 0;
        }
        TickType_t queue_wait_ticks =
            backlog_pending ? pdMS_TO_TICKS(kFeedBacklogFlushIntervalMs) : kWiFiSTATaskUpdateIntervalTicks;
        if (xQueueReceive(ip_wan_reporting_composite_array_queue_, raw_packets_buf, queue_wait_ticks) != pdTRUE) {
            // No packets: still drive the locally decoded protocols, which send the heartbeats.
            reporting_composite_array = CompositeArray::PackRawPacketsBuffer(
                raw_packets_buf, sizeof(CompositeArray::RawPackets::Header), nullptr, nullptr, nullptr);
        } else if (!CompositeArray::UnpackRawPacketsBuffer(reporting_composite_array, raw_packets_buf,
                                                           CompositeArray::RawPackets::kMaxLenBytes)) {
            CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to unpack CompositeArray from buffer.");
            continue;
        }

        // Update feeds with raw and digested reports.
        if (!UpdateReporting(active_report_sinks, num_active_report_sinks, &reporting_composite_array)) {
            CONSOLE_ERROR("CommsManager::IPWANTask", "Error during UpdateReporting for feeds.");
        }
    }

    // Close all sockets while exiting.
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        CloseFeedSocket(i);
    }

    // Free heap buffer.
    heap_caps_free(raw_packets_buf);

    CONSOLE_INFO("CommsManager::IPWANTask", "IP WAN Task exiting.");
}

uint32_t CommsManager::FeedConfigCRC(uint16_t feed_index) {
    const SettingsManager::Settings& s = settings_manager.settings;
    uint32_t crc = crc32_ieee_802_3(reinterpret_cast<const uint8_t*>(s.feed_uris[feed_index]),
                                    sizeof(s.feed_uris[feed_index]));
    crc = crc32_ieee_802_3(reinterpret_cast<const uint8_t*>(&s.feed_ports[feed_index]), sizeof(s.feed_ports[0]), crc);
    crc = crc32_ieee_802_3(reinterpret_cast<const uint8_t*>(&s.feed_protocols[feed_index]), sizeof(s.feed_protocols[0]),
                           crc);
    crc = crc32_ieee_802_3(reinterpret_cast<const uint8_t*>(&s.feed_transports[feed_index]),
                           sizeof(s.feed_transports[0]), crc);
    return crc32_ieee_802_3(s.feed_receiver_ids[feed_index], sizeof(s.feed_receiver_ids[0]), crc);
}

void CommsManager::CloseFeedSocket(uint16_t feed_index) {
    // MQTT feeds share the client, which UpdateMQTTClient() stops when no MQTT feed is left.
    if (feed_transports_in_use_[feed_index] != SettingsManager::kFeedTransportMQTT) {
        close(feed_sock_[feed_index]);
    }
    feed_send_buffers_[feed_index].Close();
    feed_caps_[feed_index].Reset();
    feed_rx_open_[feed_index] = false;
    feed_sock_is_connected_[feed_index] = false;
    feed_sock_is_connecting_[feed_index] = false;
    CONSOLE_INFO("CommsManager::IPWANTask", "Closed socket for feed %d.", feed_index);
}

bool CommsManager::ConnectFeedSocket(uint16_t feed_index) {
    switch (feed_transports_in_use_[feed_index]) {
        case SettingsManager::kFeedTransportUDP:
            return ConnectFeedUDP(feed_index);
        case SettingsManager::kFeedTransportMQTT:
            return ConnectFeedMQTT(feed_index);
        default:
            break;
    }
    if (feed_sock_is_connecting_[feed_index]) {
        return PollFeedSocketConnect(feed_index);
    }
    // Meter reconnect attempt interval.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - feed_sock_last_connect_timestamp_ms_[feed_index] <= kTCPSocketReconnectIntervalMs) {
        return false;
    }
    feed_sock_last_connect_timestamp_ms_[feed_index] = timestamp_ms;

    // Resolve the destination for the address families this device can reach. A hostname with both A and AAAA records
    // resolves to IPv4 when IPv4 is up (lwIP returns a single address).
    int family = AF_UNSPEC;
    if (!HasIPv4()) {
        family = AF_INET6;
    } else if (!HasIPv6()) {
        family = AF_INET;
    }
    struct addrinfo* dest = nullptr;
    if (!ResolveFeedURI(settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index],
                        family, SOCK_STREAM, &dest)) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Failed to resolve URI %s for feed %d",
                      settings_manager.settings.feed_uris[feed_index], feed_index);
        return false;
    }
    struct sockaddr_storage dest_addr = {};
    socklen_t dest_addr_len = MIN(dest->ai_addrlen, sizeof(dest_addr));
    memcpy(&dest_addr, dest->ai_addr, dest_addr_len);
    freeaddrinfo(dest);

    // Create socket.
    feed_sock_[feed_index] = socket(dest_addr.ss_family, SOCK_STREAM, IPPROTO_TCP);
    if (feed_sock_[feed_index] <= 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Unable to create socket for feed %d: errno %d (%s)", feed_index,
                      errno, strerror(errno));
        CloseFeedSocket(feed_index);
        return false;
    }
    CONSOLE_INFO("CommsManager::IPWANTask", "Socket for feed %d created, connecting to %s:%d over %s", feed_index,
                 settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index],
                 dest_addr.ss_family == AF_INET6 ? "IPv6" : "IPv4");

    // Enable TCP keepalive
    setsockopt(feed_sock_[feed_index], SOL_SOCKET, SO_KEEPALIVE, &kTCPKeepAliveEnable, sizeof(kTCPKeepAliveEnable));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPIDLE, &kTCPKeepAliveIdleSecondsBeforeStartingProbe,
               sizeof(kTCPKeepAliveIdleSecondsBeforeStartingProbe));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPINTVL, &kTCPKeepAliveIntervalSecondsBetweenProbes,
               sizeof(kTCPKeepAliveIntervalSecondsBetweenProbes));
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_KEEPCNT, &kTCPKeepAliveMaxFailedProbesBeforeDisconnect,
               sizeof(kTCPKeepAliveMaxFailedProbesBeforeDisconnect));
    // Reports are sent as whole batches: send each one right away instead of waiting for the previous one's ACK.
    static const int kTCPNoDelayEnable = 1;
    setsockopt(feed_sock_[feed_index], IPPROTO_TCP, TCP_NODELAY, &kTCPNoDelayEnable, sizeof(kTCPNoDelayEnable));
    // Closing drops unsent data and resets the connection. Without this, the socket of a feed that stopped taking
    // data keeps its send buffer for minutes of retransmissions after close(), and a few reconnects use up the heap.
    static const struct linger kTCPLingerAbortOnClose = {.l_onoff = 1, .l_linger = 0};
    setsockopt(feed_sock_[feed_index], SOL_SOCKET, SO_LINGER, &kTCPLingerAbortOnClose, sizeof(kTCPLingerAbortOnClose));
    // Allow reuse of local addresses.
    setsockopt(feed_sock_[feed_index], SOL_SOCKET, SO_REUSEADDR, &kTCPReuseAddrEnable, sizeof(kTCPReuseAddrEnable));

    // Connect without blocking. This task serves every feed, and a blocking connect() to an unreachable host held it
    // for the whole TCP connect timeout (~18 s per attempt) while the WAN queue overflowed and all feeds lost frames.
    // The connect finishes in PollFeedSocketConnect() on later passes of IPWANTask. The socket stays non-blocking for
    // its whole life: sends go through feed_send_buffers_, which never wait either.
    int flags = fcntl(feed_sock_[feed_index], F_GETFL, 0);
    fcntl(feed_sock_[feed_index], F_SETFL, flags | O_NONBLOCK);
    int err = connect(feed_sock_[feed_index], (struct sockaddr*)&dest_addr, dest_addr_len);
    if (err != 0 && errno == EINPROGRESS) {
        feed_sock_is_connecting_[feed_index] = true;
        return PollFeedSocketConnect(feed_index);
    }
    if (err != 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Socket unable to connect to URI %s:%d for feed %d: errno %d (%s)",
                      settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index],
                      feed_index, errno, strerror(errno));
        CloseFeedSocket(feed_index);
        return false;
    }
    return FinishFeedSocketConnect(feed_index);
}

bool CommsManager::ConnectFeedUDP(uint16_t feed_index) {
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - feed_sock_last_connect_timestamp_ms_[feed_index] <= kTCPSocketReconnectIntervalMs) {
        return false;
    }
    feed_sock_last_connect_timestamp_ms_[feed_index] = timestamp_ms;

    const char* uri = settings_manager.settings.feed_uris[feed_index];
    uint16_t port = settings_manager.settings.feed_ports[feed_index];
    struct addrinfo* dest = nullptr;
    if (!ResolveFeedURI(uri, port, HasIPv4() ? (HasIPv6() ? AF_UNSPEC : AF_INET) : AF_INET6, SOCK_DGRAM, &dest)) {
        return false;
    }
    int sock = socket(dest->ai_family, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Unable to create UDP socket for feed %d: errno %d (%s)", feed_index,
                      errno, strerror(errno));
        freeaddrinfo(dest);
        return false;
    }
    // Allows broadcast destinations such as 192.168.1.255.
    static const int kBroadcastEnable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &kBroadcastEnable, sizeof(kBroadcastEnable));
    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    // Sets the destination for send(); no packets are exchanged.
    int err = connect(sock, dest->ai_addr, dest->ai_addrlen);
    freeaddrinfo(dest);
    if (err != 0) {
        CONSOLE_ERROR("CommsManager::IPWANTask", "Unable to send UDP to %s:%d for feed %d: errno %d (%s)", uri, port,
                      feed_index, errno, strerror(errno));
        close(sock);
        return false;
    }
    feed_sock_[feed_index] = sock;
    feed_sock_is_connected_[feed_index] = true;
    CONSOLE_INFO("CommsManager::IPWANTask", "Feed %d sends UDP datagrams to %s:%d.", feed_index, uri, port);
    return true;
}

bool CommsManager::SendFeedDatagrams(uint16_t feed_index, const char* buf, uint16_t buf_len, uint16_t num_msgs) {
    feed_send_heap_is_low_ = FeedSendHeapIsLow();
    if (feed_send_heap_is_low_) {
        feed_num_dropped_msgs_[feed_index] += num_msgs;
        return true;
    }
    SettingsManager::ReportingProtocol protocol = settings_manager.settings.feed_protocols[feed_index];
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(buf);
    uint16_t offset = 0;
    while (offset < buf_len) {
        uint16_t datagram_len = FeedDatagram::NextLen(protocol, bytes + offset, buf_len - offset);
        if (send(feed_sock_[feed_index], bytes + offset, datagram_len, MSG_DONTWAIT) < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS || errno == ENOMEM) {
                // lwIP is briefly out of buffers: the rest of the batch is lost, like a dropped datagram.
                feed_num_dropped_msgs_[feed_index] += num_msgs;
                return true;
            }
            CONSOLE_ERROR("CommsManager::SendBuf", "Closing UDP feed %d (%s:%d): errno %d (%s).", feed_index,
                          settings_manager.settings.feed_uris[feed_index],
                          settings_manager.settings.feed_ports[feed_index], errno, strerror(errno));
            CloseFeedSocket(feed_index);
            return false;
        }
        offset += datagram_len;
    }
    feed_mps_counter_[feed_index] += num_msgs;
    return true;
}

bool CommsManager::PollFeedSocketConnect(uint16_t feed_index) {
    int sock = feed_sock_[feed_index];
    fd_set write_set;
    FD_ZERO(&write_set);
    FD_SET(sock, &write_set);
    struct timeval no_wait = {.tv_sec = 0, .tv_usec = 0};
    int res = select(sock + 1, NULL, &write_set, NULL, &no_wait);
    if (res == 0) {
        // Still connecting.
        if (get_time_since_boot_ms() - feed_sock_last_connect_timestamp_ms_[feed_index] > kTCPSocketConnectTimeoutMs) {
            CONSOLE_ERROR("CommsManager::IPWANTask", "Timed out after %lu ms connecting to URI %s:%d for feed %d.",
                          kTCPSocketConnectTimeoutMs, settings_manager.settings.feed_uris[feed_index],
                          settings_manager.settings.feed_ports[feed_index], feed_index);
            CloseFeedSocket(feed_index);
        }
        return false;
    }
    int so_error = 0;
    socklen_t so_error_len = sizeof(so_error);
    if (res < 0 || getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &so_error_len) != 0 || so_error != 0) {
        int error_code = so_error != 0 ? so_error : errno;
        CONSOLE_ERROR("CommsManager::IPWANTask", "Socket unable to connect to URI %s:%d for feed %d: errno %d (%s)",
                      settings_manager.settings.feed_uris[feed_index], settings_manager.settings.feed_ports[feed_index],
                      feed_index, error_code, strerror(error_code));
        CloseFeedSocket(feed_index);
        return false;
    }
    feed_sock_is_connecting_[feed_index] = false;
    return FinishFeedSocketConnect(feed_index);
}

bool CommsManager::FinishFeedSocketConnect(uint16_t feed_index) {
    CONSOLE_INFO("CommsManager::IPWANTask", "Successfully connected to %s",
                 settings_manager.settings.feed_uris[feed_index]);
    feed_sock_is_connected_[feed_index] = true;
    feed_send_buffers_[feed_index].Open(
        {.write = FeedSocketWrite,
         .ctx = reinterpret_cast<void*>(static_cast<intptr_t>(feed_sock_[feed_index])),
         .backlog_capacity_bytes =
             HardwareCapabilities::HasPSRAM() ? kFeedBacklogBytesPSRAM : kFeedBacklogBytesInternal,
         .alloc = FeedBacklogAlloc,
         .dealloc = heap_caps_free},
        get_time_since_boot_ms());
    feed_wait_budget_ms_[feed_index] = kFeedWaitBudgetMs;

    // Perform beginning-of-connection actions here.
    switch (settings_manager.settings.feed_protocols[feed_index]) {
        case SettingsManager::ReportingProtocol::kBeast:
        case SettingsManager::ReportingProtocol::kBeastNoUAT:
        case SettingsManager::ReportingProtocol::kBeastNoUATUplink: {
            uint8_t beast_message_buf[2 /* 0x1a 0xe4 */ + BeastReporter::kUuidNumChars +
                                      BeastReporter::kBeastCapsFrameLenBytes];
            uint16_t beast_message_len_bytes = BeastReporter::BuildFeedStartFrame(
                beast_message_buf, settings_manager.settings.feed_receiver_ids[feed_index]);
            // The capabilities query follows 0xe4 in the same write, so both go out in one TCP segment.
            beast_message_len_bytes += feed_caps_[feed_index].Start(beast_message_buf + beast_message_len_bytes,
                                                                    BeastCapsToOffer(), get_time_since_boot_ms());
            feed_rx_open_[feed_index] = true;
            // The stream must start with this frame: if it can't be taken now, reconnect in the next interval.
            if (feed_send_heap_is_low_ ||
                feed_send_buffers_[feed_index].Write(beast_message_buf, beast_message_len_bytes,
                                                     get_time_since_boot_ms()) > FeedSendBuffer::kQueued) {
                CONSOLE_ERROR("CommsManager::IPWANTask",
                              "Could not send the %d Byte Beast start of feed message to feed %d with URI %s on "
                              "port %d.",
                              beast_message_len_bytes, feed_index, settings_manager.settings.feed_uris[feed_index],
                              settings_manager.settings.feed_ports[feed_index]);
                CloseFeedSocket(feed_index);
                return false;
            }
            break;
        }
        default:
            // No start of connections actions required for other protocols.
            break;
    }
    return true;
}

uint16_t CommsManager::BeastCapsToOffer() {
    uint16_t offer = 0;
    if (settings_manager.settings.remote_id_rx_enabled) offer |= BeastReporter::kBeastCapRemoteID;
    return offer & BeastReporter::kBeastCapsSendable;
}

void CommsManager::ReceiveFeedSocket(uint16_t feed_index) {
    if (!feed_rx_open_[feed_index]) return;
    BeastReporter::CapsSession& caps = feed_caps_[feed_index];
    const bool was_waiting = caps.state() == BeastReporter::CapsSession::kWaiting;
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (!was_waiting && timestamp_ms - feed_rx_last_drain_timestamp_ms_[feed_index] < kFeedRecvDrainIntervalMs) {
        return;
    }
    feed_rx_last_drain_timestamp_ms_[feed_index] = timestamp_ms;
    uint8_t buf[kFeedRecvChunkBytes];
    // Also drained after the check, so pings can't fill the receive window.
    for (uint16_t chunk = 0; chunk < kFeedRecvMaxChunksPerPass; chunk++) {
        int len = recv(feed_sock_[feed_index], buf, sizeof(buf), MSG_DONTWAIT);
        if (len <= 0) {
            if (len == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                // The server closed its side or the socket failed: stop reading; sends report it as before.
                feed_rx_open_[feed_index] = false;
            }
            break;
        }
        if (caps.Receive(buf, len, timestamp_ms)) {
            // Answer in time: this pass's frames already use the agreed bits.
            CONSOLE_INFO("CommsManager::ReceiveFeedSocket", "Feed %d: offered capabilities 0x%x, using 0x%x.",
                         feed_index, caps.offered(), caps.bits());
        }
    }
    caps.Update(timestamp_ms);
    if (!feed_rx_open_[feed_index] && caps.state() == BeastReporter::CapsSession::kWaiting) {
        caps.Reset();  // No answer can arrive any more: plain Beast.
    }
    if (was_waiting && caps.state() == BeastReporter::CapsSession::kPlain) {
        CONSOLE_INFO("CommsManager::ReceiveFeedSocket", "Feed %d: no capabilities answer, plain Beast.", feed_index);
    }
}

bool CommsManager::SendBuf(uint16_t iface, const char* buf, uint16_t buf_len, uint16_t num_msgs) {
    if (iface >= SettingsManager::Settings::kMaxNumFeeds) {
        CONSOLE_ERROR("CommsManager::SendBuf", "Invalid feed index %d.", iface);
        return false;
    }
    if (!feed_sock_is_connected_[iface]) {
        CONSOLE_ERROR("CommsManager::SendBuf", "Can't send to feed %d, socket not connected.", iface);
        return false;
    }
    switch (feed_transports_in_use_[iface]) {
        case SettingsManager::kFeedTransportUDP:
            return SendFeedDatagrams(iface, buf, buf_len, num_msgs);
        case SettingsManager::kFeedTransportMQTT:
            return PublishFeedMQTT(iface, buf, buf_len, num_msgs);
        default:
            break;
    }

#ifdef ENABLE_TCP_SOCKET_RATE_LIMITING
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_send_buf_counter_reset_timestamp_ms > kSendBufRateLimitIntervalMs) {
        // Reset the counter every 10ms.
        last_send_buf_counter_reset_timestamp_ms = timestamp_ms;
        send_buf_counter_bytes = 0;
    }
    if (send_buf_counter_bytes + buf_len > kSendBufRateLimitBytesPerInterval) {
        vTaskDelay(
            pdTICKS_TO_MS(kRateLimitDelayDurationMs));  // Delay to yield to other tasks and allow buffers to clear.
    }
    send_buf_counter_bytes += buf_len;
#endif

    // Every send can take a TCP segment and a WiFi TX buffer from the heap, so the heap is checked before each one.
    feed_send_heap_is_low_ = FeedSendHeapIsLow();
    // Backlog full: wait for the socket out of this feed's budget.
    while (!feed_send_heap_is_low_ && feed_wait_budget_ms_[iface] > 0 &&
           !feed_send_buffers_[iface].HasRoomFor(buf_len)) {
        vTaskDelay(pdMS_TO_TICKS(1));
        feed_wait_budget_ms_[iface]--;
        if (!feed_send_buffers_[iface].Flush(get_time_since_boot_ms())) {
            break;  // Write() below reports the socket error.
        }
    }
    // While the heap is low the socket is offered nothing and the batch is dropped whole.
    FeedSendBuffer::Result result =
        feed_send_heap_is_low_
            ? FeedSendBuffer::kDropped
            : feed_send_buffers_[iface].Write(reinterpret_cast<const uint8_t*>(buf), buf_len, get_time_since_boot_ms());
    switch (result) {
        case FeedSendBuffer::kSent:
        case FeedSendBuffer::kQueued:
            feed_mps_counter_[iface] += num_msgs;  // Log that a message was sent in statistics.
            break;
        case FeedSendBuffer::kDropped:
            // This feed is behind. Its messages are counted and logged by UpdateFeedSendBuffers(); other feeds carry
            // on.
            feed_num_dropped_msgs_[iface] += num_msgs;
            break;
        case FeedSendBuffer::kError:
        default:
            // Either the socket failed, or it took part of a batch and there was no memory to keep the rest.
            CONSOLE_ERROR("CommsManager::SendBuf",
                          "Closing feed %d (%s:%d): socket error or no backlog memory for a cut %d byte batch (last "
                          "errno %d, %s).",
                          iface, settings_manager.settings.feed_uris[iface],
                          settings_manager.settings.feed_ports[iface], buf_len, errno, strerror(errno));
            CloseFeedSocket(iface);
            return false;
    }
    return true;
}

bool CommsManager::FeedSendHeapIsLow() {
    const uint32_t dma_backpressure_threshold_bytes = (wifi_ap_enabled || wifi_sta_enabled)
                                                          ? kDMABackPressureThresholdBytes
                                                          : kDMABackPressureThresholdEthernetOnlyBytes;
    return HardwareCapabilities::GetInternalFreeBytes() < kHeapBackPressureThresholdBytes ||
           heap_caps_get_free_size(MALLOC_CAP_DMA) < dma_backpressure_threshold_bytes;
}

void CommsManager::UpdateFeedSendBuffers() {
    uint32_t timestamp_ms = get_time_since_boot_ms();

    feed_send_heap_is_low_ = FeedSendHeapIsLow();

    uint32_t wait_budget_refill_ms =
        (timestamp_ms - feed_wait_budget_last_refill_timestamp_ms_) / kFeedWaitBudgetRefillDivisor;
    feed_wait_budget_last_refill_timestamp_ms_ += wait_budget_refill_ms * kFeedWaitBudgetRefillDivisor;

    bool log_drops = timestamp_ms - feed_drop_last_log_timestamp_ms_ >= kFeedDropLogIntervalMs;
    for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
        if (feed_sock_is_connected_[i] && feed_transports_in_use_[i] == SettingsManager::kFeedTransportTCP) {
            if (feed_send_buffers_[i].BacklogLenBytes() == 0) {
                // Caught up: earn back waiting time.
                feed_wait_budget_ms_[i] = MIN(kFeedWaitBudgetMs, feed_wait_budget_ms_[i] + wait_budget_refill_ms);
            }
            if (!feed_send_heap_is_low_ && !feed_send_buffers_[i].Flush(timestamp_ms)) {
                CONSOLE_ERROR("CommsManager::IPWANTask", "Socket error on feed %d (%s:%d): errno %d (%s).", i,
                              settings_manager.settings.feed_uris[i], settings_manager.settings.feed_ports[i], errno,
                              strerror(errno));
                CloseFeedSocket(i);
            } else if (feed_send_buffers_[i].StalledMs(timestamp_ms) > kFeedStallTimeoutMs) {
                CONSOLE_WARNING("CommsManager::IPWANTask",
                                "Feed %d (%s:%d) took no data for %lu ms, closing its socket to reconnect.", i,
                                settings_manager.settings.feed_uris[i], settings_manager.settings.feed_ports[i],
                                kFeedStallTimeoutMs);
                CloseFeedSocket(i);
            }
        }
        if (log_drops && feed_num_dropped_msgs_[i] > 0) {
            CONSOLE_WARNING("CommsManager::IPWANTask",
                            "Feed %d (%s:%d) is not keeping up: dropped %lu messages for this feed in %lu ms%s.", i,
                            settings_manager.settings.feed_uris[i], settings_manager.settings.feed_ports[i],
                            feed_num_dropped_msgs_[i], timestamp_ms - feed_drop_last_log_timestamp_ms_,
                            feed_send_heap_is_low_ ? " (heap low)" : "");
            feed_num_dropped_msgs_[i] = 0;
        }
    }
    if (log_drops) {
        feed_drop_last_log_timestamp_ms_ = timestamp_ms;
    }
}

void CommsManager::UpdateFeedMetrics() {
    // Update feed statistics once per second and print them. Put this before the queue receive so that it runs even
    // if no packets are received.
    static const uint16_t kStatsMessageMaxLen = 500;
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - feed_mps_last_update_timestamp_ms_ > kMsPerSec) {
        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            feed_mps[i] = feed_mps_counter_[i];
            feed_mps_counter_[i] = 0;
        }
        feed_mps_last_update_timestamp_ms_ = timestamp_ms;

        char feeds_metrics_message[kStatsMessageMaxLen] = {'\0'};

        for (uint16_t i = 0; i < SettingsManager::Settings::kMaxNumFeeds; i++) {
            char single_feed_metrics_message[kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds] = {'\0'};
            snprintf(single_feed_metrics_message, kStatsMessageMaxLen / SettingsManager::Settings::kMaxNumFeeds,
                     "%d:[%d] ", i, feed_mps[i]);
            strcat(feeds_metrics_message, single_feed_metrics_message);
        }
        CONSOLE_INFO("CommsManager::IPWANTask", "Feed msgs/s: %s", feeds_metrics_message);
    }
}

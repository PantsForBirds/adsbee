// Writes a settings blob laid out by a release's own settings.hh. Build against a release tag's firmware/common:
// g++ -std=c++17 -I<stub with empty CONSOLE_* macros in comms.hh> -I<tag>/firmware/common/{settings,utils,utils/crc} gen.cc <tag>/firmware/common/utils/crc/crc.cpp
#include <cstdio>
#include <cstring>
#include "settings.hh"

int main() {
    static SettingsManager::Settings s;  // Release defaults from the release's own constructor.
    auto& c = s.core_network_settings;
    c.esp32_enabled = true;
    strcpy(c.hostname, "attic-bee");
    c.wifi_ap_enabled = false;
    c.wifi_ap_channel = 6;
    strcpy(c.wifi_ap_ssid, "ADSBee-AP-1234");
    strcpy(c.wifi_ap_password, "apsecret99");
    c.wifi_sta_enabled = true;
    strcpy(c.wifi_sta_ssid, "Bench Net 2.4G");
    strcpy(c.wifi_sta_password, "correct horse battery staple");
    c.ethernet_enabled = true;
    s.r1090_rx_enabled = false;
    s.tl_offset_mv = 777;
    s.r1090_bias_tee_enabled = true;
    s.watchdog_timeout_sec = 42;
    s.log_level = static_cast<decltype(s.log_level)>(3);
    s.reporting_protocols[1] = static_cast<std::remove_reference_t<decltype(s.reporting_protocols[1])>>(2);
#ifndef NO_BAUD
    s.baud_rates[1] = 921600;
#endif
    strcpy(s.feed_uris[1], "feed.adsb.lol");
    s.feed_ports[1] = 30004;
    s.feed_is_active[1] = true;
    s.feed_protocols[1] = static_cast<std::remove_reference_t<decltype(s.feed_protocols[1])>>(5);
    for (int i = 0; i < 8; i++) s.feed_receiver_ids[1][i] = 0xA0 + i;
    s.mavlink_system_id = 9;
    s.mavlink_component_id = 200;
    c.UpdateCRC32();
    fwrite(&s, sizeof(s), 1, stdout);
    fprintf(stderr, "version %u size %zu crc 0x%08x\n", (unsigned)s.settings_version, sizeof(s), (unsigned)c.crc32);
    return 0;
}

// WiFi promiscuous sniffer for Broadcast Remote ID (ASTM F3411) over WiFi beacon frames.
//
// Used only on the internal-RAM (non-PSRAM) build when WiFi AP/STA are disabled: the WiFi radio is repurposed as a
// channel-hopping (1/6/11) sniffer while Ethernet carries IP. The driver is brought up in NULL mode with trimmed RX
// buffers and promiscuous mode enabled; beacon frames are scanned for the Open Drone ID vendor-specific IE
// (OUI FA:0B:BC, type 0x0D) and the enclosed message pack is handed to RemoteIDManager::OnRawRemoteIDPacket().
//
// On the PSRAM coexist build the radio is locked to the AP/STA channel; channel-locked best-effort sniffing there is
// left as future work (RemoteIDManager does not start the sniffer while WiFi AP/STA are up).

#include "remote_id_manager.hh"

#include <cstring>

#include "comms.hh"  // Logging.
#include "esp_wifi.h"
#include "hal.hh"  // get_time_since_boot_ms.
#include "remote_id_tx.hh"

namespace {

// Open Drone ID WiFi vendor-specific IE framing.
constexpr uint8_t kElementIdVendorSpecific = 0xDD;
constexpr uint8_t kODIDOui[3] = {0xFA, 0x0B, 0xBC};
constexpr uint8_t kODIDOuiType = 0x0D;
// Within a vendor IE's data: [OUI(3)][OUI type(1)][message counter(1)][ODID message pack ...].
constexpr uint8_t kODIDPayloadOffsetInIE = 5;

// 802.11 management frame beacon layout offsets.
constexpr uint8_t kBeaconSAOffset = 10;         // addr2 (source address) in the MAC header.
constexpr uint8_t kBeaconTaggedParamsOffset = 36;  // 24-byte MAC header + 12-byte fixed beacon params.
constexpr uint16_t kBeaconSubtype = 0x0080;      // Frame control type=mgmt(0), subtype=beacon(8).
constexpr uint16_t kFrameControlTypeSubtypeMask = 0x00FC;

bool s_sniffer_running = false;

void PromiscuousRxCb(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;
    const wifi_promiscuous_pkt_t* pkt = static_cast<const wifi_promiscuous_pkt_t*>(buf);
    const uint8_t* frame = pkt->payload;
    uint16_t frame_len = pkt->rx_ctrl.sig_len;
    if (frame_len < kBeaconTaggedParamsOffset) return;

    uint16_t frame_control = (uint16_t)frame[0] | ((uint16_t)frame[1] << 8);
    if ((frame_control & kFrameControlTypeSubtypeMask) != kBeaconSubtype) return;  // Beacons only.

    const uint8_t* sa = &frame[kBeaconSAOffset];

    // Walk the tagged parameters (Information Elements) looking for the ODID vendor-specific IE.
    uint16_t i = kBeaconTaggedParamsOffset;
    while (i + 2 <= frame_len) {
        uint8_t element_id = frame[i];
        uint8_t ie_len = frame[i + 1];
        const uint8_t* ie_data = &frame[i + 2];
        if (i + 2 + ie_len > frame_len) break;  // Truncated IE.

        if (element_id == kElementIdVendorSpecific && ie_len >= kODIDPayloadOffsetInIE &&
            memcmp(ie_data, kODIDOui, sizeof(kODIDOui)) == 0 && ie_data[3] == kODIDOuiType) {
            uint8_t odid_len = ie_len - kODIDPayloadOffsetInIE;
            if (odid_len > 0 && odid_len <= RawRemoteIDPacket::kMaxPayloadLenBytes) {
                RawRemoteIDPacket packet;
                packet.received_timestamp_ms = get_time_since_boot_ms();
                memcpy(packet.source_mac, sa, 6);
                packet.rssi_dbm = pkt->rx_ctrl.rssi;
                packet.transport = RawRemoteIDPacket::kTransportWiFiBeacon;
                packet.channel = pkt->rx_ctrl.channel;
                packet.payload_len_bytes = odid_len;
                memcpy(packet.payload, &ie_data[kODIDPayloadOffsetInIE], odid_len);
                remote_id_manager.OnRawRemoteIDPacket(packet);
            }
            return;
        }
        i += 2 + ie_len;
    }
}

// --- Shared WiFi radio ownership -------------------------------------------------------------------------------
// Both the Remote ID sniffer (RX) and the Remote ID beacon transmitter (TX) need the WiFi driver up in NULL mode, and
// esp_wifi_init/deinit must be called exactly once. This refcount makes whichever starts first bring the radio up and
// whichever stops last tear it down.
uint8_t s_wifi_radio_users = 0;

bool WiFiRadioAcquire() {
    if (s_wifi_radio_users > 0) {
        s_wifi_radio_users++;
        return true;
    }

    // Bring up the WiFi driver in NULL mode (no AP/STA association) with trimmed RX buffers to conserve internal RAM.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.static_rx_buf_num = 4;
    cfg.dynamic_rx_buf_num = 8;
    cfg.ampdu_rx_enable = 0;

    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        CONSOLE_ERROR("remote_id_wifi", "esp_wifi_init failed: %d.", err);
        return false;
    }
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK ||
        (err = esp_wifi_set_mode(WIFI_MODE_NULL)) != ESP_OK || (err = esp_wifi_start()) != ESP_OK) {
        CONSOLE_ERROR("remote_id_wifi", "WiFi NULL-mode bring-up failed: %d.", err);
        esp_wifi_deinit();
        return false;
    }

    s_wifi_radio_users = 1;
    return true;
}

void WiFiRadioRelease() {
    if (s_wifi_radio_users == 0) return;
    if (--s_wifi_radio_users > 0) return;
    esp_wifi_stop();
    esp_wifi_deinit();
}

// --- WiFi beacon transmit ---------------------------------------------------------------------------------------
// Builds and injects an 802.11 beacon frame carrying the Open Drone ID message pack in a vendor-specific Information
// Element — the exact inverse of the parser above.

// Channel used for Remote ID beacon transmission. Channel 6 is the conventional mid-band choice and is one of the
// channels the sniffer already listens on.
constexpr uint8_t kTxBeaconChannel = 6;
// Beacon interval in Time Units (1 TU = 1.024 ms). 100 TU ~= 102.4 ms is the standard beacon rate; the transmitter
// emits on the Remote ID tick (>= 1 Hz) rather than every interval, so this is just the advertised value.
constexpr uint16_t kTxBeaconIntervalTU = 100;
// SSID advertised by the Remote ID beacon. Receivers key on the vendor IE, not the SSID.
constexpr char kTxBeaconSSID[] = "ADSBee-RID";

bool s_wifi_tx_running = false;
uint8_t s_tx_mac[6] = {0};
uint16_t s_tx_seq_num = 0;

// Assembles a complete 802.11 beacon frame with the ODID vendor IE. Returns the frame length, or 0 on failure.
uint16_t BuildODIDBeaconFrame(uint8_t* buf, uint16_t buf_len_bytes, const uint8_t* odid, uint16_t odid_len,
                              uint8_t message_counter) {
    const uint8_t ssid_len = sizeof(kTxBeaconSSID) - 1;
    // MAC header(24) + fixed beacon params(12) + SSID IE(2+len) + supported rates IE(2+1) + DS param IE(2+1)
    // + vendor IE(2 + 5 + odid_len).
    uint16_t frame_len = 24 + 12 + (2 + ssid_len) + 3 + 3 + (2 + kODIDPayloadOffsetInIE + odid_len);
    if (odid_len == 0 || frame_len > buf_len_bytes || (kODIDPayloadOffsetInIE + odid_len) > 255) return 0;

    memset(buf, 0, frame_len);
    uint16_t i = 0;

    // --- MAC header (management / beacon, broadcast destination) ---
    buf[i++] = 0x80;  // Frame control: type=management, subtype=beacon.
    buf[i++] = 0x00;
    buf[i++] = 0x00;  // Duration.
    buf[i++] = 0x00;
    memset(&buf[i], 0xFF, 6);  // Destination: broadcast.
    i += 6;
    memcpy(&buf[i], s_tx_mac, 6);  // Source.
    i += 6;
    memcpy(&buf[i], s_tx_mac, 6);  // BSSID.
    i += 6;
    uint16_t seq_ctrl = (s_tx_seq_num++ & 0x0FFF) << 4;  // Sequence control (fragment number 0).
    buf[i++] = static_cast<uint8_t>(seq_ctrl & 0xFF);
    buf[i++] = static_cast<uint8_t>(seq_ctrl >> 8);

    // --- Fixed beacon parameters ---
    uint64_t timestamp_us = static_cast<uint64_t>(get_time_since_boot_ms()) * 1000ULL;
    for (uint8_t b = 0; b < 8; b++) buf[i++] = static_cast<uint8_t>((timestamp_us >> (8 * b)) & 0xFF);
    buf[i++] = static_cast<uint8_t>(kTxBeaconIntervalTU & 0xFF);
    buf[i++] = static_cast<uint8_t>(kTxBeaconIntervalTU >> 8);
    buf[i++] = 0x00;  // Capability info (no ESS/privacy: this is a beacon-only Remote ID transmitter).
    buf[i++] = 0x00;

    // --- SSID IE ---
    buf[i++] = 0x00;  // Element ID: SSID.
    buf[i++] = ssid_len;
    memcpy(&buf[i], kTxBeaconSSID, ssid_len);
    i += ssid_len;

    // --- Supported rates IE (a single mandatory 1 Mbit/s rate keeps the frame minimal but well-formed) ---
    buf[i++] = 0x01;  // Element ID: supported rates.
    buf[i++] = 0x01;
    buf[i++] = 0x82;  // 1 Mbit/s, basic rate.

    // --- DS parameter set IE (current channel) ---
    buf[i++] = 0x03;  // Element ID: DS parameter set.
    buf[i++] = 0x01;
    buf[i++] = kTxBeaconChannel;

    // --- Open Drone ID vendor-specific IE (inverse of the parser above) ---
    buf[i++] = kElementIdVendorSpecific;
    buf[i++] = static_cast<uint8_t>(kODIDPayloadOffsetInIE + odid_len);
    memcpy(&buf[i], kODIDOui, sizeof(kODIDOui));
    i += sizeof(kODIDOui);
    buf[i++] = kODIDOuiType;
    buf[i++] = message_counter;
    memcpy(&buf[i], odid, odid_len);
    i += odid_len;

    return i;
}

}  // namespace

bool RemoteIDManager::WiFiSnifferStart() {
    if (wifi_sniffer_running_) return true;

    if (!WiFiRadioAcquire()) return false;

    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(&PromiscuousRxCb);
    esp_err_t err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        CONSOLE_ERROR("remote_id_wifi", "esp_wifi_set_promiscuous failed: %d.", err);
        WiFiRadioRelease();
        return false;
    }

    sniffer_channel_index_ = 0;
    // If the transmitter already owns the channel, leave it there (see WiFiSnifferServiceHopper).
    if (!s_wifi_tx_running) {
        esp_wifi_set_channel(kSnifferChannels[0], WIFI_SECOND_CHAN_NONE);
    }
    last_channel_hop_ms_ = get_time_since_boot_ms();
    wifi_sniffer_running_ = true;
    s_sniffer_running = true;
    CONSOLE_INFO("remote_id_wifi", "Remote ID WiFi sniffer started (channels 1/6/11).");
    return true;
}

void RemoteIDManager::WiFiSnifferStop() {
    if (!wifi_sniffer_running_) return;
    esp_wifi_set_promiscuous(false);
    wifi_sniffer_running_ = false;
    s_sniffer_running = false;
    WiFiRadioRelease();
}

void RemoteIDManager::WiFiSnifferServiceHopper() {
    // While the beacon transmitter is running it owns the channel (a hop would move the transmission off channel and
    // break receivers listening there), so the sniffer stays parked on the transmit channel and listens there.
    if (s_wifi_tx_running) return;
    uint32_t now_ms = get_time_since_boot_ms();
    if (now_ms - last_channel_hop_ms_ < kSnifferChannelDwellMs) return;
    sniffer_channel_index_ = (sniffer_channel_index_ + 1) % (sizeof(kSnifferChannels) / sizeof(kSnifferChannels[0]));
    esp_wifi_set_channel(kSnifferChannels[sniffer_channel_index_], WIFI_SECOND_CHAN_NONE);
    last_channel_hop_ms_ = now_ms;
}

bool RemoteIDManager::WiFiTxStart() {
    if (wifi_tx_running_) return true;

    if (!WiFiRadioAcquire()) return false;

    // Source/BSSID MAC for the beacon: use the interface's own MAC so the transmitter is uniquely identifiable (Remote
    // ID receivers key their track on this address).
    if (esp_wifi_get_mac(WIFI_IF_STA, s_tx_mac) != ESP_OK) {
        // Fall back to a locally-administered address derived from the base MAC if the interface has none.
        memset(s_tx_mac, 0, sizeof(s_tx_mac));
        s_tx_mac[0] = 0x02;
    }

    // The transmitter owns the channel; park the radio (and the sniffer, if running) on it.
    esp_wifi_set_channel(kTxBeaconChannel, WIFI_SECOND_CHAN_NONE);

    wifi_tx_running_ = true;
    s_wifi_tx_running = true;
    CONSOLE_INFO("remote_id_wifi", "Remote ID WiFi beacon transmitter started (channel %d).", kTxBeaconChannel);
    return true;
}

void RemoteIDManager::WiFiTxStop() {
    if (!wifi_tx_running_) return;
    wifi_tx_running_ = false;
    s_wifi_tx_running = false;
    WiFiRadioRelease();
}

void RemoteIDManager::WiFiTxServiceTick(RemoteIDTransmitter& transmitter) {
    if (!wifi_tx_running_) return;

    uint8_t odid[RemoteIDTransmitter::kMaxPackLenBytes];
    uint16_t odid_len = transmitter.BuildMessagePack(odid);
    if (odid_len == 0) return;

    // Max beacon frame: headers/IEs (~50 B) + the ODID pack.
    uint8_t frame[64 + RemoteIDTransmitter::kMaxPackLenBytes];
    uint16_t frame_len =
        BuildODIDBeaconFrame(frame, sizeof(frame), odid, odid_len,
                             transmitter.NextMessageCounter(RawRemoteIDPacket::kTransportWiFiBeacon));
    if (frame_len == 0) return;

    // en_sys_seq=false: we manage the sequence number ourselves so receivers see a monotonic beacon sequence.
    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, frame, frame_len, /*en_sys_seq=*/false);
    if (err != ESP_OK) {
        CONSOLE_WARNING("remote_id_wifi", "esp_wifi_80211_tx failed: %d.", err);
    }
}

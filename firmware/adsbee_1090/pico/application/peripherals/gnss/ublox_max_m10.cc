#include "ublox_max_m10.hh"

#include "comms.hh"             // For comms_manager (UART I/O) and CONSOLE_* logging.
#include "hal.hh"               // For get_time_since_boot_ms().
#include "hardware/watchdog.h"  // For watchdog_update() during the blocking init/config pass.

namespace {

// Append a UBX-CFG-VALSET cfgData entry (4-byte key, little-endian, followed by value_size value
// bytes, little-endian) to buf at offset. Returns the new offset.
uint16_t AppendCfgKeyValue(uint8_t* buf, uint16_t offset, uint32_t key_id, uint64_t value,
                           uint8_t value_size_bytes) {
    buf[offset++] = key_id & 0xFF;
    buf[offset++] = (key_id >> 8) & 0xFF;
    buf[offset++] = (key_id >> 16) & 0xFF;
    buf[offset++] = (key_id >> 24) & 0xFF;
    for (uint8_t i = 0; i < value_size_bytes; i++) {
        buf[offset++] = (value >> (8 * i)) & 0xFF;
    }
    return offset;
}

// 8-bit Fletcher checksum computed over a UBX frame's class, id, length, and payload bytes.
struct UbxChecksum {
    uint8_t a = 0;
    uint8_t b = 0;
    void Update(uint8_t byte) {
        a = static_cast<uint8_t>(a + byte);
        b = static_cast<uint8_t>(b + a);
    }
};

// Sanity cap on received UBX payload lengths. The largest replies we expect (UBX-NAV-SAT, UBX-MON-VER) are a few
// hundred bytes; anything bigger is a corrupted length field, and swallowing it would desync the scanner from the
// stream until timeout.
constexpr uint16_t kUbxMaxRxPayloadLenBytes = 1024;

}  // namespace

void UbloxMAXM10::SendUbxFrame(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload,
                               uint16_t payload_len) {
    comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(kUbxSync1));
    comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(kUbxSync2));

    UbxChecksum checksum;

    uint8_t header[4] = {msg_class, msg_id, static_cast<uint8_t>(payload_len & 0xFF),
                         static_cast<uint8_t>((payload_len >> 8) & 0xFF)};
    for (uint8_t i = 0; i < sizeof(header); i++) {
        comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(header[i]));
        checksum.Update(header[i]);
    }
    for (uint16_t i = 0; i < payload_len; i++) {
        comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(payload[i]));
        checksum.Update(payload[i]);
    }
    comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(checksum.a));
    comms_manager.iface_putc(SettingsManager::kGNSSUART, static_cast<char>(checksum.b));
}

bool UbloxMAXM10::ScanForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, uint8_t* out_payload,
                                    uint16_t out_cap, uint16_t* out_len, bool* is_ack) {
    // Minimal UBX frame state machine that scans the incoming byte stream (which also carries NMEA)
    // for a complete, well-formed UBX frame whose class/id match want_class/want_id. Up to out_cap
    // payload bytes are copied into out_payload (if provided); out_len receives the full payload
    // length. For ACK/NAK frames, is_ack (if provided) distinguishes ACK-ACK from ACK-NAK.
    enum State : uint8_t { kSync1, kSync2, kClass, kId, kLen1, kLen2, kPayload, kCkA, kCkB } state = kSync1;
    uint8_t msg_class = 0, msg_id = 0;
    uint16_t payload_len = 0, payload_idx = 0;
    UbxChecksum checksum;
    uint8_t received_ck_a = 0;

    uint32_t start_ms = get_time_since_boot_ms();
    char c;
    while (get_time_since_boot_ms() - start_ms < timeout_ms) {
        if (!ReadBufferedByte(c)) {
            continue;  // No byte available yet; keep polling until timeout.
        }
        uint8_t b = static_cast<uint8_t>(c);
        switch (state) {
            case kSync1:
                if (b == kUbxSync1) state = kSync2;
                break;
            case kSync2:
                checksum = UbxChecksum();
                state = (b == kUbxSync2) ? kClass : kSync1;
                break;
            case kClass:
                msg_class = b;
                checksum.Update(b);
                state = kId;
                break;
            case kId:
                msg_id = b;
                checksum.Update(b);
                state = kLen1;
                break;
            case kLen1:
                payload_len = b;
                checksum.Update(b);
                state = kLen2;
                break;
            case kLen2:
                payload_len |= static_cast<uint16_t>(b) << 8;
                checksum.Update(b);
                payload_idx = 0;
                if (payload_len > kUbxMaxRxPayloadLenBytes) {
                    state = kSync1;  // Implausible length (corrupted frame); resync instead of swallowing the stream.
                } else {
                    state = (payload_len == 0) ? kCkA : kPayload;
                }
                break;
            case kPayload:
                if (out_payload != nullptr && payload_idx < out_cap) out_payload[payload_idx] = b;
                checksum.Update(b);
                if (++payload_idx >= payload_len) state = kCkA;
                break;
            case kCkA:
                received_ck_a = b;
                state = kCkB;
                break;
            case kCkB:
                // Frame complete. Only accept it if the Fletcher checksum verifies, so corrupted frames (or
                // coincidental byte sequences in the interleaved NMEA stream) can't be mistaken for real replies/ACKs.
                // want_id == kUbxIdWildcard matches any id within want_class.
                if (received_ck_a == checksum.a && b == checksum.b && msg_class == want_class &&
                    (want_id == kUbxIdWildcard || msg_id == want_id)) {
                    if (is_ack != nullptr) *is_ack = (msg_id == kUbxIdAckAck);
                    if (out_len != nullptr) *out_len = payload_len;
                    return true;
                }
                state = kSync1;  // Bad checksum or not the frame we're after; keep scanning.
                break;
        }
    }
    return false;  // Timed out.
}

bool UbloxMAXM10::WaitForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, bool* is_ack) {
    return ScanForUbxMessage(want_class, want_id, timeout_ms, nullptr, 0, nullptr, is_ack);
}

bool UbloxMAXM10::WaitForAck(uint8_t acked_class, uint8_t acked_id) {
    // The module replies to a VALSET with UBX-ACK-ACK (class 0x05 id 0x01) on success or
    // UBX-ACK-NAK (id 0x00) on failure. Match any frame in the ACK class so a NAK is detected
    // immediately (rather than scanning until timeout), and distinguish ACK from NAK via is_ack.
    // (acked_class/acked_id are not separately verified against the payload because in our init flow
    // only one VALSET is outstanding at a time.)
    (void)acked_class;
    (void)acked_id;
    bool is_ack = false;
    if (!WaitForUbxMessage(kUbxClassAck, kUbxIdWildcard, kAckTimeoutMs, &is_ack)) {
        return false;  // No ACK/NAK before timeout.
    }
    return is_ack;
}

bool UbloxMAXM10::ProbeLiveness() {
    // Poll UBX-MON-VER (a zero-length payload is a poll request per interface description S3.5.2)
    // and wait briefly for the reply (class/id 0x0A 0x04). UBX in/out are enabled on UART1 in the
    // module's factory-default state, so a present module answers even before our config pass runs.
    //
    // Retry up to kProbeAttempts times rather than probing once: u-blox does not publish a fixed
    // Vcc-to-UART-ready time and recommends polling MON-VER until it answers, so a module still
    // finishing its cold-start on the first poll is caught on a later attempt instead of being
    // declared absent. Poke the watchdog between attempts since this whole path runs before the
    // main super-loop's first poke.
    for (uint32_t attempt = 0; attempt < kProbeAttempts; attempt++) {
        SendUbxFrame(kUbxClassMon, kUbxIdMonVer, nullptr, 0);
        if (WaitForUbxMessage(kUbxClassMon, kUbxIdMonVer, kProbeTimeoutMs)) {
            return true;
        }
        watchdog_update();
    }
    return false;
}

bool UbloxMAXM10::CfgValGet(uint32_t key_id, uint8_t value_size_bytes, uint64_t& value_out) {
    // Poll one key from the RAM layer. Request payload: version(0) + layer(0=RAM) + position(u2=0)
    // + key(u4). (See interface description S3.10.4.)
    uint8_t req[4 + 4];
    req[0] = 0x00;  // version 0 (poll request)
    req[1] = 0x00;  // layer 0 = RAM (the effective, running config)
    req[2] = 0x00;  // position lo
    req[3] = 0x00;  // position hi
    AppendCfgKeyValue(req, 4, key_id, 0, 0);  // key only, no value bytes for a poll request.
    SendUbxFrame(kUbxClassCfg, kUbxIdCfgValget, req, sizeof(req));

    // Response (UBX-CFG-VALGET, version 1): version(0x01) + layer + position(u2) + repeated
    // [key(u4) + value]. For our single-key request the first (only) pair starts at offset 4.
    uint8_t resp[4 + 4 + 8];  // header + one key + up to an 8-byte value.
    uint16_t resp_len = 0;
    if (!ScanForUbxMessage(kUbxClassCfg, kUbxIdCfgValget, kAckTimeoutMs, resp, sizeof(resp), &resp_len,
                           nullptr)) {
        return false;  // No VALGET response (key unknown to the receiver, or timed out).
    }
    // Need the 4-byte header + 4-byte key + value_size_bytes of value.
    if (resp_len < static_cast<uint16_t>(4 + 4 + value_size_bytes)) {
        return false;  // Response too short (e.g. key not present in this layer).
    }
    uint64_t value = 0;
    for (uint8_t i = 0; i < value_size_bytes; i++) {
        value |= static_cast<uint64_t>(resp[4 + 4 + i]) << (8 * i);
    }
    value_out = value;
    return true;
}

bool UbloxMAXM10::CfgValSet(uint32_t key_id, uint64_t value, uint8_t value_size_bytes, uint8_t layers) {
    // Payload: version(0) + layers + reserved[2] + cfgData(key + value).
    uint8_t payload[4 + 4 + 8];
    payload[0] = 0x00;    // version 0 (transactionless)
    payload[1] = layers;  // RAM/BBR/Flash bitfield
    payload[2] = 0x00;    // reserved
    payload[3] = 0x00;    // reserved
    uint16_t len = AppendCfgKeyValue(payload, 4, key_id, value, value_size_bytes);

    SendUbxFrame(kUbxClassCfg, kUbxIdCfgValset, payload, len);
    return WaitForAck(kUbxClassCfg, kUbxIdCfgValset);
}

int32_t UbloxMAXM10::SyncConfigDefaults(uint32_t deadline_ms) {
    // Read-modify-write the firmware default configuration table into RAM + BBR. For each key, read
    // the current value with VALGET and only VALSET it when it differs. On an already-provisioned
    // module every key already matches, so nothing is written: the live navigation solution and the
    // BBR-held ephemeris/almanac/last-position survive and the module hot-starts. BBR persists
    // because V_BCKP is on an always-on rail. (Warm/hot-start-sensitive keys are commented out of
    // kUbloxConfigDefaults so they're never touched here -- see ublox_max_m10_defaults.hh.)
    //
    // Each key costs up to kAckTimeoutMs if unanswered, so a module that dies partway through this
    // ~371-key pass could otherwise stall past the watchdog window. Poke the watchdog each iteration
    // and abort (returning negative) once deadline_ms passes so the caller can mark the receiver
    // unhealthy and fall back, rather than the watchdog rebooting us mid-init.
    uint8_t layers = kCfgLayerRam | kCfgLayerBbr;
    int32_t num_written = 0;
    uint16_t num_read_fail = 0;
    for (uint32_t i = 0; i < kNumUbloxConfigDefaults; i++) {
        watchdog_update();
        // Signed difference is overflow-safe for wrapping millisecond timestamps.
        if (static_cast<int32_t>(get_time_since_boot_ms() - deadline_ms) >= 0) {
            CONSOLE_WARNING("UbloxMAXM10::SyncConfigDefaults",
                            "Config pass exceeded %lu ms budget after %lu/%u keys (%u read-backs failed so far); "
                            "module unresponsive, aborting.",
                            static_cast<unsigned long>(kInitConfigBudgetMs), static_cast<unsigned long>(i),
                            static_cast<unsigned>(kNumUbloxConfigDefaults), num_read_fail);
            return -1;
        }
        const UbloxCfgDefault& d = kUbloxConfigDefaults[i];
        uint64_t current = 0;
        if (!CfgValGet(d.key_id, d.size_bytes, current)) {
            // Couldn't read this key back; write it to be safe (fresh module / unknown state).
            num_read_fail++;
            if (CfgValSet(d.key_id, d.value, d.size_bytes, layers)) num_written++;
            continue;
        }
        if (current != d.value) {
            if (CfgValSet(d.key_id, d.value, d.size_bytes, layers)) num_written++;
        }
    }
    CONSOLE_INFO("UbloxMAXM10::SyncConfigDefaults",
                 "Config sync: wrote %ld/%u keys (%u read-backs failed).", static_cast<long>(num_written),
                 static_cast<unsigned>(kNumUbloxConfigDefaults), num_read_fail);
    return num_written;
}

void UbloxMAXM10::ApplyRuntimeMessageConfig(uint8_t layers) {
    // Factory default for NMEA message output rates is 0 (off); we need GGA + RMC at 1 Hz.
    CfgValSetU1(kCfgMsgoutNmeaGgaUart1, 1, layers);
    CfgValSetU1(kCfgMsgoutNmeaRmcUart1, 1, layers);
    // Silence the other NMEA sentences we don't consume, to reduce UART traffic.
    CfgValSetU1(kCfgMsgoutNmeaGsaUart1, 0, layers);
    CfgValSetU1(kCfgMsgoutNmeaGsvUart1, 0, layers);
    CfgValSetU1(kCfgMsgoutNmeaGllUart1, 0, layers);
    CfgValSetU1(kCfgMsgoutNmeaVtgUart1, 0, layers);
}

bool UbloxMAXM10::SendInitCommands() {
    // Quick liveness probe first: if the module is absent/unresponsive, bail immediately so we
    // don't grind through the full config pass and stall boot. Init() marks the receiver unhealthy
    // on a false return so the app falls back gracefully.
    if (!ProbeLiveness()) {
        return false;
    }

    // Read-modify-write the default table: writes only the keys that differ, so a module we've
    // already provisioned keeps its warm/hot-start data untouched. Bounded by kInitConfigBudgetMs so
    // a module that answers the probe but then stops responding can't stall init past the watchdog
    // window; a negative return means the budget was blown -> bail unhealthy and let the app fall back.
    uint32_t config_deadline_ms = get_time_since_boot_ms() + kInitConfigBudgetMs;
    if (SyncConfigDefaults(config_deadline_ms) < 0) {
        return false;
    }

    // Application-specific config on top of the defaults (RAM + BBR). These are idempotent writes;
    // if the module already has them, VALSET is harmless. (Kept as unconditional writes rather than
    // read-modify-write since they're few and none is warm-start-sensitive.)
    uint8_t layers = kCfgLayerRam | kCfgLayerBbr;

    // Sync the module's UART1 baud rate to the user-configured/persisted setting, since
    // CFG-UART1-BAUDRATE is excluded from kUbloxConfigDefaults (see WARM-START PRESERVE note there)
    // to avoid fighting this value. Read-modify-write like SyncConfigDefaults(): only actually issue
    // the VALSET when the value differs. We already probed successfully at this same rate
    // (GetDefaultBaudrate() returns the settings value), so this is normally a true no-op -- and it
    // must stay that way, because even a same-value VALSET on this key can make the module briefly
    // reset/re-latch its UART, corrupting the NMEA-enable writes that immediately follow.
    {
        uint64_t current_baudrate = 0;
        uint32_t target_baudrate = settings_manager.settings.baud_rates[SettingsManager::kGNSSUART];
        if (!CfgValGet(kCfgUart1Baudrate, 4, current_baudrate) || current_baudrate != target_baudrate) {
            CfgValSetU4(kCfgUart1Baudrate, target_baudrate, layers);
        }
    }

    // NMEA sentence selection.
    ApplyRuntimeMessageConfig(layers);

    // Enable AssistNow Autonomous so the module predicts its own orbits and shortens time-to-fix
    // even after a genuine cold start (factory default is disabled).
    CfgValSetL(kCfgAnaUseAna, true, layers);

    // TEMPORARY diagnostic: enable periodic UBX-MON-RF (antenna/AGC/noise) and UBX-NAV-SAT
    // (per-satellite C/N0 + count) output so DebugDumpModuleStatus can report why no fix is acquired.
    // Remove with the rest of the GNSS debug instrumentation.
    CfgValSetU1(kCfgMsgoutUbxMonRfUart1, 1, layers);
    CfgValSetU1(kCfgMsgoutUbxNavSatUart1, 1, layers);

    return true;
}

void UbloxMAXM10::ResendRuntimeConfig() {
    // After a UART handover (ESP32 flash) the module kept running; just re-assert the NMEA output
    // selection to RAM so sentences resume. Cheap and non-destructive -- no full config pass, so
    // the live fix is undisturbed.
    ApplyRuntimeMessageConfig(kCfgLayerRam);
    // TEMPORARY: also restore the diagnostic UBX output (remove with the rest of the debug hooks).
    CfgValSetU1(kCfgMsgoutUbxMonRfUart1, 1, kCfgLayerRam);
    CfgValSetU1(kCfgMsgoutUbxNavSatUart1, 1, kCfgLayerRam);
}

void UbloxMAXM10::DebugIngestByte(char c) {
    // TEMPORARY non-blocking UBX sniffer (remove with the rest of the GNSS debug instrumentation).
    // Runs the same UBX framing as ScanForUbxMessage but one byte per call, keeping leading payload
    // bytes for MON-RF and computing numSvs / max-C/N0 on the fly for NAV-SAT (which can be large).
    uint8_t b = static_cast<uint8_t>(c);
    switch (sniff_state_) {
        case UbxSniffState::kSync1:
            if (b == kUbxSync1) sniff_state_ = UbxSniffState::kSync2;
            break;
        case UbxSniffState::kSync2:
            sniff_state_ = (b == kUbxSync2) ? UbxSniffState::kClass : UbxSniffState::kSync1;
            break;
        case UbxSniffState::kClass:
            sniff_class_ = b;
            sniff_state_ = UbxSniffState::kId;
            break;
        case UbxSniffState::kId:
            sniff_id_ = b;
            sniff_state_ = UbxSniffState::kLen1;
            break;
        case UbxSniffState::kLen1:
            sniff_payload_len_ = b;
            sniff_state_ = UbxSniffState::kLen2;
            break;
        case UbxSniffState::kLen2:
            sniff_payload_len_ |= static_cast<uint16_t>(b) << 8;
            sniff_payload_idx_ = 0;
            sniff_nav_sat_numsvs_ = 0;
            sniff_nav_sat_max_cno_ = 0;
            sniff_state_ = (sniff_payload_len_ == 0) ? UbxSniffState::kCkA : UbxSniffState::kPayload;
            break;
        case UbxSniffState::kPayload: {
            // Buffer leading bytes (for MON-RF block-0 fields).
            if (sniff_payload_idx_ < kSniffBufLen) sniff_buf_[sniff_payload_idx_] = b;
            // Streaming NAV-SAT decode: numSvs at off 5; per-SV (12 B from off 8): cno at +2.
            if (sniff_class_ == kUbxClassNav && sniff_id_ == kUbxIdNavSat) {
                if (sniff_payload_idx_ == 5) {
                    sniff_nav_sat_numsvs_ = b;
                } else if (sniff_payload_idx_ >= 8) {
                    uint16_t off = sniff_payload_idx_ - 8;
                    if (off % 12 == 2) {  // cno byte of an SV record.
                        if (b > sniff_nav_sat_max_cno_) sniff_nav_sat_max_cno_ = b;
                    }
                }
            }
            if (++sniff_payload_idx_ >= sniff_payload_len_) sniff_state_ = UbxSniffState::kCkA;
            break;
        }
        case UbxSniffState::kCkA:
            sniff_state_ = UbxSniffState::kCkB;
            break;
        case UbxSniffState::kCkB:
            DebugDecodeSniffedFrame();
            sniff_state_ = UbxSniffState::kSync1;
            break;
    }
}

void UbloxMAXM10::DebugDecodeSniffedFrame() {
    // TEMPORARY: store a snapshot from the just-completed sniffed frame.
    if (sniff_class_ == kUbxClassMon && sniff_id_ == kUbxIdMonRf && sniff_payload_len_ >= 4 + 24) {
        // Block 0 starts at payload offset 4; fields relative to block base: +1 flags(jam bits1..0),
        // +2 antStatus, +3 antPower, +12 noisePerMS(u2), +14 agcCnt(u2), +18 magI, +20 magQ.
        const uint8_t* blk = &sniff_buf_[4];
        mon_rf_jamming_ = blk[1] & 0x03;
        mon_rf_ant_status_ = blk[2];
        mon_rf_ant_power_ = blk[3];
        mon_rf_noise_ = static_cast<uint16_t>(blk[12] | (blk[13] << 8));
        mon_rf_agc_ = static_cast<uint16_t>(blk[14] | (blk[15] << 8));
        mon_rf_mag_i_ = blk[18];
        mon_rf_mag_q_ = blk[20];
        mon_rf_valid_ = true;
    } else if (sniff_class_ == kUbxClassNav && sniff_id_ == kUbxIdNavSat) {
        nav_sat_numsvs_ = sniff_nav_sat_numsvs_;
        nav_sat_max_cno_ = sniff_nav_sat_max_cno_;
        nav_sat_valid_ = true;
    }
}

void UbloxMAXM10::DebugDumpModuleStatus() {
    // TEMPORARY: print the latest sniffed MON-RF + NAV-SAT snapshot (remove with the rest of the
    // GNSS debug instrumentation).
    if (mon_rf_valid_) {
        static const char* kAntStatusStr[] = {"INIT", "DONTKNOW", "OK", "SHORT", "OPEN"};
        static const char* kAntPowerStr[] = {"OFF", "ON", "DONTKNOW"};
        const char* ant_status_s = mon_rf_ant_status_ < 5 ? kAntStatusStr[mon_rf_ant_status_] : "?";
        const char* ant_power_s = mon_rf_ant_power_ < 3 ? kAntPowerStr[mon_rf_ant_power_] : "?";
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS RF: jam=%u antStatus=%u(%s) antPower=%u(%s) noise=%u agc=%u(/8191) magI=%u magQ=%u\r\n",
                       mon_rf_jamming_, mon_rf_ant_status_, ant_status_s, mon_rf_ant_power_, ant_power_s, mon_rf_noise_,
                       mon_rf_agc_, mon_rf_mag_i_, mon_rf_mag_q_);
    } else {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS RF: waiting for UBX-MON-RF frame...\r\n");
    }
    if (nav_sat_valid_) {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS SAT: numSvs=%u maxCno=%u dBHz\r\n", nav_sat_numsvs_, nav_sat_max_cno_);
    } else {
        CONSOLE_INFO("UbloxMAXM10::DebugDumpModuleStatus", "GNSS SAT: waiting for UBX-NAV-SAT frame...\r\n");
    }
}

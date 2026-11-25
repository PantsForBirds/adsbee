#pragma once

#include "gnss_receiver.hh"
#include "ublox_max_m10_defaults.hh"  // For kUbloxConfigDefaults[] (auto-generated).

/**
 * Driver for the u-blox MAX-M10(S) GNSS module.
 *
 * Implements the receiver-specific pieces on top of GNSSReceiver:
 *   - The UBX configuration interface (UBX-CFG-VALSET) with a selectable storage layer.
 *   - Exposed config-set helpers so any configuration item can be changed.
 *   - An init sequence that programs the full firmware default configuration into RAM + BBR as a
 *     known-good baseline (V_BCKP is an always-on battery-derived rail, so BBR persists), then
 *     applies the application-specific overrides we need (e.g. enabling GGA/RMC output).
 */
class UbloxMAXM10 : public GNSSReceiver {
   public:
    // The rate we run the module's UART1 at (t-1340). At 9600 baud our output (GGA + RMC + GST + the TEMPORARY
    // UBX-MON-RF/NAV-SAT debug output, 235 + 16 + 12 n B per epoch) exceeds the line above ~62 satellites and backs up by
    // whole seconds, so RMCs label later PPS edges (t-1336). At 115200 (11520 B/s) 120 satellites (1691 B) take 147 ms.
    static constexpr uint32_t kOperatingBaudrate = 115200;
    // Rates a module may be at before we command it, probed after the link rate: the MAX-M10S power-up default (9600,
    // what every module we've configured so far ran at) and the M10 SPG 5.10 interface description's
    // CFG-UART1-BAUDRATE default (38400). A module keeps a commanded rate in RAM while powered and in BBR while V_BCKP
    // holds, so after a reboot it is usually already at kOperatingBaudrate (warm start).
    static constexpr uint32_t kPowerUpBaudrate = 9600;
    static constexpr uint32_t kInterfaceDescriptionDefaultBaudrate = 38400;
    // After a CFG-UART1-BAUDRATE VALSET: time for the module to switch before we probe at the new rate. Its ACK (at
    // either rate, u-blox doesn't say) is waited for first, up to kAckTimeoutMs.
    static constexpr uint32_t kBaudSwitchSettleMs = 20;
    // An RMC is labeled up to this long after its PPS edge once the link runs at kOperatingBaudrate or faster (the
    // generic PPSClock::kMaxLabelDelayUs, 900 ms, is for 9600-baud receivers). Budget: the whole epoch's output takes
    // at most 147 ms on the wire (120 satellites of debug output), the module starts it within ~100 ms of the epoch
    // (u-blox doesn't publish a figure), leaving >= 250 ms for a main-loop pause before the RMC is parsed. A
    // backed-up stream lands later and goes unlabeled instead of labeling a later edge.
    static constexpr uint32_t kFastLinkMaxLabelDelayUs = 500'000;

    // Settle floor after power-on before we start probing. u-blox does not publish a fixed
    // Vcc-to-UART-ready time for the M10 (only TTFF, which is fix time, not UART readiness) and
    // recommends polling UBX-MON-VER until it answers; this is a conservative floor on top of that
    // poll-until-ready retry (see ProbeLiveness / kProbeRounds).
    // Max time to wait for a single UBX response (ACK/NAK after a VALSET, or a VALGET reply). Bounds
    // exactly one outstanding transaction: a UBX frame is <10 ms on the wire at 38400 baud and the
    // module ACKs in single/low-tens of ms; the only slack needed is for the scanner to skip past the
    // interleaved 1 Hz GGA/RMC NMEA. A NAK matches immediately (wildcard ACK-class match in
    // WaitForAck), so this timeout only elapses on a genuinely absent/unresponsive reply.
    static constexpr uint32_t kAckTimeoutMs = 150;

    // Liveness probe (UBX-MON-VER poll) timeout, per attempt. A u-blox-reported data point shows some
    // modules are still not ready to answer at 300 ms after startup, so allow a bit more headroom.
    static constexpr uint32_t kProbeTimeoutMs = 500;

    // Number of UBX-MON-VER poll rounds before declaring the module absent; each round polls once at every candidate
    // baud rate (link rate, operating rate, user setting, power-up defaults: up to 3 distinct with default settings).
    // Implements the recommended "poll until ready" pattern so a module still finishing its cold-start on the first
    // poll is still detected, rather than failing a single-shot probe. The watchdog is poked between attempts. Total
    // worst case if absent: kProbeRounds * (number of candidates) * kProbeTimeoutMs (3 s with default settings).
    static constexpr uint32_t kProbeRounds = 2;
    static constexpr uint16_t kMaxProbeBaudrates = 5;

    // Overall wall-clock budget for the blocking config pass (SyncConfigDefaults). Bounds the ~371-key
    // read-modify-write so a module that answers the probe but then stops responding cannot stall init
    // past the ~10 s watchdog window. Sized with margin below it; the per-key watchdog poke keeps the
    // watchdog fed within the budget, and exceeding it aborts the pass and marks the receiver unhealthy.
    static constexpr uint32_t kInitConfigBudgetMs = 10000;

    // UBX protocol constants.
    static constexpr uint8_t kUbxSync1 = 0xB5;
    static constexpr uint8_t kUbxSync2 = 0x62;
    static constexpr uint8_t kUbxClassCfg = 0x06;
    static constexpr uint8_t kUbxClassAck = 0x05;
    static constexpr uint8_t kUbxClassMon = 0x0A;
    static constexpr uint8_t kUbxClassNav = 0x01;
    static constexpr uint8_t kUbxIdCfgValset = 0x8A;
    static constexpr uint8_t kUbxIdCfgValget = 0x8B;
    static constexpr uint8_t kUbxIdMonVer = 0x04;
    static constexpr uint8_t kUbxIdMonRf = 0x38;   // UBX-MON-RF: per-RF-block antenna/AGC/noise status.
    static constexpr uint8_t kUbxIdNavSat = 0x35;  // UBX-NAV-SAT: per-satellite C/N0 + count.
    static constexpr uint8_t kUbxIdAckAck = 0x01;
    static constexpr uint8_t kUbxIdAckNak = 0x00;
    // Sentinel for WaitForUbxMessage to match any message id within the requested class. No real
    // UBX message id uses 0xFF.
    static constexpr uint8_t kUbxIdWildcard = 0xFF;

    // VALSET "layers" bitfield (a config-set may target multiple layers at once).
    enum CfgLayer : uint8_t {
        kCfgLayerRam = 0x01,    // Effective immediately.
        kCfgLayerBbr = 0x02,    // Battery-backed RAM; effective on restart, persists with V_BCKP.
        kCfgLayerFlash = 0x04,  // External flash (if present); effective on restart.
    };

    // Selected commonly-used configuration key IDs. The full default catalog lives in
    // ublox_max_m10_defaults.hh; these named constants are for the overrides we apply explicitly.
    // Size of each value is encoded in bits 30..28 of the key id.
    static constexpr uint32_t kCfgUart1Baudrate = 0x40520001;          // U4
    static constexpr uint32_t kCfgMsgoutNmeaGgaUart1 = 0x209100bb;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaRmcUart1 = 0x209100ac;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGstUart1 = 0x209100d4;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGsaUart1 = 0x209100c0;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGsvUart1 = 0x209100c5;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaGllUart1 = 0x209100ca;     // U1
    static constexpr uint32_t kCfgMsgoutNmeaVtgUart1 = 0x209100b1;     // U1
    static constexpr uint32_t kCfgNmeaProtver = 0x20930001;            // E1
    static constexpr uint32_t kCfgAnaUseAna = 0x10230001;             // L: AssistNow Autonomous enable
    // TEMPORARY diagnostic: periodic UBX status output on UART1 (remove with the rest of the GNSS
    // debug instrumentation).
    static constexpr uint32_t kCfgMsgoutUbxMonRfUart1 = 0x2091035a;   // U1
    static constexpr uint32_t kCfgMsgoutUbxNavSatUart1 = 0x20910016;  // U1

    UbloxMAXM10(GNSSReceiverConfig config_in) : GNSSReceiver(config_in, BSP::kGNSSModuleUbloxMAXM10) {}

    bool Init() override;

    /**
     * The rate we command the module to: kOperatingBaudrate while the GNSS baud setting is at its default (9600, which
     * every unit has unless a user set it, and the rate that backs up), otherwise the user's explicit setting. Kept
     * apart from the setting so the default doesn't change for GENERIC receivers and no settings version bump is
     * needed.
     */
    static uint32_t OperatingBaudrate();

    // Commands the module to OperatingBaudrate() if the link isn't there (AT+BAUD_RATE=GNSS_UART,... at runtime).
    void ApplyBaudRateSetting() override;

    /**
     * Set a configuration item via UBX-CFG-VALSET.
     * @param[in] key_id 32-bit configuration key ID (size encoded in bits 30..28).
     * @param[in] value Raw little-endian value to store (interpret per the item's type).
     * @param[in] value_size_bytes Number of value bytes (1, 2, 4, or 8).
     * @param[in] layers Bitwise-OR of CfgLayer values selecting where to store the value.
     * @retval True if the module acknowledged (UBX-ACK-ACK), false on NAK/timeout.
     */
    bool CfgValSet(uint32_t key_id, uint64_t value, uint8_t value_size_bytes, uint8_t layers);

    /**
     * Read a configuration item's current value via UBX-CFG-VALGET (from the RAM layer).
     * @param[in] key_id 32-bit configuration key ID (size encoded in bits 30..28).
     * @param[in] value_size_bytes Number of value bytes expected (1, 2, 4, or 8).
     * @param[out] value_out Receives the little-endian value the module reported.
     * @retval True if the module returned this key's value, false on timeout / not present.
     */
    bool CfgValGet(uint32_t key_id, uint8_t value_size_bytes, uint64_t& value_out);

    // Typed convenience wrappers around CfgValSet.
    bool CfgValSetU1(uint32_t key, uint8_t v, uint8_t layers) { return CfgValSet(key, v, 1, layers); }
    bool CfgValSetU2(uint32_t key, uint16_t v, uint8_t layers) { return CfgValSet(key, v, 2, layers); }
    bool CfgValSetU4(uint32_t key, uint32_t v, uint8_t layers) { return CfgValSet(key, v, 4, layers); }
    bool CfgValSetU8(uint32_t key, uint64_t v, uint8_t layers) { return CfgValSet(key, v, 8, layers); }
    bool CfgValSetL(uint32_t key, bool v, uint8_t layers) { return CfgValSet(key, v ? 1 : 0, 1, layers); }
    bool CfgValSetE1(uint32_t key, uint8_t v, uint8_t layers) { return CfgValSet(key, v, 1, layers); }

   protected:
    // The rate the module was last found or commanded at (OperatingBaudrate() until a probe says otherwise). A
    // commanded rate persists in the module's RAM/BBR, so this is where it is after an ESP32 flash or our reboot.
    uint32_t GetLinkBaudrate() const override { return link_baudrate_; }
    bool SendInitCommands() override;
    void ResendRuntimeConfig() override;

    // TEMPORARY debug hooks (remove with the rest of the GNSS debug instrumentation):
    //  - DebugIngestByte: passively sniffs UBX-MON-RF and UBX-NAV-SAT out of the live byte stream.
    //  - DebugDumpModuleStatus: prints the latest sniffed antenna/RF + satellite snapshot.
    void DebugIngestByte(char c) override;
    void DebugDumpModuleStatus() override;

   private:
    // Build and transmit a raw UBX frame (sync + class/id + length + payload + checksum).
    void SendUbxFrame(uint8_t msg_class, uint8_t msg_id, const uint8_t* payload, uint16_t payload_len);

    // Core UBX receive scanner: reads the incoming byte stream (up to timeout_ms) for a complete UBX
    // frame matching want_class/want_id, discarding NMEA, non-matching UBX frames, and frames with a
    // bad Fletcher checksum. Optionally copies up to out_cap payload bytes into out_payload and
    // reports the true payload length.
    // @param[out] is_ack Optional; for ACK/NAK frames, set true on ACK-ACK and false on ACK-NAK.
    // @retval True if a matching frame with a valid checksum arrived before the timeout, false otherwise.
    bool ScanForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, uint8_t* out_payload,
                           uint16_t out_cap, uint16_t* out_len, bool* is_ack);

    // Thin wrapper: wait for a matching UBX frame without capturing its payload.
    bool WaitForUbxMessage(uint8_t want_class, uint8_t want_id, uint32_t timeout_ms, bool* is_ack = nullptr);

    // Wait (up to kAckTimeoutMs) for a UBX-ACK-ACK/NAK for the given message class/id.
    // @retval True on ACK-ACK, false on ACK-NAK or timeout.
    bool WaitForAck(uint8_t acked_class, uint8_t acked_id);

    // Quick, bounded check that the module is present and talking UBX: poll UBX-MON-VER for up to
    // kProbeRounds rounds over the candidate baud rates (kProbeTimeoutMs each, watchdog poked between
    // attempts), returning as soon as it replies, with the link (our UART and link_baudrate_) left at the
    // rate that answered. Implements u-blox's recommended "poll until ready" readiness detection and gates
    // the bulk config pass so a missing module never stalls boot.
    // @retval True if the module replied.
    bool ProbeLiveness();

    // Polls UBX-MON-VER at the current link rate up to `attempts` times. @retval True if the module replied.
    bool PollMonVer(uint32_t attempts);

    // Fills rates with the distinct probe candidates, most likely first. @retval Number written.
    uint16_t ProbeBaudrates(uint32_t (&rates)[kMaxProbeBaudrates]) const;

    // Moves our UART and link_baudrate_ to baudrate (the module is not told), and the PPS label delay with it.
    void SetLinkBaudrate(uint32_t baudrate);

    // PPS label delay for the link rate: kFastLinkMaxLabelDelayUs at kOperatingBaudrate or faster.
    void UpdateLabelDelay();

    // Commands the module (answering at the link rate) to target with CFG-UART1-BAUDRATE in RAM + BBR, follows it,
    // and checks it answers there. If it doesn't, goes back to the old rate and checks that.
    // @retval True if the module answers at the link rate afterwards (target, or the old rate if the switch failed).
    bool SwitchModuleBaudrate(uint32_t target);

    // Read-modify-write the default configuration table into RAM + BBR: for each key, read the
    // module's current value (UBX-CFG-VALGET) and only write it (UBX-CFG-VALSET) if it differs.
    // On an already-provisioned module nothing is written, so the live fix and BBR-held
    // ephemeris/almanac survive and the module hot-starts. Pokes the watchdog each iteration.
    // @param[in] deadline_ms Absolute get_time_since_boot_ms() timestamp at which to abort the pass so
    //                        an unresponsive module can't stall init past the watchdog window.
    // @retval Number of keys written (0 means "already fully configured"), or a negative value if the
    //         deadline was exceeded (module stopped responding mid-pass).
    int32_t SyncConfigDefaults(uint32_t deadline_ms);

    // Enable exactly the NMEA sentences we consume (GGA + RMC + GST) and silence the rest, on UART1.
    // Cheap and idempotent; used both during init and after a UART handover (ResendRuntimeConfig).
    // @param[in] layers CfgLayer bitfield to target.
    void ApplyRuntimeMessageConfig(uint8_t layers);

    uint32_t link_baudrate_ = kOperatingBaudrate;

    // ---- TEMPORARY UBX sniffer state (remove with the rest of the GNSS debug instrumentation) ----
    // Non-blocking, byte-at-a-time UBX frame state machine fed by DebugIngestByte(). It decodes
    // UBX-MON-RF (antenna/AGC/noise) and UBX-NAV-SAT (numSvs + max C/N0) as they stream by, storing
    // the latest snapshot for DebugDumpModuleStatus() to print.
    enum class UbxSniffState : uint8_t { kSync1, kSync2, kClass, kId, kLen1, kLen2, kPayload, kCkA, kCkB };
    UbxSniffState sniff_state_ = UbxSniffState::kSync1;
    uint8_t sniff_class_ = 0;
    uint8_t sniff_id_ = 0;
    uint16_t sniff_payload_len_ = 0;
    uint16_t sniff_payload_idx_ = 0;
    static constexpr uint16_t kSniffBufLen = 32;  // Enough for the fixed MON-RF fields we read.
    uint8_t sniff_buf_[kSniffBufLen] = {0};       // Captures the leading payload bytes (MON-RF block 0).
    uint8_t sniff_nav_sat_numsvs_ = 0;            // NAV-SAT numSvs (decoded streaming).
    uint8_t sniff_nav_sat_max_cno_ = 0;           // NAV-SAT running max C/N0 across SVs (streaming).

    // Latest decoded snapshots (valid_ set true once at least one frame of each type seen).
    bool mon_rf_valid_ = false;
    uint8_t mon_rf_jamming_ = 0, mon_rf_ant_status_ = 0, mon_rf_ant_power_ = 0, mon_rf_mag_i_ = 0, mon_rf_mag_q_ = 0;
    uint16_t mon_rf_noise_ = 0, mon_rf_agc_ = 0;
    bool nav_sat_valid_ = false;
    uint8_t nav_sat_numsvs_ = 0, nav_sat_max_cno_ = 0;

    // Decode a completed sniffed frame (sniff_class_/sniff_id_/sniff_buf_) into the snapshots above.
    void DebugDecodeSniffedFrame();
};

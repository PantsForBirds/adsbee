#include <cstring>  // For memcpy.

#include "comms.hh"
#include "hal.hh"  // For timestamping.
#include "settings.hh"

// Reporting utils.
#include "aircraftjson_utils.hh"
#include "beast_utils.hh"
#include "csbee_utils.hh"
#include "gdl90_utils.hh"
#include "mavlink_utils.hh"
#include "fec.hh"        // For UATReedSolomon::DeInterleaveUplinkMessage.
#include "raw_utils.hh"
#include "uat_packet.hh"  // For DecodedUATUplinkPacket.

#ifdef ON_ESP32
AircraftDictionary& aircraft_dictionary = adsbee_server.aircraft_dictionary;
#else
#include "adsbee.hh"  // For access to the aircraft dictionary.
#include "gnss_interface.hh"
AircraftDictionary& aircraft_dictionary = adsbee.aircraft_dictionary;
#endif

GDL90Reporter gdl90;

// Failure tallies, one per reporting function. File scope rather than CommsManager members because
// there is a single CommsManager per device and this keeps the three per-platform comms.hh headers
// (which already triplicate the round state) untouched.
static CommsManager::ReportFailureTally raw_tally, beast_tally, csbee_tally, gdl90_tally, gdl90_uplink_tally,
    aircraftjson_tally;

/**
 * These have to be macros rather than helper functions: the RP2040 and 1421 CONSOLE_* macros
 * string-concatenate the tag into the format string, so the tag must be a literal. The do/while
 * wrapper also makes them safe to use unbraced, which the ESP32 CONSOLE_* macros are not.
 */
#define EMIT_REPORT_TALLY(tag, tally)                                                                              \
    do {                                                                                                           \
        CONSOLE_ERROR(tag, "%u send(s) and %u frame build(s) failed; first send failure on sink %u (UID %lu).",     \
                      (tally).num_send_failures, (tally).num_build_failures, (tally).first_failed_sink,            \
                      (unsigned long)(tally).first_failed_uid);                                                    \
        uint32_t flush_timestamp_ms = get_time_since_boot_ms();                                                    \
        (tally) = CommsManager::ReportFailureTally{};                                                              \
        (tally).last_logged_timestamp_ms = flush_timestamp_ms;                                                     \
    } while (0)

// Flushes at the end of a reporting round, so every chunk's failures collapse into one line.
#define FLUSH_REPORT_TALLY(tag, tally)                                             \
    do {                                                                           \
        if ((tally).num_send_failures > 0 || (tally).num_build_failures > 0) {     \
            EMIT_REPORT_TALLY(tag, tally);                                         \
        }                                                                          \
    } while (0)

// Flushes on the raw packet path, which runs at up to 20Hz and has no round boundary to flush on.
// Leaves the tally intact when the rate limit blocks a flush, so counts accumulate into the next window.
#define FLUSH_REPORT_TALLY_RATE_LIMITED(tag, tally)                                                        \
    do {                                                                                                   \
        if (((tally).num_send_failures > 0 || (tally).num_build_failures > 0) &&                           \
            get_time_since_boot_ms() - (tally).last_logged_timestamp_ms >= kSendFailureLogIntervalMs) {    \
            EMIT_REPORT_TALLY(tag, tally);                                                                 \
        }                                                                                                  \
    } while (0)

bool CommsManager::SendBufToSinks(const ReportSink* sinks, uint16_t num_sinks, const char* buf, uint16_t buf_len,
                                  ReportFailureTally& tally, uint16_t num_msgs, uint32_t uid) {
    bool ret = true;
    for (uint16_t i = 0; i < num_sinks; i++) {
        // Test each send's own return value. Folding into a running flag and then checking it would
        // blame every subsequent sink for the first sink's failure.
        if (!SendBuf(sinks[i], buf, buf_len, num_msgs)) {
            if (tally.num_send_failures == 0) {
                tally.first_failed_sink = i;
                tally.first_failed_uid = uid;
            }
            tally.num_send_failures++;
            ret = false;
        }
    }
    return ret;
}

// Set to 0 to revert to per-packet sends for debugging.
#ifndef COMMS_REPORTING_BATCH_SENDS
#define COMMS_REPORTING_BATCH_SENDS 1
#endif

// Max packets of each type that can fit in one composite array, used to size batch buffers.
static constexpr size_t kMaxModeSPerCompositeArray =
    (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) / sizeof(RawModeSPacket);
static constexpr size_t kMaxUATADSBPerCompositeArray =
    (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) / sizeof(RawUATADSBPacket);
static constexpr size_t kMaxUATUplinkPerCompositeArray =
    (CompositeArray::RawPackets::kMaxLenBytes - sizeof(CompositeArray::RawPackets::Header)) /
    sizeof(RawUATUplinkPacket);

// Conservative upper bounds for batch buffer sizes. Sums the per-type maxima — no single composite
// array can hit all three simultaneously (they share the 2kB budget), but the sum is provably safe.
// Beast: kModeSBeastFrameMaxLenBytes is derived from kExtendedSquitterPacketLenBytes (14 bytes),
//        which is already the worst-case payload; short squitter (7 bytes) produces smaller frames.
static constexpr size_t kBeastBatchBufMaxBytes =
    kMaxModeSPerCompositeArray * BeastReporter::kModeSBeastFrameMaxLenBytes;
// Raw frames are batched into a fixed 2 kB buffer that is flushed whenever the next frame wouldn't fit, so a full
// composite array goes out in a few sends rather than one. This replaces a ~13 kB worst-case buffer; on the PSRAM-less
// ESP32-S3 that RAM comes straight out of the heap that WiFi, lwIP and httpd share.
static constexpr size_t kRawBatchBufMaxBytes = 2048;
static_assert(kRawBatchBufMaxBytes >= kRawModeSFrameMaxNumChars &&
                  kRawBatchBufMaxBytes >= kRawUATADSBFrameMaxNumChars &&
                  kRawBatchBufMaxBytes >= kRawUATUplinkFrameMaxNumChars,
              "Raw batch buffer must be able to hold at least one frame of each type.");

bool CommsManager::UpdateReporting(const ReportSink* sinks, const SettingsManager::ReportingProtocol* sink_protocols,
                                   uint16_t num_sinks, const CompositeArray::RawPackets* packets_to_report) {
    bool ret = true;
    uint32_t timestamp_ms = get_time_since_boot_ms();

    // Every sink lands in exactly one per-protocol array below, so each array must be able to hold all
    // sinks a caller can legally pass (serial interfaces on the Pico, IP feeds on the ESP32).
    if (num_sinks > SettingsManager::kMaxNumReportSinks) {
        CONSOLE_ERROR("CommsManager::UpdateReporting", "Called with %u sinks but arrays are sized for %u; clamping.",
                      num_sinks, SettingsManager::kMaxNumReportSinks);
        num_sinks = SettingsManager::kMaxNumReportSinks;
    }

    // Build lists of sinks for each reporting protocol.
    ReportSink raw_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink beast_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink beast_no_uat_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink beast_no_uat_uplink_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink csbee_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink mavlink1_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink mavlink2_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink gdl90_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink gdl90_no_uat_uplink_sinks[SettingsManager::kMaxNumReportSinks];
    ReportSink aircraftjson_sinks[SettingsManager::kMaxNumReportSinks];

    uint16_t num_raw_sinks = 0, num_beast_sinks = 0, num_beast_no_uat_sinks = 0, num_beast_no_uat_uplink_sinks = 0,
             num_csbee_sinks = 0, num_mavlink1_sinks = 0, num_mavlink2_sinks = 0, num_gdl90_sinks = 0,
             num_gdl90_no_uat_uplink_sinks = 0, num_aircraftjson_sinks = 0;

    for (uint16_t i = 0; i < num_sinks; i++) {
        switch (sink_protocols[i]) {
            case SettingsManager::kNoReports:
                break;  // Not a reporting protocol.
            case SettingsManager::kRaw:
                raw_sinks[num_raw_sinks++] = sinks[i];
                break;
            case SettingsManager::kBeast:
                beast_sinks[num_beast_sinks++] = sinks[i];
                break;
            case SettingsManager::kBeastNoUAT:
                beast_no_uat_sinks[num_beast_no_uat_sinks++] = sinks[i];
                break;
            case SettingsManager::kBeastNoUATUplink:
                beast_no_uat_uplink_sinks[num_beast_no_uat_uplink_sinks++] = sinks[i];
                break;
            case SettingsManager::kCSBee:
                csbee_sinks[num_csbee_sinks++] = sinks[i];
                break;
            case SettingsManager::kMAVLINK1:
                mavlink1_sinks[num_mavlink1_sinks++] = sinks[i];
                break;
            case SettingsManager::kMAVLINK2:
                mavlink2_sinks[num_mavlink2_sinks++] = sinks[i];
                break;
            case SettingsManager::kGDL90:
                gdl90_sinks[num_gdl90_sinks++] = sinks[i];
                break;
            case SettingsManager::kGDL90NoUATUplink:
                gdl90_no_uat_uplink_sinks[num_gdl90_no_uat_uplink_sinks++] = sinks[i];
                break;
            case SettingsManager::kAircraftJSON:
                aircraftjson_sinks[num_aircraftjson_sinks++] = sinks[i];
                break;
            default:
                // Both values are printed numerically on purpose. Reaching this arm means
                // sink_protocols[i] is past the end of kReportingProtocolStrs by construction, and
                // sinks[i] is a feed index (up to kMaxNumFeeds) on the ESP32 rather than an index
                // into the three-entry kSerialInterfaceStrs.
                CONSOLE_ERROR("CommsManager::UpdateReporting", "Unrecognized reporting protocol %u on sink %u, skipping.",
                              static_cast<unsigned>(sink_protocols[i]), static_cast<unsigned>(sinks[i]));
                break;  // Not a periodic report protocol.
        }
    }

    // Both GDL90 variants emit identical heartbeat/ownship/traffic reports and share one reporting
    // round; kGDL90NoUATUplink differs only in being excluded from the uplink pass-through below.
    ReportSink gdl90_all_sinks[SettingsManager::kMaxNumReportSinks];
    uint16_t num_gdl90_all_sinks = 0;
    for (uint16_t i = 0; i < num_gdl90_sinks; i++) {
        gdl90_all_sinks[num_gdl90_all_sinks++] = gdl90_sinks[i];
    }
    for (uint16_t i = 0; i < num_gdl90_no_uat_uplink_sinks; i++) {
        gdl90_all_sinks[num_gdl90_all_sinks++] = gdl90_no_uat_uplink_sinks[i];
    }

    /**  Report Raw Packets **/
    if (packets_to_report->len_bytes > sizeof(CompositeArray::RawPackets::Header)) {
        // Only report raw packets if they are provided (still send locally decoded reports even if no raw packets).
        if (!ReportRaw(raw_sinks, num_raw_sinks, *packets_to_report)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportRaw.");
            ret = false;
        }
        // Send all inclusive Beast reports.
        if (!ReportBeast(beast_sinks, num_beast_sinks, *packets_to_report)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportBeast.");
            ret = false;
        }
        // Send No UAT Beast reports.
        if (!ReportBeast(beast_no_uat_sinks, num_beast_no_uat_sinks, *packets_to_report,
                         SettingsManager::kBeastNoUAT)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportBeast with no UAT.");
            ret = false;
        }
        // Send No UAT Uplink Beast reports.
        if (!ReportBeast(beast_no_uat_uplink_sinks, num_beast_no_uat_uplink_sinks, *packets_to_report,
                         SettingsManager::kBeastNoUATUplink)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportBeast with no UAT or Uplink.");
            ret = false;
        }
        // Send GDL90 reports (UAT uplink only).
        if (!ReportGDL90Uplink(gdl90_sinks, num_gdl90_sinks, *packets_to_report)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during GDL90 UAT uplink report.");
            ret = false;
        }
    }

    /** Locally Decoded Reports **/
    // All locally-decoded protocols share a UID snapshot and a single 1000ms reporting interval.
    // A new round starts only when every active protocol has finished its previous round.

    // If a protocol lost its sinks mid-round, deactivate it so it doesn't block other protocols.
    if (csbee_round_active_ && num_csbee_sinks == 0) csbee_round_active_ = false;
    if (mavlink1_round_active_ && num_mavlink1_sinks == 0) mavlink1_round_active_ = false;
    if (mavlink2_round_active_ && num_mavlink2_sinks == 0) mavlink2_round_active_ = false;
    if (gdl90_round_active_ && num_gdl90_all_sinks == 0) gdl90_round_active_ = false;
    if (aircraftjson_round_active_ && num_aircraftjson_sinks == 0) aircraftjson_round_active_ = false;

    bool all_locally_decoded_done = !csbee_round_active_ && !mavlink1_round_active_ && !mavlink2_round_active_ &&
                                    !gdl90_round_active_ && !aircraftjson_round_active_;
    bool any_locally_decoded_active = (num_csbee_sinks > 0 || num_mavlink1_sinks > 0 || num_mavlink2_sinks > 0 ||
                                       num_gdl90_all_sinks > 0 || num_aircraftjson_sinks > 0);

    if (any_locally_decoded_active && all_locally_decoded_done &&
        timestamp_ms - last_locally_decoded_report_timestamp_ms_ >= kCSBeeReportingIntervalMs) {
        last_locally_decoded_report_timestamp_ms_ = timestamp_ms;
        csbee_overrun_reported_ = false;
        mavlink1_overrun_reported_ = false;
        mavlink2_overrun_reported_ = false;
        gdl90_overrun_reported_ = false;
        aircraftjson_overrun_reported_ = false;

        // Snapshot current aircraft UIDs. All protocols will walk this same array.
        report_uids_count_ = 0;
        for (auto& itr : aircraft_dictionary.dict) {
            if (report_uids_count_ < kMaxReportUIDs) {
                report_uids_[report_uids_count_++] = itr.first;
            }
        }
        csbee_report_uid_index_ = 0;
        mavlink1_report_uid_index_ = 0;
        mavlink2_report_uid_index_ = 0;
        gdl90_report_uid_index_ = 0;
        aircraftjson_report_uid_index_ = 0;

        // Activate rounds for protocols that currently have sinks.
        csbee_round_active_ = (num_csbee_sinks > 0);
        mavlink1_round_active_ = (num_mavlink1_sinks > 0);
        mavlink2_round_active_ = (num_mavlink2_sinks > 0);
        gdl90_round_active_ = (num_gdl90_all_sinks > 0);
        aircraftjson_round_active_ = (num_aircraftjson_sinks > 0);
    }

    // Drive each in-progress reporting round. Log overrun once per round if the interval lapses.
    uint32_t elapsed_ms = timestamp_ms - last_locally_decoded_report_timestamp_ms_;
    if (csbee_round_active_) {
        if (!csbee_overrun_reported_ && elapsed_ms >= kCSBeeReportingIntervalMs) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "CSBee reporting overran the %lums reporting interval.",
                          kCSBeeReportingIntervalMs);
            csbee_overrun_reported_ = true;
        }
        if (!ReportCSBee(csbee_sinks, num_csbee_sinks)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportCSBee.");
            ret = false;
        }
    }
    if (mavlink1_round_active_) {
        if (!mavlink1_overrun_reported_ && elapsed_ms >= kMAVLINKReportingIntervalMs) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "MAVLINK1 reporting overran the %lums reporting interval.",
                          kMAVLINKReportingIntervalMs);
            mavlink1_overrun_reported_ = true;
        }
        if (!ReportMAVLINK(mavlink1_sinks, num_mavlink1_sinks, 1)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportMAVLINK with MAVLINK1 sinks.");
            ret = false;
        }
    }
    if (mavlink2_round_active_) {
        if (!mavlink2_overrun_reported_ && elapsed_ms >= kMAVLINKReportingIntervalMs) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "MAVLINK2 reporting overran the %lums reporting interval.",
                          kMAVLINKReportingIntervalMs);
            mavlink2_overrun_reported_ = true;
        }
        if (!ReportMAVLINK(mavlink2_sinks, num_mavlink2_sinks, 2)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportMAVLINK with MAVLINK2 sinks.");
            ret = false;
        }
    }
    if (gdl90_round_active_) {
        if (!gdl90_overrun_reported_ && elapsed_ms >= kGDL90ReportingIntervalMs) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "GDL90 reporting overran the %lums reporting interval.",
                          kGDL90ReportingIntervalMs);
            gdl90_overrun_reported_ = true;
        }
        if (!ReportGDL90(gdl90_all_sinks, num_gdl90_all_sinks)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportGDL90.");
            ret = false;
        }
    }
    if (aircraftjson_round_active_) {
        if (!aircraftjson_overrun_reported_ && elapsed_ms >= kAircraftJSONReportingIntervalMs) {
            CONSOLE_ERROR("CommsManager::UpdateReporting",
                          "AircraftJSON reporting overran the %lums reporting interval.",
                          kAircraftJSONReportingIntervalMs);
            aircraftjson_overrun_reported_ = true;
        }
        if (!ReportAircraftJSON(aircraftjson_sinks, num_aircraftjson_sinks)) {
            CONSOLE_ERROR("CommsManager::UpdateReporting", "Error during ReportAircraftJSON.");
            ret = false;
        }
    }

    return ret;
}

bool CommsManager::ReportRaw(ReportSink* sinks, uint16_t num_sinks, const CompositeArray::RawPackets& packets) {
    if (num_sinks == 0) {
        return true;  // Nobody is listening; don't spend time building frames.
    }
    char error_msg[CompositeArray::RawPackets::kErrorMessageMaxLen] = {0};
    if (!packets.IsValid(error_msg)) {
        CONSOLE_ERROR("CommsManager::ReportRaw", "Invalid CompositeArray::RawPackets: %s", error_msg);
        return false;
    }

#if COMMS_REPORTING_BATCH_SENDS
    // Batch frames into a buffer and send it whenever it can't take the next frame, to minimize lwIP IPC round-trips
    // without holding a worst-case-sized buffer.
    static char raw_batch_buf[kRawBatchBufMaxBytes];
    uint16_t batch_len = 0;
    uint16_t total_packets = 0;
    bool ret = true;

    // The BuildRaw*Frame functions are thin wrappers around snprintf, so on truncation they return the
    // would-have-been length (>= the frame cap) and on an encoding error they return a negative that
    // wraps to a huge uint16_t. Advancing batch_len by either would walk the write cursor past the
    // bytes actually written, so validate before accumulating. The remaining-space check flushes the
    // batch when the next frame might not fit (the static_assert above guarantees it fits in an empty buffer).
#define BUILD_RAW_FRAME(build_call, frame_max_num_chars)                                            \
    do {                                                                                            \
        if (static_cast<size_t>(batch_len) + (frame_max_num_chars) > sizeof(raw_batch_buf)) {       \
            ret &= SendBufToSinks(sinks, num_sinks, raw_batch_buf, batch_len, raw_tally, total_packets); \
            batch_len = 0;                                                                          \
            total_packets = 0;                                                                      \
        }                                                                                           \
        uint16_t frame_len = (build_call);                                                          \
        if (frame_len == 0 || frame_len >= (frame_max_num_chars)) {                 \
            raw_tally.num_build_failures++;                                         \
            break;                                                                  \
        }                                                                           \
        batch_len += frame_len;                                                     \
        total_packets++;                                                            \
    } while (0)

    for (uint16_t i = 0; i < packets.header->num_mode_s_packets; i++) {
        BUILD_RAW_FRAME(BuildRawModeSFrame(packets.mode_s_packets[i], raw_batch_buf + batch_len),
                        kRawModeSFrameMaxNumChars);
    }
    for (uint16_t i = 0; i < packets.header->num_uat_adsb_packets; i++) {
        BUILD_RAW_FRAME(BuildRawUATADSBFrame(packets.uat_adsb_packets[i], raw_batch_buf + batch_len),
                        kRawUATADSBFrameMaxNumChars);
    }
    for (uint16_t i = 0; i < packets.header->num_uat_uplink_packets; i++) {
        BUILD_RAW_FRAME(BuildRawUATUplinkFrame(packets.uat_uplink_packets[i], raw_batch_buf + batch_len),
                        kRawUATUplinkFrameMaxNumChars);
    }
#undef BUILD_RAW_FRAME

    if (batch_len > 0) {
        ret &= SendBufToSinks(sinks, num_sinks, raw_batch_buf, batch_len, raw_tally, total_packets);
    }
    FLUSH_REPORT_TALLY_RATE_LIMITED("CommsManager::ReportRaw", raw_tally);
#else
    bool ret = true;
    // Same snprintf return caveat as the batch path above: a frame length at or past the cap means the
    // frame was truncated, so drop it rather than sending a length that overruns raw_frame_buf.
#define SEND_RAW_FRAME(build_call, frame_max_num_chars)                                           \
    do {                                                                                          \
        uint16_t num_bytes_in_frame = (build_call);                                               \
        if (num_bytes_in_frame == 0 || num_bytes_in_frame >= (frame_max_num_chars)) {             \
            raw_tally.num_build_failures++;                                                       \
            break;                                                                                \
        }                                                                                         \
        ret &= SendBufToSinks(sinks, num_sinks, raw_frame_buf, num_bytes_in_frame, raw_tally);    \
    } while (0)

    for (uint16_t i = 0; i < packets.header->num_mode_s_packets; i++) {
        char raw_frame_buf[kRawModeSFrameMaxNumChars];
        SEND_RAW_FRAME(BuildRawModeSFrame(packets.mode_s_packets[i], raw_frame_buf), kRawModeSFrameMaxNumChars);
    }
    for (uint16_t i = 0; i < packets.header->num_uat_adsb_packets; i++) {
        char raw_frame_buf[kRawUATADSBFrameMaxNumChars];
        SEND_RAW_FRAME(BuildRawUATADSBFrame(packets.uat_adsb_packets[i], raw_frame_buf), kRawUATADSBFrameMaxNumChars);
    }
    for (uint16_t i = 0; i < packets.header->num_uat_uplink_packets; i++) {
        char raw_frame_buf[kRawUATUplinkFrameMaxNumChars];
        SEND_RAW_FRAME(BuildRawUATUplinkFrame(packets.uat_uplink_packets[i], raw_frame_buf),
                       kRawUATUplinkFrameMaxNumChars);
    }
#undef SEND_RAW_FRAME
    FLUSH_REPORT_TALLY_RATE_LIMITED("CommsManager::ReportRaw", raw_tally);
#endif
    return ret;
}

bool CommsManager::ReportBeast(ReportSink* sinks, uint16_t num_sinks, const CompositeArray::RawPackets& packets,
                               SettingsManager::ReportingProtocol protocol) {
    if (num_sinks == 0) {
        return true;  // Nobody is listening; don't spend time building frames.
    }
    char error_msg[CompositeArray::RawPackets::kErrorMessageMaxLen] = {0};
    if (!packets.IsValid(error_msg)) {
        CONSOLE_ERROR("CommsManager::ReportBeast", "Invalid CompositeArray::RawPackets: %s", error_msg);
        return false;
    }

#if COMMS_REPORTING_BATCH_SENDS
    // Batch all frames into one buffer, then do a single send per sink to minimize lwIP IPC round-trips.
    static uint8_t beast_batch_buf[kBeastBatchBufMaxBytes];
    uint16_t batch_len = 0;
    uint16_t total_packets = 0;

    // The Beast builders return 0 on an invalid packet length and take no output buffer bound, so they
    // write blind. BuildUATADSBBeastFrame and BuildUATUplinkBeastFrame log the reason themselves;
    // BuildModeSBeastFrame fails silently, which is why the drop is tallied here for all three. The
    // remaining-space check guards the batch buffer, whose size bound is derived rather than enforced.
#define BUILD_BEAST_FRAME(build_call, frame_max_len_bytes)                     \
    do {                                                                      \
        if (static_cast<size_t>(batch_len) + (frame_max_len_bytes) > sizeof(beast_batch_buf)) { \
            beast_tally.num_build_failures++;                                 \
            break;                                                            \
        }                                                                     \
        uint16_t frame_len = (build_call);                                    \
        if (frame_len == 0) {                                                 \
            beast_tally.num_build_failures++;                                 \
            break;                                                            \
        }                                                                     \
        batch_len += frame_len;                                               \
        total_packets++;                                                      \
    } while (0)

    for (uint16_t i = 0; i < packets.header->num_mode_s_packets; i++) {
        BUILD_BEAST_FRAME(BeastReporter::BuildModeSBeastFrame(beast_batch_buf + batch_len, packets.mode_s_packets[i]),
                          BeastReporter::kModeSBeastFrameMaxLenBytes);
    }
    if (protocol != SettingsManager::kBeastNoUAT) {
        for (uint16_t i = 0; i < packets.header->num_uat_adsb_packets; i++) {
            BUILD_BEAST_FRAME(
                BeastReporter::BuildUATADSBBeastFrame(beast_batch_buf + batch_len, packets.uat_adsb_packets[i]),
                BeastReporter::kUATADSBBeastFrameMaxLenBytes);
        }
    }
    if (protocol != SettingsManager::kBeastNoUATUplink && protocol != SettingsManager::kBeastNoUAT) {
        for (uint16_t i = 0; i < packets.header->num_uat_uplink_packets; i++) {
            BUILD_BEAST_FRAME(
                BeastReporter::BuildUATUplinkBeastFrame(beast_batch_buf + batch_len, packets.uat_uplink_packets[i]),
                BeastReporter::kUATUplinkBeastFrameMaxLenBytes);
        }
    }
#undef BUILD_BEAST_FRAME

    bool ret = true;
    if (batch_len > 0) {
        ret = SendBufToSinks(sinks, num_sinks, (char*)beast_batch_buf, batch_len, beast_tally, total_packets);
    }
    FLUSH_REPORT_TALLY_RATE_LIMITED("CommsManager::ReportBeast", beast_tally);
#else
    bool ret = true;
#define SEND_BEAST_FRAME(build_call)                                                                       \
    do {                                                                                                   \
        uint16_t num_bytes_in_frame = (build_call);                                                        \
        if (num_bytes_in_frame == 0) {                                                                     \
            beast_tally.num_build_failures++;                                                              \
            break;                                                                                         \
        }                                                                                                  \
        ret &= SendBufToSinks(sinks, num_sinks, (char*)beast_frame_buf, num_bytes_in_frame, beast_tally);   \
    } while (0)

    for (uint16_t i = 0; i < packets.header->num_mode_s_packets; i++) {
        uint8_t beast_frame_buf[BeastReporter::kModeSBeastFrameMaxLenBytes];
        SEND_BEAST_FRAME(BeastReporter::BuildModeSBeastFrame(beast_frame_buf, packets.mode_s_packets[i]));
    }
    if (protocol != SettingsManager::kBeastNoUAT) {
        for (uint16_t i = 0; i < packets.header->num_uat_adsb_packets; i++) {
            uint8_t beast_frame_buf[BeastReporter::kUATADSBBeastFrameMaxLenBytes];
            SEND_BEAST_FRAME(BeastReporter::BuildUATADSBBeastFrame(beast_frame_buf, packets.uat_adsb_packets[i]));
        }
    }
    if (protocol != SettingsManager::kBeastNoUATUplink && protocol != SettingsManager::kBeastNoUAT) {
        for (uint16_t i = 0; i < packets.header->num_uat_uplink_packets; i++) {
            uint8_t beast_frame_buf[BeastReporter::kUATUplinkBeastFrameMaxLenBytes];
            SEND_BEAST_FRAME(BeastReporter::BuildUATUplinkBeastFrame(beast_frame_buf, packets.uat_uplink_packets[i]));
        }
    }
#undef SEND_BEAST_FRAME
    FLUSH_REPORT_TALLY_RATE_LIMITED("CommsManager::ReportBeast", beast_tally);
#endif
    return ret;
}

bool CommsManager::ReportCSBee(ReportSink* sinks, uint16_t num_sinks) {
    bool ret = true;

    // Process aircraft within the time budget.
    uint32_t chunk_start_ms = get_time_since_boot_ms();
    while (csbee_report_uid_index_ < report_uids_count_) {
        if (get_time_since_boot_ms() - chunk_start_ms >= kCSBeeChunkBudgetMs) {
            return ret;  // Budget exhausted; resume on next UpdateReporting tick.
        }

        uint32_t uid = report_uids_[csbee_report_uid_index_++];

        auto itr = aircraft_dictionary.dict.find(uid);
        if (itr == aircraft_dictionary.dict.end()) {
            continue;  // Aircraft pruned mid-round; skip without losing remaining entries.
        }

        char message[kCSBeeMessageStrMaxLen];
        int message_len_bytes = -1;

        if (ModeSAircraft* mode_s_aircraft = get_if<ModeSAircraft>(&(itr->second)); mode_s_aircraft) {
            message_len_bytes = WriteCSBeeModeSAircraftMessageStr(message, *mode_s_aircraft);
        } else if (UATAircraft* uat_aircraft = get_if<UATAircraft>(&(itr->second)); uat_aircraft) {
            message_len_bytes = WriteCSBeeUATAircraftMessageStr(message, *uat_aircraft);
        } else if (get_if<RemoteIDAircraft>(&(itr->second))) {
            // TODO: emit a dedicated CSBee "#R" Remote ID sentence. For now drones are reported via GDL90 and the
            // Aircraft JSON output; skip them here rather than forcing a drone into the fixed #A/#U CSV schema.
            continue;
        } else {
            CONSOLE_WARNING("CommsManager::ReportCSBee", "Unknown aircraft type in dictionary for UID 0x%lx.", uid);
            continue;
        }

        if (message_len_bytes < 0) {
            CONSOLE_ERROR("CommsManager::ReportCSBee",
                          "Error in WriteCSBeeAircraftMessageStr for UID 0x%lx, error code %d.", uid,
                          message_len_bytes);
            ret = false;
            continue;  // Log error but do not abort the round; remaining aircraft still need reporting.
        }
        if (message_len_bytes >= kCSBeeMessageStrMaxLen) {
            // The CSBee writers only return negative on an snprintf encoding error, never on truncation:
            // the body is capped at kCSBeeMessageStrMaxLen - kCRCMaxNumChars - 1 and the CRC is then
            // appended at the would-have-been offset. A length at or past the buffer size means the
            // message was truncated and its CRC is bogus, so drop it rather than sending a corrupt frame.
            csbee_tally.num_build_failures++;
            ret = false;
            continue;
        }

        ret &= SendBufToSinks(sinks, num_sinks, message, message_len_bytes, csbee_tally, 1, uid);
    }

    // All UIDs in the snapshot have been processed — send the statistics footer.
    char stats_message[kCSBeeMessageStrMaxLen];
    int16_t stats_len_bytes = WriteCSBeeStatisticsMessageStr(
        stats_message,
        aircraft_dictionary.metrics.demods_1090,                     // DPS
        aircraft_dictionary.metrics.raw_squitter_frames,             // RAW_SFPS
        aircraft_dictionary.metrics.valid_squitter_frames,           // SFPS
        aircraft_dictionary.metrics.raw_extended_squitter_frames,    // RAW_ESFPS
        aircraft_dictionary.metrics.valid_extended_squitter_frames,  // ESFPS
        aircraft_dictionary.metrics.raw_uat_adsb_frames + aircraft_dictionary.metrics.raw_uat_uplink_frames,  // RAW_UAT
        aircraft_dictionary.metrics.valid_uat_adsb_frames +
            aircraft_dictionary.metrics.valid_uat_uplink_frames,  // VALID_UAT
        aircraft_dictionary.GetNumAircraft(),                     // NUM_AIRCRAFT
        0u,                                                       // TSCAL
        get_time_since_boot_ms() / 1000                           // UPTIME
    );
    if (stats_len_bytes < 0) {
        CONSOLE_ERROR("CommsManager::ReportCSBee",
                      "Encountered an error in WriteCSBeeStatisticsMessageStr, error code %d.", stats_len_bytes);
        ret = false;
    } else if (stats_len_bytes >= kCSBeeMessageStrMaxLen) {
        CONSOLE_ERROR("CommsManager::ReportCSBee", "WriteCSBeeStatisticsMessageStr truncated at %d bytes, dropping.",
                      stats_len_bytes);
        ret = false;
    } else {
        ret &= SendBufToSinks(sinks, num_sinks, stats_message, stats_len_bytes, csbee_tally);
    }

    // End of the round: flush the accumulated failures from every chunk as one line.
    FLUSH_REPORT_TALLY("CommsManager::ReportCSBee", csbee_tally);
    csbee_round_active_ = false;
    return ret;
}

bool CommsManager::ReportMAVLINK(ReportSink* sinks, uint16_t num_sinks, uint8_t mavlink_version) {
    // Zero sinks is a normal state, not a fault: UpdateReporting already gates on sink count. Match the
    // silent success the other reporting functions return.
    if (num_sinks == 0) {
        return true;
    }
    // Unlike the other protocols, the mavlink_msg_*_send_struct calls below return void, so there is no
    // send failure signal to tally here.
    if (mavlink_version != 1 && mavlink_version != 2) {
        CONSOLE_ERROR("CommsManager::ReportMAVLINK", "MAVLINK version %d does not exist.", mavlink_version);
        return false;
    }

    uint16_t& uid_index = (mavlink_version == 1) ? mavlink1_report_uid_index_ : mavlink2_report_uid_index_;
    bool& round_active = (mavlink_version == 1) ? mavlink1_round_active_ : mavlink2_round_active_;

    // Always refresh the protocol version before any send (idempotent).
    for (uint16_t i = 0; i < num_sinks; i++) {
        mavlink_set_proto_version(sinks[i], mavlink_version);
    }

    // Send the HEARTBEAT once at the start of each round.
    if (uid_index == 0) {
        mavlink_heartbeat_t heartbeat_msg = {.custom_mode = 0,
                                             .type = MAV_TYPE_ADSB,
                                             .autopilot = MAV_AUTOPILOT_INVALID,
                                             .base_mode = 0,
                                             .system_status = MAV_STATE_ACTIVE,
                                             .mavlink_version = mavlink_version};
        for (uint16_t i = 0; i < num_sinks; i++) {
            mavlink_msg_heartbeat_send_struct(static_cast<mavlink_channel_t>(sinks[i]), &heartbeat_msg);
        }
    }

    // Send an ADSB_VEHICLE message for each aircraft within the time budget.
    uint32_t chunk_start_ms = get_time_since_boot_ms();
    while (uid_index < report_uids_count_) {
        if (get_time_since_boot_ms() - chunk_start_ms >= kMAVLINKChunkBudgetMs) {
            return true;  // Budget exhausted; resume on next UpdateReporting tick.
        }

        uint32_t uid = report_uids_[uid_index++];

        auto itr = aircraft_dictionary.dict.find(uid);
        if (itr == aircraft_dictionary.dict.end()) {
            continue;  // Aircraft pruned mid-round; skip without losing remaining entries.
        }

        mavlink_adsb_vehicle_t adsb_vehicle_msg;
        if (ModeSAircraft* mode_s_aircraft = get_if<ModeSAircraft>(&(itr->second)); mode_s_aircraft) {
            adsb_vehicle_msg = ModeSAircraftToMAVLINKADSBVehicleMessage(*mode_s_aircraft);
        } else if (UATAircraft* uat_aircraft = get_if<UATAircraft>(&(itr->second)); uat_aircraft) {
            adsb_vehicle_msg = UATAircraftToMAVLINKADSBVehicleMessage(*uat_aircraft);
        } else if (get_if<RemoteIDAircraft>(&(itr->second))) {
            // TODO: report drones as ADSB_VEHICLE (emitter type UAV) or native OPEN_DRONE_ID_* messages. For now drones
            // are surfaced via GDL90 and the Aircraft JSON output; skip them here.
            continue;
        } else {
            CONSOLE_WARNING("CommsManager::ReportMAVLINK", "Unknown aircraft type in dictionary for UID 0x%lx.", uid);
            continue;
        }
        for (uint16_t i = 0; i < num_sinks; i++) {
            mavlink_msg_adsb_vehicle_send_struct(static_cast<mavlink_channel_t>(sinks[i]), &adsb_vehicle_msg);
        }
    }

    // All UIDs processed — send the delimiter and mark this version's round complete.
    switch (mavlink_version) {
        case 1: {
            mavlink_request_data_stream_t request_data_stream_msg = {};
            for (uint16_t i = 0; i < num_sinks; i++) {
                mavlink_msg_request_data_stream_send_struct(static_cast<mavlink_channel_t>(sinks[i]),
                                                            &request_data_stream_msg);
            }
            break;
        }
        case 2: {
            mavlink_message_interval_t message_interval_msg = {
                .interval_us = (int32_t)(kMAVLINKReportingIntervalMs * (uint32_t)kUsPerMs),
                .message_id = MAVLINK_MSG_ID_ADSB_VEHICLE};
            for (uint16_t i = 0; i < num_sinks; i++) {
                mavlink_msg_message_interval_send_struct(static_cast<mavlink_channel_t>(sinks[i]),
                                                         &message_interval_msg);
            }
            break;
        }
        default:
            CONSOLE_ERROR("CommsManager::ReportMAVLINK", "MAVLINK version %d does not exist.", mavlink_version);
            round_active = false;
            return false;
    }

    round_active = false;
    return true;
}

bool CommsManager::ReportGDL90(ReportSink* sinks, uint16_t num_sinks) {
    bool ret = true;

    uint8_t buf[GDL90Reporter::kGDL90MessageMaxLenBytes];
    uint16_t msg_len;

    // Send the HEARTBEAT and OWNSHIP REPORT once at the start of each round.
    if (gdl90_report_uid_index_ == 0) {
        // Receiver position is owned by the RP2040. On the ESP32 it arrives in the composite
        // device status alongside the position-available flag.
#ifdef ON_ESP32
        const SettingsManager::RxPosition& rx_position = object_dictionary.composite_device_status.rp2040.rx_position;
        bool rx_position_available = object_dictionary.composite_device_status.rp2040.rx_position_available;
        bool gnss_utc_time_valid = object_dictionary.composite_device_status.rp2040.gnss_utc_time_valid;
#else
        const SettingsManager::RxPosition& rx_position = adsbee.rx_position;
        bool rx_position_available = adsbee.rx_position_available;
        bool gnss_utc_time_valid = gnss != nullptr && gnss->fix().utc_time_valid;
#endif
        // Build the ownship report up front: its validity also drives the heartbeat's GPS Position Valid flag.
        // Selecting RX_POSITION=GNSS routes the latest fresh GNSS fix populated by ADSBee::UpdateRxPosition().
        GDL90Reporter::GDL90TargetReportData ownship_data;
        bool have_position = GDL90Reporter::BuildOwnshipReportData(ownship_data, rx_position, rx_position_available);

        gdl90.gnss_position_valid = have_position;
        gdl90.utc_timing_is_valid = gnss_utc_time_valid;
        gdl90.maintenance_required = false;

        msg_len = gdl90.WriteGDL90HeartbeatMessage(buf, sizeof(buf), get_time_since_boot_ms() / 1000,
                                                   aircraft_dictionary.metrics.valid_squitter_frames +
                                                       aircraft_dictionary.metrics.valid_extended_squitter_frames +
                                                       aircraft_dictionary.metrics.valid_uat_adsb_frames,
                                                   aircraft_dictionary.metrics.valid_uat_uplink_frames);
        // The GDL90 writers return a byte count with no error sentinel; 0 means nothing was written.
        if (msg_len == 0) {
            gdl90_tally.num_build_failures++;
            ret = false;
        } else {
            ret &= SendBufToSinks(sinks, num_sinks, (char*)buf, msg_len, gdl90_tally);
        }

        msg_len = gdl90.WriteGDL90TargetReportMessage(buf, sizeof(buf), ownship_data, true);
        if (msg_len == 0) {
            gdl90_tally.num_build_failures++;
            ret = false;
        } else {
            ret &= SendBufToSinks(sinks, num_sinks, (char*)buf, msg_len, gdl90_tally);
        }
    }

    // Send a traffic report for each aircraft within the time budget.
    uint32_t chunk_start_ms = get_time_since_boot_ms();
    while (gdl90_report_uid_index_ < report_uids_count_) {
        if (get_time_since_boot_ms() - chunk_start_ms >= kGDL90ChunkBudgetMs) {
            // Not an error: chunking is the normal pacing mechanism. A round that genuinely overruns
            // the reporting interval is caught once per round by gdl90_overrun_reported_ in
            // UpdateReporting.
            return ret;  // Budget exhausted; resume on next UpdateReporting tick.
        }

        uint32_t uid = report_uids_[gdl90_report_uid_index_++];

        auto itr = aircraft_dictionary.dict.find(uid);
        if (itr == aircraft_dictionary.dict.end()) {
            continue;  // Aircraft pruned mid-round; skip without losing remaining entries.
        }

        msg_len = 0;
        if (ModeSAircraft* mode_s_aircraft = get_if<ModeSAircraft>(&(itr->second)); mode_s_aircraft) {
            msg_len = gdl90.WriteGDL90TargetReportMessage(buf, sizeof(buf), *mode_s_aircraft, false);
        } else if (UATAircraft* uat_aircraft = get_if<UATAircraft>(&(itr->second)); uat_aircraft) {
            msg_len = gdl90.WriteGDL90TargetReportMessage(buf, sizeof(buf), *uat_aircraft, false);
        } else if (RemoteIDAircraft* remote_id_aircraft = get_if<RemoteIDAircraft>(&(itr->second)); remote_id_aircraft) {
            if (!remote_id_aircraft->HasBitFlag(RemoteIDAircraft::kBitFlagPositionValid)) {
                // A drone heard only via Basic ID / Operator ID has no position yet. Reporting it would emit a traffic
                // target at lat/lon 0,0, and unlike Mode S / UAT a consumer can't detect that from NIC: Remote ID always
                // reports NIC = 0 because a drone's containment radius is genuinely unknown. Skip it instead.
                continue;
            }
            msg_len = gdl90.WriteGDL90TargetReportMessage(buf, sizeof(buf), *remote_id_aircraft, false);
        } else {
            CONSOLE_WARNING("CommsManager::ReportGDL90", "Unknown aircraft type in dictionary for UID 0x%lx.", uid);
            continue;
        }
        if (msg_len == 0) {
            gdl90_tally.num_build_failures++;
            ret = false;
            continue;
        }
        ret &= SendBufToSinks(sinks, num_sinks, (char*)buf, msg_len, gdl90_tally, 1, uid);
    }

    // End of the round: flush the accumulated failures from every chunk as one line.
    FLUSH_REPORT_TALLY("CommsManager::ReportGDL90", gdl90_tally);
    gdl90_round_active_ = false;
    return ret;
}

bool CommsManager::ReportAircraftJSON(ReportSink* sinks, uint16_t num_sinks) {
    bool ret = true;

    uint32_t chunk_start_ms = get_time_since_boot_ms();
    while (aircraftjson_report_uid_index_ < report_uids_count_) {
        if (get_time_since_boot_ms() - chunk_start_ms >= kAircraftJSONChunkBudgetMs) {
            // Not an error: chunking is the normal pacing mechanism. A round that genuinely overruns
            // the reporting interval is caught once per round by aircraftjson_overrun_reported_ in
            // UpdateReporting.
            return ret;  // Budget exhausted; resume on next UpdateReporting tick.
        }

        uint32_t uid = report_uids_[aircraftjson_report_uid_index_++];

        auto itr = aircraft_dictionary.dict.find(uid);
        if (itr == aircraft_dictionary.dict.end()) {
            continue;  // Aircraft pruned mid-round; skip without losing remaining entries.
        }

        char message[kAircraftJSONMessageStrMaxLen];
        int message_len_bytes = -1;

        if (ModeSAircraft* mode_s_aircraft = get_if<ModeSAircraft>(&(itr->second)); mode_s_aircraft) {
            message_len_bytes = WriteAircraftJSONModeSAircraftStr(message, *mode_s_aircraft);
        } else if (UATAircraft* uat_aircraft = get_if<UATAircraft>(&(itr->second)); uat_aircraft) {
            message_len_bytes = WriteAircraftJSONUATAircraftStr(message, *uat_aircraft);
        } else if (RemoteIDAircraft* remote_id_aircraft = get_if<RemoteIDAircraft>(&(itr->second)); remote_id_aircraft) {
            message_len_bytes = WriteAircraftJSONRemoteIDAircraftStr(message, *remote_id_aircraft);
        } else {
            CONSOLE_WARNING("CommsManager::ReportAircraftJSON", "Unknown aircraft type in dictionary for UID 0x%lx.",
                            uid);
            continue;
        }

        if (message_len_bytes < 0) {
            // The AircraftJSON writers detect truncation themselves and return -1 for a buffer overrun.
            CONSOLE_ERROR("CommsManager::ReportAircraftJSON",
                          "Error in WriteAircraftJSON*Str for UID 0x%lx, error code %d.", uid, message_len_bytes);
            ret = false;
            continue;
        }

        ret &= SendBufToSinks(sinks, num_sinks, message, message_len_bytes, aircraftjson_tally, 1, uid);
    }

    // End of the round: flush the accumulated failures from every chunk as one line.
    FLUSH_REPORT_TALLY("CommsManager::ReportAircraftJSON", aircraftjson_tally);
    aircraftjson_round_active_ = false;
    return ret;
}

bool CommsManager::ReportGDL90Uplink(ReportSink* sinks, uint16_t num_sinks, const CompositeArray::RawPackets& packets) {
    if (num_sinks == 0) {
        return true;  // Nobody is listening; don't spend time building frames.
    }
    // Same guard ReportRaw and ReportBeast use: the loop below trusts header->num_uat_uplink_packets.
    char error_msg[CompositeArray::RawPackets::kErrorMessageMaxLen] = {0};
    if (!packets.IsValid(error_msg)) {
        CONSOLE_ERROR("CommsManager::ReportGDL90Uplink", "Invalid CompositeArray::RawPackets: %s", error_msg);
        return false;
    }
    bool ret = true;

    for (uint16_t i = 0; i < packets.header->num_uat_uplink_packets; i++) {
        // GDL90 carries just the de-interleaved 432-byte payload. Raw uplink packets that reach the reporting queues
        // have already been FEC-corrected in place by the decoder (RS-capable platforms) or arrived pre-corrected
        // (everyone else), so a plain de-interleave is all that's needed here -- no second RS pass.
        const RawUATUplinkPacket& raw = packets.uat_uplink_packets[i];
        if (raw.encoded_message_len_bytes != RawUATUplinkPacket::kUplinkMessageNumBytes) {
            CONSOLE_WARNING("CommsManager::ReportGDL90Uplink", "Invalid UAT uplink packet length %d, skipping.",
                            raw.encoded_message_len_bytes);
            continue;
        }
        uint8_t payload[DecodedUATUplinkPacket::kDecodedPayloadNumBytes];
        UATReedSolomon::DeInterleaveUplinkMessage(payload, raw.encoded_message);

        uint8_t buf[GDL90Reporter::kGDL90MessageMaxLenBytes];
        uint16_t msg_len = gdl90.WriteGDL90UplinkDataMessage(
            buf, sizeof(buf), payload, DecodedUATUplinkPacket::kDecodedPayloadNumBytes,
            GDL90Reporter::MLAT48MHz64BitCountsToUATTORTicks(raw.mlat_48mhz_64bit_counts));
        if (msg_len == 0) {
            // Oversize payload; WriteGDL90UplinkDataMessage logs the reason itself.
            gdl90_uplink_tally.num_build_failures++;
            ret = false;
            continue;
        }

        ret &= SendBufToSinks(sinks, num_sinks, (char*)buf, msg_len, gdl90_uplink_tally);
    }
    FLUSH_REPORT_TALLY_RATE_LIMITED("CommsManager::ReportGDL90Uplink", gdl90_uplink_tally);
    return ret;
}

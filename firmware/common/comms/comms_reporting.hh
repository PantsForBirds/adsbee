/**
 * This file is intended to be included as a fragment within the public portion of the CommsManager class definition in
 * comms.hh, allowing reporting function definitions to be used across platforms.
 */

static constexpr uint32_t kRawReportingCheckIntervalMs = 50;  // Check if we need to report packets internally at 20Hz.
static constexpr uint32_t kRawReportingMaxIntervalMs =
    200;  // Don't allow more than 200ms to go by without reporting (5Hz).
static constexpr uint32_t kMAVLINKReportingIntervalMs = 1000;
static constexpr uint32_t kCSBeeReportingIntervalMs = 1000;
static constexpr uint32_t kGDL90ReportingIntervalMs = 1000;
static constexpr uint32_t kAircraftJSONReportingIntervalMs = 1000;
static constexpr uint32_t kCSBeeChunkBudgetMs = 50;        // Max wall time per ReportCSBee invocation.
static constexpr uint32_t kMAVLINKChunkBudgetMs = 50;      // Max wall time per ReportMAVLINK invocation.
static constexpr uint32_t kGDL90ChunkBudgetMs = 50;        // Max wall time per ReportGDL90 invocation.
static constexpr uint32_t kAircraftJSONChunkBudgetMs = 50;  // Max wall time per ReportAircraftJSON invocation.
// Shared UID snapshot capacity — one array serves all periodic reporting protocols.
static constexpr uint16_t kMaxReportUIDs = kAircraftDictionaryMaxNumAircraft;

/**
 * Reporting interfaces are different on different platforms. This union allows passing either a SerialInterface enum
 * value or a feed index to the reporting functions, which will then interpret the value appropriately based on the
 * platform.
 */
typedef uint16_t ReportSink;

// Minimum gap between failure summaries on the raw packet path, which runs at up to 20Hz and has no
// round boundary to flush on.
static constexpr uint32_t kSendFailureLogIntervalMs = 1000;

/**
 * Per-function failure tally covering one reporting round (or one rate limit window on the raw path).
 * A CONSOLE_* call can trigger a synchronous SPI transaction to the coprocessor, which is far too
 * expensive to do once per aircraft per sink inside a chunk budget. The report loops count failures
 * here instead, and emit a single summary line when the round ends.
 */
struct ReportFailureTally {
    uint16_t num_send_failures = 0;
    uint16_t num_build_failures = 0;  // Frames dropped before send (builder returned 0 or overran).
    uint16_t first_failed_sink = 0;
    uint32_t first_failed_uid = 0;  // 0 when the failure isn't tied to a specific aircraft.
    uint32_t last_logged_timestamp_ms = 0;
};

/**
 * Sends a buffer to every sink, recording the first failure in tally instead of logging it inline.
 * @param[in] sinks Array of ReportSinks to send the buffer on.
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * @param[in] buf Buffer to send.
 * @param[in] buf_len Number of bytes in buf.
 * @param[in,out] tally Tally to record failures into.
 * @param[in] num_msgs Number of report frames batched into buf, used for feed statistics.
 * @param[in] uid Aircraft UID associated with this send, or 0 if it isn't aircraft specific.
 * @retval True if every send succeeded.
 */
bool SendBufToSinks(const ReportSink* sinks, uint16_t num_sinks, const char* buf, uint16_t buf_len,
                    ReportFailureTally& tally, uint16_t num_msgs = 1, uint32_t uid = 0);

// Reporting Functions
bool UpdateReporting(const ReportSink* sinks, const SettingsManager::ReportingProtocol* sink_protocols,
                     uint16_t num_sinks, const CompositeArray::RawPackets* packets_to_report = nullptr);

/**
 * Sends out RAW formatted transponder data on the selected serial interface.
 * @param[in] sinks Array of ReportSinks to broadcast RAW messages on.
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * @param[in] packets CompositeArray::RawPackets struct with counts and pointers to arrays of each kind of packet to
 * report.
 * @retval True if successful, false if something broke.
 */
bool ReportRaw(ReportSink* sinks, uint16_t num_sinks, const CompositeArray::RawPackets& packets);

/**
 * Sends out Mode S Beast formatted transponder data on the selected serial interface. Reports all transponder
 * packets in the provided packets_to_report array, which is used to allow printing arbitrary blocks of transponder
 * packets received via the CommsManager's built-in transponder_packet_reporting_queue_.
 * @param[in] sinks Array of ReportSinks to broadcast Mode S Beast messages on.
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * @param[in] packets CompositeArray::RawPackets struct with counts and pointers to arrays of each kind of packet to
 * report.
 * @param[in] protocol Reporting protocol to use. Must be one of kBeast, kBeastNoUAT, or kBeastNoUATUplink.
 * @retval True if successful, false if something broke.
 */
bool ReportBeast(ReportSink* sinks, uint16_t num_sinks, const CompositeArray::RawPackets& packets,
                 SettingsManager::ReportingProtocol protocol = SettingsManager::kBeast);

/**
 * Sends out comma separated aircraft information for each aircraft in the aircraft dictionary.
 * @param[in] sinks Array of ReportSinks to broadcast aircraft information on.
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * @retval True if successful, false if something broke.
 */
bool ReportCSBee(ReportSink* sinks, uint16_t num_sinks);

/**
 * Sends a series of MAVLINK ADSB_VEHICLE messages on the selected serial interface, one for each tracked aircraft
 * in the aircraft dictionary, plus a MAVLINK MESSAGE_INTERVAL message used as a delimiter at the end of the train
 * of ADSB_VEHICLE messages.
 *
 * @param[in] sinks Array of ReportSinks to broadcast MAVLINK messages on. Note that this gets cast to a
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * MAVLINK channel as a bit of a dirty hack under the hood, then un-cast back into a SerialInterface in the UART send
 * function within MAVLINK. Shhhhhhh it's fine for now.
 * @param[in] mavlink_version MAVLINK protocol version to use (1 or 2).
 * @retval True if successful, false if something went sideways.
 */
bool ReportMAVLINK(ReportSink* sinks, uint16_t num_sinks, uint8_t mavlink_version = 2);

/**
 * Reports the contents of the aircraft dictionary using the Garmin GDL90 protocol.
 * @param[in] sinks Array of ReportSinks to broadcast GDL90 messages on.
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * @retval True if successful, false if something broke.
 */
bool ReportGDL90(ReportSink* sinks, uint16_t num_sinks);

/**
 * Reports a GDL90 UAT uplink message on the selected serial interface(s). This is a separate function since GDL90 only
 * reports message frames for uplink messages, everything else is a digested summary of the aircraft dictionary.
 * reporting loop.
 * @param[in] sinks Array of ReportSinks to broadcast GDL90 UAT uplink messages on.
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * @param[in] packets CompositeArray::RawPackets struct. Only UAT uplink messages will be cherry picked out, since GDL90
 * doesn't send raw ADSB packets for Mode S or UAT.
 * @retval True if successful, false if something broke.
 */
bool ReportGDL90Uplink(ReportSink* sinks, uint16_t num_sinks, const CompositeArray::RawPackets& packets);

/**
 * Reports the contents of the aircraft dictionary as newline-delimited readsb-compatible JSON objects,
 * one object per aircraft.
 * @param[in] sinks Array of ReportSinks to broadcast JSON messages on.
 * @param[in] num_sinks Number of ReportSinks in the sinks array.
 * @retval True if successful, false if something broke.
 */
bool ReportAircraftJSON(ReportSink* sinks, uint16_t num_sinks);

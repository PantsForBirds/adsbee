#include "sub_ghz_radio.hh"

#include "hal.hh"
#include "macros.hh"
#include "pico.hh"  // For LED blinks.
#include "uat_packet_decoder.hh"
#include "unit_conversions.hh"  // for CeilBitsToBytes

extern "C" {
#include "ti/devices/cc13x2_cc26x2/driverlib/rf_prop_mailbox.h"  // For status codes.
}

static const uint32_t kRxRestartTimeoutMs = 5000;

static const uint16_t kNumPartialDataEntries = SubGHzRadio::kRxPacketQueueLen;
static const uint16_t kPartialDataEntryHeaderSizeBytes = 12;
static const uint16_t kPartialDataEntryNumAppendedBytes = 6;  // Appended Bytes determined by the settings below.
// Headroom beyond the largest packet (a 552-byte UAT uplink). Without it the entry is an EXACT fit:
// the stored bytes (len word + header + payload + appended) equal the entry's usable capacity, so
// "packet ended" and "element reached the end of the entry" (TRM 26.3.2.7.4) land on the same byte,
// and the radio CPU resolves that collision by ending CMD_PROP_RX_ADV with PROP_ERROR_PAR (0x3800,
// "observed illegal parameter") on every uplink. The packet still delivers (RX_OK fires first), but
// each uplink silently ended the RX command with an error and forced a restart.
static const uint16_t kPartialDataEntryPayloadHeadroomBytes = 16;
static const uint16_t kPartialDataEntryPayloadSizeBytes =
    SubGHzRadio::kRxPacketMaxLenBytes + kPartialDataEntryPayloadHeadroomBytes;
static const uint16_t kPartialDataEntryPayloadLenSzBytes = 2;  // Length field size in bytes.

static const uint16_t kPacketHeaderSizeBytes = 1;  // Equivalent to CeilBitsToBytes(RF_cmdPropRxAdv.hdrConf.numHdrBits);

// Entry stride, rounded up to word alignment: TI's queue layout requires every data entry to start
// word aligned (RFQueue.h pads each entry the same way); rfc_dataEntryPartial_t is aligned(4).
static const uint16_t kPartialDataEntryUnpaddedSizeBytes =
    sizeof(rfc_dataEntryPartial_t) + kPartialDataEntryPayloadLenSzBytes + kPacketHeaderSizeBytes +
    kPartialDataEntryPayloadSizeBytes + kPartialDataEntryNumAppendedBytes;
static const uint16_t kPartialDataEntrySizeBytes = (kPartialDataEntryUnpaddedSizeBytes + 3) & ~uint16_t{3};
static_assert(kPartialDataEntrySizeBytes % 4 == 0, "Partial data entry stride must be word aligned.");

static const RF_EventMask kRxEventMask =
    RF_EventMdmSoft | RF_EventNDataWritten | (RF_EventRxEntryDone | RF_EventRxOk | RF_EventRxNOk) |
    (RF_EventLastCmdDone | RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdStopped);

static volatile int16_t current_packet_len_bytes = INT16_MIN;

// Packet RX Configuration
//  ------------------------------------------------------------------------------------------------------------------------
//  | Partial Data Entry Header | Payload Length (lenSz Bytes) | Payload Bytes | Optional Appended Bytes
//  ------------------------------------------------------------------------------------------------------------------------

// Optional appended bytes
//  -------------------------------------------------------
//  | CRC1 | CRC0 | RSSI | TS0 | TS1 | TS2 | TS3 | Status |
//  -------------------------------------------------------
static uint8_t rx_data_entry_buf[kNumPartialDataEntries * kPartialDataEntrySizeBytes] __attribute__((aligned(4)));
static rfc_dataEntryPartial_t* partial_data_entry_ptrs[kNumPartialDataEntries] = {0};

// This is the entry that we are currently allowed to read from (not being used by the RF core).
static rfc_dataEntryPartial_t* current_read_entry;

static dataQueue_t rx_data_queue;

inline void RollDataQueue() {
    current_read_entry->status = DATA_ENTRY_PENDING;
    current_read_entry = (rfc_dataEntryPartial_t*)(((rfc_dataEntryPartial_t*)current_read_entry)->pNextEntry);
    rx_data_queue.pCurrEntry = (uint8_t*)current_read_entry;
}

// Technical Reference Manual Section 23.7.5.4: https://www.ti.com/lit/ug/swcu117i/swcu117i.pdf
void rf_cmd_callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e) {
    // CONSOLE_INFO("SubGHzRadio", "RF command completed with handle %p, command handle %d, event mask %u", h, ch, e);

    if (e & RF_EventMdmSoft) {
        // Preamble detected. Get ready to figure out packet length!
        current_packet_len_bytes = INT16_MIN;
    }

    if (e & RF_EventNDataWritten) {
        if (current_packet_len_bytes < 0) {
            // Only set the length once per packet (irqIntv=0 fires at 16 bytes; rxData[2] and [3] are ready).
            uint8_t sync_word_ls4 = (&(current_read_entry->rxData))[kPartialDataEntryPayloadLenSzBytes] &
                                    0x0F;  // Last 4 bits of sync word are the first 4 bits of the payload.
            uint8_t mdb_type_code = (&(current_read_entry->rxData))[kPartialDataEntryPayloadLenSzBytes + 1] >> 3;
            current_packet_len_bytes =
                UATPacketDecoder::SyncWordLSBAndMDBTypeCodeToMessageLenBytes(sync_word_ls4, mdb_type_code);

            rfc_CMD_PROP_SET_LEN_t RF_cmdPropSetLen = {.commandNo = CMD_PROP_SET_LEN,
                                                       .rxLen = (uint16_t)(current_packet_len_bytes)};
            // A failed SET_LEN (e.g. CMDSTA ContextError from arriving after the RX command ended)
            // would otherwise be invisible; count it (no console in ISR context).
            if (RF_runImmediateCmd(h, (uint32_t*)&RF_cmdPropSetLen) != RF_StatCmdDoneSuccess) {
                subg_radio.set_len_error_count = subg_radio.set_len_error_count + 1;
            }
        }
    }

    if (e & RF_EventRxOk) {
        subg_radio.HandlePacketRx(current_read_entry);
    }

    if (e & (RF_EventLastCmdDone | RF_EventCmdAborted | RF_EventRxBufFull)) {
        RollDataQueue();
    }
}

bool SubGHzRadio::Init() {
    RF_Params rf_params;
    RF_Params_init(&rf_params);

    // Overriding settings from smartrf/smartrf_settings.c.
    /* RF_cmdPropRxAdv  - Advanced Rx command settings. */
    RF_cmdPropRxAdv.pQueue = &rx_data_queue;
    RF_cmdPropRxAdv.pOutput = (uint8_t*)&rx_statistics_;
    RF_cmdPropRxAdv.rxConf = {
        .bAutoFlushIgnored = 0,  // If 1, automatically discard ignored packets from RX queue.
        .bAutoFlushCrcErr = 0,   // If 1, automatically discard packets with CRC error from RX queue.
        // bIncludeHdr must be 1 to receive the first byte after sync in the data entry. For UAT this is the last byte
        // of the full 36 bit sync word.
        .bIncludeHdr =
            1,  // If 1, include the received header or length byte in the stored packet; otherwise discard it.
        .bIncludeCrc = 0,       // 2 bytes (if default CRC is used)
        .bAppendRssi = 1,       // 1 byte
        .bAppendTimestamp = 1,  // 4 bytes
        .bAppendStatus = 1      // 1 Byte
    };
    // RF_cmdPropRxAdv.pktConf.bRepeatOk = 0;  // Go back to sync search after receiving a packet correctly.
    // RF_cmdPropRxAdv.pktConf.bRepeatNok = 0; // Go back to sync search after receiving a packet incorrectly.
    // Sync on the most significant 32 bits of the 36 bit sync word. Save the last 4 bits of sync for discrimination
    // between uplink and ADSB packets in the payload. RF_cmdPropRx.syncWord = RawUATADSBPacket::kSyncWordMS32;
    RF_cmdPropRxAdv.syncWord0 = RawUATADSBPacket::kSyncWordMS32;
    RF_cmdPropRxAdv.syncWord1 = RawUATUplinkPacket::kSyncWordMS32;

    // Must be 0 (unlimited) so that CMD_PROP_SET_LEN (called in RF_EventNDataWritten) can shorten reception for
    // ADSB packets instead of always receiving the full 552-byte uplink length. If the RF callback is preempted
    // before CMD_PROP_SET_LEN fires, the RF core keeps writing until the data entry is full (kRxPacketMaxLenBytes),
    // at which point PROP_ERROR_RXFULL fires — not a memory stomp. RF software interrupt priority must be higher
    // than SPI priority; 1 point higher works well, higher values cause instability.
    RF_cmdPropRxAdv.maxPktLen = 0;

    // The last 4 bits of the Sync word are interpreted as the header, so the rest of the packet stays byte-aligned.
    RF_cmdPropRxAdv.hdrConf = {.numHdrBits = 4, .lenPos = 0, .numLenBits = 0};

    // No address field.
    RF_cmdPropRxAdv.addrConf = {0};

    /* RF_cmdPropRadioDivSetup - Radio setup command settings. */
    RF_cmdPropRadioDivSetup.formatConf.nSwBits =
        32;  // Use a 32-bit sync word to allow the last 4 bits of the sync to be used for format discrimination.

    // Set up the data queue.
    for (uint16_t i = 0; i < kNumPartialDataEntries; i++) {
        // Fill out the addresses first so that our pNextEntry assignment works in the next loop.
        partial_data_entry_ptrs[i] = (rfc_dataEntryPartial_t*)&rx_data_entry_buf[i * kPartialDataEntrySizeBytes];
    }

    for (uint16_t i = 0; i < kNumPartialDataEntries; i++) {
        rfc_dataEntryPartial_t* entry = partial_data_entry_ptrs[i];
        entry->length = sizeof(rfc_dataEntryPartial_t::pktStatus) + sizeof(rfc_dataEntryPartial_t::nextIndex) +
                        kPacketHeaderSizeBytes + kPartialDataEntryPayloadLenSzBytes +
                        kPartialDataEntryPayloadSizeBytes + kPartialDataEntryNumAppendedBytes;
        entry->config.irqIntv = 0;  // Set interrupt interval. We only need 1 byte after the header, increase to reduce
                                    // interrupt frequency. 0=16
        entry->config.type = DATA_ENTRY_TYPE_PARTIAL;
        entry->config.lenSz = kPartialDataEntryPayloadLenSzBytes;
        entry->status = DATA_ENTRY_PENDING;
        entry->pNextEntry = (uint8_t*)partial_data_entry_ptrs[(i + 1) % kNumPartialDataEntries];  // Circular queue.

        // Sentinel values for debuging.
        memset(&(entry->rxData) + kPartialDataEntryPayloadLenSzBytes, 0xAA + i,
               kPartialDataEntryPayloadSizeBytes + kPartialDataEntryNumAppendedBytes);
    }
    rx_data_queue.pCurrEntry = (uint8_t*)partial_data_entry_ptrs[0];
    rx_data_queue.pLastEntry = nullptr;

    current_read_entry = (rfc_dataEntryPartial_t*)(rx_data_queue.pCurrEntry);

    rf_handle_ = RF_open(&rf_object_, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rf_params);
    if (!rf_handle_) {
        CONSOLE_ERROR("SubGHzRadio::Init", "Failed to open RF device.");
        return false;  // Failed to open RF handle.
    }

    RF_EventMask ret = RF_runCmd(rf_handle_, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, nullptr, 0);
    if (ret != RF_EventLastCmdDone) {
        CONSOLE_ERROR("SubGHzRadio::Init", "Failed to set frequency.");
        RF_close(rf_handle_);
        return false;  // Failed to run command FS.
    }

    return StartPacketRx();
}

bool SubGHzRadio::DeInit() {
    RF_close(rf_handle_);
    return true;
}

bool SubGHzRadio::HandlePacketRx(rfc_dataEntryPartial_t* filled_entry) {
    /* Handle the packet data, located at &currentDataEntry->data:
     * - Length is the first two bytes with the current configuration
     * - Data starts from the second byte */
    // rfc_dataEntryPartial_t *filled_entry = (rfc_dataEntryPartial_t *)rx_data_queue.pLastEntry;
    uint8_t* buf = (uint8_t*)(&filled_entry->rxData);
    uint16_t rx_data_bytes_after_len =
        buf[0] | (buf[1] << 8);  // 2-Byte length is packaged LSB first (little endian). Includes appended bytes.
    uint16_t packet_len_bytes = rx_data_bytes_after_len - kPartialDataEntryNumAppendedBytes - kPacketHeaderSizeBytes;
    // Note that filled_entry->nextIndex should == packet_len_bytes + kPartialDataEntryNumAppendedBytes.
    uint8_t packet_sync_ls4 = buf[2] & 0x0F;  // Last 4 bits of sync word are the first 4 bits of the payload.
    uint8_t* packet_data = buf + kPartialDataEntryPayloadLenSzBytes + kPacketHeaderSizeBytes;

    if (packet_len_bytes > kRxPacketMaxLenBytes || packet_len_bytes == 0) {
        return false;
    }

    // Appended Bytes (no CRC)
    //------------------------------------------
    // | RSSI | TS0 | TS1 | TS2 | TS3 | Status |
    //------------------------------------------
    int8_t rssi_dbm = static_cast<int8_t>(packet_data[packet_len_bytes]);
    uint32_t timestamp = packet_data[packet_len_bytes + 1] | (packet_data[packet_len_bytes + 2] << 8) |
                         (packet_data[packet_len_bytes + 3] << 16) | (packet_data[packet_len_bytes + 4] << 24);
    int8_t status = static_cast<int8_t>(packet_data[packet_len_bytes + kPartialDataEntryNumAppendedBytes - 1]);
    static_assert(kPartialDataEntryNumAppendedBytes == 6);  // This section is hardcoded to match the appended bytes.

    switch (current_packet_len_bytes) {
        case RawUATADSBPacket::kShortADSBMessageNumBytes:
        case RawUATADSBPacket::kLongADSBMessageNumBytes: {
            RawUATADSBPacket raw_packet = RawUATADSBPacket(packet_data, current_packet_len_bytes);
            raw_packet.sigs_dbm = rssi_dbm;
            raw_packet.mlat_48mhz_64bit_counts = timestamp;

            object_dictionary.metrics.num_raw_uat_adsb_packets++;
            uat_packet_decoder.raw_uat_adsb_packet_queue.Enqueue(raw_packet);
            break;
        }
        case RawUATUplinkPacket::kUplinkMessageNumBytes: {
            RawUATUplinkPacket raw_packet = RawUATUplinkPacket(packet_data, current_packet_len_bytes);
            raw_packet.sigs_dbm = rssi_dbm;
            raw_packet.mlat_48mhz_64bit_counts = timestamp;

            object_dictionary.metrics.num_raw_uat_uplink_packets++;
            uat_packet_decoder.raw_uat_uplink_packet_queue.Enqueue(raw_packet);
            break;
        }
        default: {
            // Unrecognized packet length, skip. Don't print here since it's in the interrupt context.
            return false;
        }
    }

    return true;
}

bool SubGHzRadio::StartPacketRx() {
    last_rx_start_timestamp_ms_ = get_time_since_boot_ms();
    rx_cmd_handle_ = RF_postCmd(rf_handle_, (RF_Op*)&RF_cmdPropRxAdv, RF_PriorityNormal, rf_cmd_callback, kRxEventMask);
    if (rx_cmd_handle_ < 0) {
        CONSOLE_ERROR("SubGHzRadio::Init", "Failed to post command PROP_RX: returned code %d.", rx_cmd_handle_);
        return false;  // Failed to post command PROP_RX.
    }
    return true;
}

bool SubGHzRadio::Update() {
    if (!rx_enabled) {
        return true;
    }

    volatile uint16_t rx_status = ((volatile RF_Op*)&RF_cmdPropRxAdv)->status;
    // Compare the status class (high byte) exactly: PROP_DONE_* = 0x34xx, PROP_ERROR_* = 0x38xx. Masking with '&'
    // instead of '==' made every error status match the PROP_DONE_OK branch (0x3800 & 0x3400 != 0), silently
    // restarting RX without logging the error.
    if ((rx_status & 0xFF00) == (PROP_DONE_OK & 0xFF00)) {
        StartPacketRx();
    } else if ((rx_status & 0xFF00) == (PROP_ERROR_PAR & 0xFF00)) {
        char* error_str;
        switch (rx_status) {
            case PROP_ERROR_PAR:
                error_str = (char*)"Illegal parameter";
                break;
            case PROP_ERROR_RXBUF:
                error_str = (char*)"No available Rx buffer at the start of a packet";
                break;
            case PROP_ERROR_RXFULL:
                error_str = (char*)"Out of Rx buffer during reception in a partial read buffer";
                break;
            case PROP_ERROR_NO_SETUP:
                error_str = (char*)"Radio was not set up in proprietary mode";
                break;
            case PROP_ERROR_NO_FS:
                error_str = (char*)"Synth was not programmed when running Rx or Tx";
                break;
            case PROP_ERROR_RXOVF:
                error_str = (char*)"Rx overflow observed during operation";
                break;
            case PROP_ERROR_TXUNF:
                error_str = (char*)"Tx underflow observed during operation";
                break;
            default:
                error_str = (char*)"Unknown error";
        }
        CONSOLE_ERROR("SubGHzRadio::Update", "PROP_RX threw error (0x%x): %s. Restarting.", rx_status, error_str);
        StartPacketRx();
    }

    return true;
}
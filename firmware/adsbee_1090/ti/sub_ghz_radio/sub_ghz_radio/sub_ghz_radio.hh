#pragma once

extern "C" {
#include <ti/drivers/GPIO.h>
#include <ti/drivers/rf/RF.h>
#include <ti_radio_config.h>

#include "smartrf/smartrf_settings.h"

/* clang-format off */
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_data_entry.h)
/** clang-format on */
}

#include "bsp.hh"
// #include "buffer_utils.hh"
#include "comms.hh"
#include "mlat_time_sync.hh"
#include "object_dictionary.hh"
#include "unit_conversions.hh"
#include "uat_packet.hh"

class SubGHzRadio
{
public:
    static const uint16_t kRxPacketQueueLen = 2;
    static const uint16_t kRxPacketMaxLenBytes = RawUATUplinkPacket::kUplinkMessageNumBytes;

    struct SubGHzRadioConfig
    {
        // Add any configuration parameters needed for the SubGHzRadio.
    };

    SubGHzRadio(SubGHzRadioConfig config_in): config_(config_in) {};
    ~SubGHzRadio() {};

    /**
     * Initializes the radio and begins packet reception.
     */
    bool Init();

    /**
     * De-initializes the radio.
     */
    bool DeInit();

    /**
     * Enables or disables the bias tee.
     * @param enabled True to enable the bias tee, false to disable it.
     */
    inline void SetBiasTeeEnable(bool enabled) {
        // Bias tee is active LO because it's a PMOS device.
        CONSOLE_INFO("SubGHzRadio::SetBiasTeeEnable", "Bias tee %s", enabled ? "ENABLED" : "DISABLED");
        GPIO_write(bsp.kSubGBiasTeeEnablePin, !enabled);
    }
    /**
     * Begin the proprietary RF receive command.
     * @retval True if command was posted successfully, false otherwise.
     */
    bool StartPacketRx();

    /**
     * Restarts packet Rx after a receive command that ended with an error, and logs the error.
     * @retval True if update succeeded, false otherwise.
     */
    bool Update();

    /**
     * Called from the RF callback when the receive command has ended and the data queue points at a free entry. Posts
     * the next receive command, or leaves the restart to Update() if the command ended with an error.
     */
    void HandleRxCommandEnd();

    /**
     * Handles a received packet from the RF core.
     * @param filled_entry Pointer to the filled data entry from the RF core.
     * @retval True if packet was handled successfully, false otherwise.
     */
    bool HandlePacketRx(rfc_dataEntryPartial_t *filled_entry);

    bool rx_enabled = true;

    // Failed CMD_PROP_SET_LEN immediate commands (RF_runImmediateCmd status != success), counted from
    // the RF callback (ISR context, no logging there). A nonzero value means packet-length shortening
    // raced the RX command ending -- packets may be truncated or over-read.
    volatile uint32_t set_len_error_count = 0;

    // Receptions where current_packet_len_bytes disagreed with the data entry's length (stale after coalesced RF
    // events). Diagnostic only. Counted from the RF callback (software interrupt context).
    volatile uint32_t uat_len_mismatch_count = 0;

    /**
     * Sub-GHz packet timestamps go out on the RP2040's 48 MHz MLAT clock. The RP2040 pulses the SYNC line (DIO5) once a
     * second and then writes the pulse's MLAT count (kAddrSyncPulse). The radio timer (RAT) captures each rising
     * edge; pairing the two gives a sync point for time_sync_.
     */
    // Pairs a SyncPulse from the RP2040 with the last captured edge. Main loop only.
    void HandleSyncPulse(const ObjectDictionary::SyncPulse& pulse);
    // Called from the RAT capture callback.
    void HandleSyncCapture(uint32_t rat) {
        sync_capture_rat_ = rat;
        sync_capture_count_ = sync_capture_count_ + 1;
    }
    // ObjectDictionary::MLATSyncFlag bits for the device status.
    uint8_t GetMLATSyncFlags() const;

private:
    // Routes the sync pin to the radio timer and captures its rising edges. The RAT restarts with the RF client, so
    // this also restarts the fit.
    void StartSyncCapture();
    // Frees the capture channel. Call before RF_close().
    void StopSyncCapture();
    RF_RatHandle sync_capture_handle_ = RF_ALLOC_ERROR;

    // Two crystals (CC1312 and RP2040): timestamps start once a second sync point has measured the rate. Read by the RF
    // callback, so changed with interrupts off.
    MLATTimeSync time_sync_ = MLATTimeSync(MLATTimeSync::kMLATTicksPerRATTick, 2);
    volatile uint32_t sync_capture_rat_ = 0;
    volatile uint16_t sync_capture_count_ = 0;
    SyncPulsePairing sync_pulse_pairing_;
    uint32_t last_sync_point_timestamp_ms_ = 0;
    uint32_t rat_start_timestamp_ms_ = 0;

    SubGHzRadioConfig config_;

    RF_Handle rf_handle_;
    RF_Object rf_object_;

    rfc_dataEntryPartial_t *current_data_entry_; // Pointer to the data entry being processed (not held by the RF core).
    rfc_propRxOutput_t rx_statistics_;

    RF_CmdHandle rx_cmd_handle_;
    // Status of a receive command that Update() has to restart, 0 if none. Written by the RF callback.
    volatile uint16_t rx_restart_status_ = 0;

    uint32_t last_rx_start_timestamp_ms_ = 0;
};

extern SubGHzRadio subg_radio;
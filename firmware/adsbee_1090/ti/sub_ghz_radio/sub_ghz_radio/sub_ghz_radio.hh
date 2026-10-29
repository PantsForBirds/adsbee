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
#include "adsl_packet.hh"
#include "comms.hh"
#include "settings.hh"
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

    /**
     * Requests a Sub-GHz protocol mode (AT+SUBG_MODE). Safe from the SPI callback: Update() re-opens the RF client in
     * the new mode from the main loop. Out-of-range values fall back to UAT_RX.
     */
    void SetMode(SettingsManager::SubGHzRadioMode mode);
    SettingsManager::SubGHzRadioMode GetMode() const { return mode_; }
    static bool ModeIsADSL(SettingsManager::SubGHzRadioMode mode) {
        return mode == SettingsManager::kSubGHzRadioModeADSLRx || mode == SettingsManager::kSubGHzRadioModeADSLRx868_4MHz;
    }

    bool rx_enabled = true;

    // Failed CMD_PROP_SET_LEN immediate commands (RF_runImmediateCmd status != success), counted from
    // the RF callback (ISR context, no logging there). A nonzero value means packet-length shortening
    // raced the RX command ending -- packets may be truncated or over-read.
    volatile uint32_t set_len_error_count = 0;

    // Receptions where current_packet_len_bytes disagreed with the data entry's length (stale after coalesced RF
    // events). Diagnostic only. Counted from the RF callback (software interrupt context).
    volatile uint32_t uat_len_mismatch_count = 0;

private:
    SubGHzRadioConfig config_;

    /**
     * Applies a mode requested with SetMode(): aborts reception and re-opens the RF client with the mode's PHY.
     */
    bool ApplyRequestedMode();

    RF_Handle rf_handle_ = nullptr;  // nullptr while the RF client is closed.
    RF_Object rf_object_;

    SettingsManager::SubGHzRadioMode mode_ = SettingsManager::kSubGHzRadioModeUATRx;
    volatile SettingsManager::SubGHzRadioMode requested_mode_ = SettingsManager::kSubGHzRadioModeUATRx;

    rfc_dataEntryPartial_t *current_data_entry_; // Pointer to the data entry being processed (not held by the RF core).
    rfc_propRxOutput_t rx_statistics_;

    RF_CmdHandle rx_cmd_handle_;
    // Status of a receive command that Update() has to restart, 0 if none. Written by the RF callback.
    volatile uint16_t rx_restart_status_ = 0;

    uint32_t last_rx_start_timestamp_ms_ = 0;
};

extern SubGHzRadio subg_radio;
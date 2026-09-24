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
#include "settings.hh"
#include "unit_conversions.hh"
#include "uat_packet.hh"

class SubGHzRadio
{
public:
    // Partial data entries in the RF core's circular RX queue. Sized so back-to-back receptions don't
    // exhaust entries (PROP_ERROR_RXBUF) while the main loop is busy elsewhere; each entry is
    // ~(kRxPacketMaxLenBytes + overhead) bytes of static RAM.
    static const uint16_t kRxPacketQueueLen = 4;
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
        // No bias tee pin on CC1314 hardware.
        CONSOLE_INFO("SubGHzRadio::SetBiasTeeEnable", "Bias tee %s (no-op on CC1314)", enabled ? "ENABLED" : "DISABLED");
    }
    /**
     * Begin the proprietary RF receive command.
     * @retval True if command was posted successfully, false otherwise.
     */
    bool StartPacketRx();

    /**
     * Starts a packet Rx if it hasn't been started and we are currently supposed to be receiving, plus some other housekeeping.
     * @retval True if update succeeded, false otherwise.
     */
    bool Update();

    /**
     * Quiesces the internal RF core so the MCU can enter STANDBY. Cancels the in-progress RX command
     * and closes the RF client, which releases the PowerCC26XX_DISALLOW_STANDBY constraint the RF core
     * holds while powered. Reuses the known-good DeInit() (RF_close) path. Pair with Resume() on wake.
     * @retval True if the radio was suspended successfully, false otherwise.
     */
    bool Suspend();

    /**
     * Re-establishes UAT reception after a Suspend()/STANDBY cycle by re-running the full Init()
     * (RF_open + CMD_FS + StartPacketRx) restore path.
     * @retval True if the radio resumed successfully, false otherwise.
     */
    bool Resume();

    /**
     * Handles a received packet from the RF core.
     * @param filled_entry Pointer to the filled data entry from the RF core.
     * @retval True if packet was handled successfully, false otherwise.
     */
    bool HandlePacketRx(rfc_dataEntryPartial_t *filled_entry);

    /**
     * Sets the Sub-GHz protocol mode (AT+SUBG_MODE). CMD_PROP_RX_ADV latches its sync words when the command
     * starts, so if normal reception is running and the mode changed, the RX command is aborted and the RF client
     * re-initialized through the same RF_cancelCmd -> DeInit() -> Init() path used by Suspend()/Resume() and
     * StopCWTest()/StopRssiScan(). If the RF client is closed (pre-Init, suspended, user-disabled) or a CW/RSSI test
     * owns the RF command, the mode is stored and applied by the next Init(). Main-loop context only.
     * @param mode Mode to apply. Out-of-range values (stale persisted settings) fall back to UAT_RX.
     * @retval True on success (including no-op), false if the RX restart failed.
     */
    bool SetMode(SettingsManager::SubGHzRadioMode mode);
    SettingsManager::SubGHzRadioMode GetMode() const { return mode_; }

    /**
     * @retval True when the RF core also syncs on the UAT ground-uplink sync word (RF_cmdPropRxAdv.syncWord1).
     */
    bool UplinkRxEnabled() const { return mode_ == SettingsManager::kSubGHzRadioModeUATRx; }

    /**
     * User enable/disable of the Sub-GHz receiver (AT+RX_ENABLE). Disabling aborts RX and closes the RF client
     * (which also releases its STANDBY power constraint); enabling re-opens it. The requested state survives
     * Suspend()/Resume() and the CW/RSSI test restore paths, which re-open the radio only if it is enabled.
     * Main-loop context only.
     * @param enabled True to receive, false to shut the receiver down.
     * @retval True on success, false if the RF client failed to open/close.
     */
    bool SetRxEnabled(bool enabled);
    bool RxIsEnabled() const { return rx_requested_; }

#ifdef HARDWARE_UNIT_TESTS
    /**
     * Starts an unmodulated continuous-wave (CW) carrier for testing. Cancels any in-progress packet
     * reception and keeps the carrier on until StopCWTest() is called. Debug builds only.
     * @param freq_mhz Carrier frequency in MHz.
     * @retval True if the carrier was started successfully, false otherwise.
     */
    bool StartCWTest(uint32_t freq_mhz);

    /**
     * Stops a CW carrier started with StartCWTest() and resumes normal packet reception. Debug builds only.
     * @retval True if the carrier was stopped and Rx resumed successfully, false otherwise.
     */
    bool StopCWTest();

    /**
     * Retunes the synthesizer to the requested frequency and restarts packet RX so instantaneous RSSI
     * can be polled with ReadRssiDbm(). Suspends normal reception until StopRssiScan() is called.
     * Debug builds only.
     * @param freq_mhz RX frequency in MHz.
     * @retval True if the scan was started successfully, false otherwise.
     */
    bool StartRssiScan(uint32_t freq_mhz);

    /**
     * Reads the instantaneous RSSI from the RF core during a StartRssiScan(). The RF core only reports RSSI while
     * an RX command is running, and the packet-RX command the scan piggybacks on ends whenever the RF core thinks
     * it received a packet or hits an RX buffer error (easy under a strong carrier); Update() won't re-arm it while
     * rx_enabled is false, so this re-posts the RX command itself whenever it finds it ended (counted in
     * rssi_scan_rx_restart_count). Debug builds only.
     * @retval RSSI in dBm, or RF_GET_RSSI_ERROR_VAL (-128) if the RF client is closed or the RX command could not
     * be re-armed.
     */
    int8_t ReadRssiDbm();

    // Number of times ReadRssiDbm() had to re-post the RX command during the current/last RSSI scan. Reset by
    // StartRssiScan(); reported by AT+RX_CW.
    uint32_t rssi_scan_rx_restart_count = 0;

    /**
     * Stops an RSSI scan started with StartRssiScan() and resumes normal packet reception. Debug
     * builds only.
     * @retval True if normal reception was restored successfully, false otherwise.
     */
    bool StopRssiScan();
#endif

    // Live "normal packet RX owns the RF command" flag: false while Suspend()ed, user-disabled, or a CW/RSSI test
    // owns the RF core. Distinct from rx_requested_ (the user's AT+RX_ENABLE intent), which decides whether the
    // restore paths bring RX back up.
    bool rx_enabled = true;

    // Health / diagnostics counter, reported and reset via AT+RX_STATS. Incremented when the RX
    // command ends with an error status (e.g. PROP_ERROR_RXFULL/RXBUF) and has to be restarted.
    uint32_t rx_error_restart_count = 0;

    // Failed CMD_PROP_SET_LEN immediate commands (RF_runImmediateCmd status != success), counted from
    // the RF callback (ISR context, no logging there). Reported and reset via AT+RX_STATS. A nonzero
    // value means packet-length shortening raced the RX command ending -- packets may be truncated or
    // over-read.
    volatile uint32_t set_len_error_count = 0;

    // Receptions where the volatile current_packet_len_bytes (set by the SET_LEN handler) disagreed
    // with the data entry's own length word at HandlePacketRx time -- i.e. the coalesced-event
    // staleness race fired. Dispatch uses the entry-derived length, so this is visibility only.
    // Counted from the RF callback (SWI context). Reported and reset via AT+RX_STATS.
    volatile uint32_t uat_len_mismatch_count = 0;

private:
    /**
     * Brings normal RX back up after Resume()/StopCWTest()/StopRssiScan() if the user has it enabled; otherwise
     * leaves the RF client closed.
     * @retval True on success (including "left closed"), false if Init() failed.
     */
    bool RestoreRx();

    SubGHzRadioConfig config_;

    RF_Handle rf_handle_ = nullptr;  // nullptr whenever the RF client is closed.
    RF_Object rf_object_;

    SettingsManager::SubGHzRadioMode mode_ = SettingsManager::kSubGHzRadioModeUATRx;
    bool rx_requested_ = true;  // User intent (AT+RX_ENABLE); see rx_enabled.

    rfc_dataEntryPartial_t *current_data_entry_; // Pointer to the data entry being processed (not held by the RF core).
    rfc_propRxOutput_t rx_statistics_;

    RF_CmdHandle rx_cmd_handle_;

    uint32_t last_rx_start_timestamp_ms_ = 0;
};

extern SubGHzRadio subg_radio;
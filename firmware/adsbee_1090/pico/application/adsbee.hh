#ifndef _ADS_BEE_HH_
#define _ADS_BEE_HH_

#include "aircraft_dictionary.hh"
#include "bsp.hh"
#include "cc1312.hh"
#include "cpp_at.hh"
#include "data_structures.hh"  // For PFBQueue.
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "macros.hh"  // For MAX / MIN.
#include "main.hh"    // For ISR core assignment.
#include "mode_s_packet.hh"
#include "settings.hh"
#include "spi_coprocessor.hh"
#include "stdint.h"

class ADSBee {
   public:
    static constexpr uint16_t kModeSPacketQueueDepth = 200;
    static constexpr uint16_t kRawUATADSBPacketQueueDepth = 50;
    static constexpr uint16_t kRawUATUplinkPacketQueueDepth = 4;
    // Remote ID packets pulled from the ESP32 over SPI, waiting to be decoded into the aircraft dictionary. The ESP32
    // already rate-limits these to ~1 Hz per drone, so a shallow queue suffices.
    static constexpr uint16_t kRawRemoteIDPacketQueueDepth = 16;

    static constexpr uint16_t kTLMaxPWMCount = 5000;  // Clock is 125MHz, shoot for 25kHz PWM.
    static constexpr int kVDDMV = 3300;               // [mV] Voltage of positive supply rail.
    static constexpr int kTLOffsetMaxMV = 2000;       // [mV]
    static constexpr int kTLOffsetMinMV = 0;          // [mV]
    static constexpr uint32_t kStatusLEDOnMs = 1;

    static constexpr uint32_t kTLLearningIntervalMs =
        10000;  // [ms] Length of Simulated Annealing interval for learning trigger level.
    static constexpr uint16_t kTLLearningNumCycles =
        100;  // Number of simulated annealing cycles for learning trigger level.
    static constexpr uint16_t kTLLearningStartTemperatureMV =
        1000;  // [mV] Starting value for simulated annealing temperature when learning triger level. This corresponds
               // to the maximum value that the trigger level could be moved (up or down) when exploring a neighbor
               // state.

    static constexpr uint16_t kADCReadySpinLimit =
        200;  // Max iterations to wait for a foreign ADC conversion (~2us) to finish before starting the RSSI read.

    static constexpr uint32_t kMLATCounterWrapIntervalMs =
        0xFFFFFF / 48'000;  // [ms] How often a 48MHz 24-bit counter wraps (period in ms).

    /**
     * Noise floor estimation. The RSSI ADC input is sampled from the main loop between packets and low-pass filtered
     * to approximate the receiver noise floor, which anchors the trigger level (noise floor + TL offset) and is the
     * reference for reported packet signal quality (sigq).
     */
    static constexpr uint32_t kNoiseFloorADCSampleIntervalMs =
        1;  // [ms] Minimum interval between RSSI ADC samples used for the noise floor estimate.
    static constexpr uint32_t kNoiseFloorDemodGuardUs =
        200;  // [us] Discard samples taken this soon after a demodulation began: covers the longest packet (~120us)
              // plus the RSSI line's RC decay (10k / 1nF, ~50us) back down to the noise floor.
    static constexpr uint32_t kNoiseFloorMaxHoldMs =
        1000;  // [ms] If no sample has been accepted for this long (receiver constantly demodulating), stop applying
               // kNoiseFloorDemodGuardUs (but still never sample during a demodulation) so the estimate can't stall.
    static constexpr uint16_t kNoiseFloorFilterShift =
        6;  // Low pass filter weight is 1/2^kNoiseFloorFilterShift: time constant of ~64 accepted samples (>=64ms).
    static constexpr uint16_t kNoiseFloorFixedPointShift =
        8;  // Fractional bits of the filter accumulator, so a small filter weight doesn't truncate away sub-mV steps.
    static constexpr int32_t kNoiseFloorOutlierMV =
        150;  // [mV] (~9dB) Samples this far from the current estimate are rejected: above it they are pulses (e.g.
              // packets whose preamble didn't trigger a demodulation), below it glitched conversions...
    static constexpr uint16_t kNoiseFloorMaxConsecutiveOutliers =
        32;  // ...unless this many consecutive samples were rejected, in which case the floor itself moved (e.g. bias
             // tee powered an external LNA on or off) and samples are accepted again so the estimate can track.

    static constexpr uint32_t kSubGRadioFailRebootIntervalMs = 10'000;
    static constexpr uint32_t kSubGRadioFailRebootPowerOffDurationMs = 20;

    /**
     * Interrupt priorities
     * MLAT counter wrap is highest priority because we don't want to capture an MLAT timestamp where a wrap is pending
     * but hasn't happened yet (will result in non-monotonic timestamps). GPIO interrupt is next priority because we
     * want to capture the demodulation start time and RSSI pretty promptly. Demod complete interrupt is lowest priority
     * because a filled state machine can wait a little before being digested.
     */
    static constexpr uint8_t kNoDemodStateMachine = 0xFF;  // demod_pin_to_sm_index_ value for non-demod GPIOs.

    static constexpr uint kMLATCounterWrapInterruptPriority = 0;
    static constexpr uint kGPIOInterruptPriority = 1;
    static constexpr uint kDemodCompleteInterruptPriority = 2;

    struct ADSBeeConfig {
        PIO preamble_detector_pio = pio0;
        uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
        PIO message_demodulator_pio = pio1;
        uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;

        uint16_t r1090_led_pin = 15;
        // Reading ADS-B on GPIO19. Will look for DEMOD signal on GPIO20.
        uint16_t pulses_pin = bsp.r1090_pulses_pin;
        uint16_t* demod_pins = bsp.r1090_demod_pins;
        // Use GPIO22 for the decode PIO program to output its recovered clock (for debugging only).
        uint16_t* recovered_clk_pins =
            bsp.r1090_recovered_clk_pins;  // Set RECOVERED_CLK to fake pin for high power preamble detector. Will be
                                           // overridden by higher priority (lower index) SM.
        // GPIO 24-25 used as PWM outputs for setting analog comparator threshold voltages.
        uint16_t tl_pwm_pin = bsp.r1090_tl_pwm_pin;
        // GPIO 26-27 used as ADC inputs for reading analog comparator threshold voltages after RF filer.
        uint16_t tl_adc_pin = bsp.r1090_tl_adc_pin;
        uint16_t tl_adc_input = bsp.r1090_tl_adc_input;
        // GPIO 28 used as ADC input for the power level of the last decoded packet.
        uint16_t rssi_adc_pin = bsp.r1090_rssi_adc_pin;
        uint16_t rssi_adc_input = bsp.r1090_rssi_adc_input;
        // GPIO 2-3 are used for the EEPROM and rx gain digipot I2C bus via I2C1.
        i2c_inst_t* onboard_i2c = bsp.onboard_i2c;
        uint16_t onboard_i2c_sda_pin = bsp.onboard_i2c_sda_pin;
        uint16_t onboard_i2c_scl_pin = bsp.onboard_i2c_scl_pin;
        uint32_t onboard_i2c_clk_freq_hz = bsp.onboard_i2c_clk_freq_hz;  // 400kHz
        bool onboard_i2c_requires_init =
            bsp.onboard_i2c_requires_init;  // In case I2c is shared with something else that already initializes it.

        uint16_t bias_tee_enable_pin = 18;

        bool has_subg = bsp.has_subg;

        uint32_t aircraft_dictionary_update_interval_ms = 1000;
        uint32_t rx_position_update_interval_ms = 1000;
    };

    ADSBee(ADSBeeConfig config_in);
    ~ADSBee() {
        subg_radio.~SPICoprocessor();
        subg_radio_ll.~CC1312();
    }
    bool Init();
    bool InitISRs();    // Separate ISR initialization to allow putting ISRs on a separate core.
    bool DeInitISRs();  // Clean up ISR resources (DMA channels, etc.) before stopping core.
    bool Update();

    /**
     * Inlne helper function that converts milliVolts at the AD8313 input to a corresponding value in dBm, using values
     * from the AD8313 datasheet.
     * @param[in] mv Voltage level, in milliVolts.
     * @retval Corresponding power level, in dBm.
     */
    static inline int AD8313MilliVoltsTodBm(int mv) {
        static constexpr uint16_t kLNAGaindB = 44;    // Gain of 2x LNAs in front of the AD8313, from bench testing.
        return 60 * (mv - 1600) / 1000 - kLNAGaindB;  // AD8313 0dBm intercept at 1.6V, slope is 60dBm/V.
    }

    /**
     * Inline helper function that converts ADC counts on theRP2040 to milliVolts.
     * @param[in] adc_counts ADC counts, 0 to 4095.
     * @retval Voltage in milliVolts.
     */
    static inline int ADCCountsToMilliVolts(uint16_t adc_counts) { return 3300 * adc_counts / 0xFFF; }

    /**
     * Returns whether the bias tee is enabled.
     * @retval True if bias tee is enabled, false otherwise.
     */
    bool BiasTeeIsEnabled() { return bias_tee_enabled_; }

    /**
     * Convenience function for temporarily disabling the watchdog without changing its timeout.
     */
    void DisableWatchdog() { watchdog_disable(); }

    /**
     * Convenience function for re-enabling the watchdog with the last used timeout.
     */
    void EnableWatchdog() { SetWatchdogTimeoutSec(watchdog_timeout_sec_); }

    /**
     * Blinks the status LED for a given number of milliseconds. Non-blocking.
     * @param[in] led_on_ms Optional parameter specifying number of milliseconds to turn on for. Defaults to
     * kStatusLEDOnMs.
     */
    void FlashStatusLED(uint32_t led_on_ms = kStatusLEDOnMs);

    /**
     * Forced status LED blink for test fixtures (AT+LED_BLINK): always lights the LED, bypassing the LED_ENABLE
     * setting until the blink expires. Non-blocking.
     * @param[in] duration_ms Number of milliseconds to turn the LED on for.
     */
    void ForceBlinkStatusLED(uint32_t duration_ms);

    /**
     * Creates a composite timestamp using the current value of the SysTick timer (running at 125MHz) and the SysTick
     * wrap counter to simulate a timer running at 48MHz (which matches the frequency of the preamble detector PIO).
     * @param[in] num_bits Number of bits to mask the counter value to. Defaults to full resolution.
     * @retval 48MHz counter value.
     */
    uint64_t GetMLAT48MHzCounts(uint16_t num_bits = 64);

    /**
     * Creates a composite timestamp using the current value of the SysTick timer (running at 125MHz) and the SysTick
     * wrap counter to simulate a timer running at 12MHz, which matches existing decoders that use the Mode S Beast
     * protocol.
     * @param[in] num_bits Number of bits to mask the counter value to. Defaults to 48 bits (6 Bytes) to match Mode S
     * Beast protocol.
     * @retval 48MHz counter value.
     */
    uint64_t GetMLAT12MHzCounts(uint16_t num_bits = 48);

    /**
     * Returns the current value of the MLAT jitter PWM slice's internal counter.
     * This is only used for testing, since the actual value is usually accessed directly or through DMA.
     * @retval Current value of the MLAT jitter PWM slice counter.
     */
    uint16_t GetMLATJitterPWMSliceCounts();

    /**
     * Returns the power level of the noise floor (RSSI sampled between packets and then low-pass filtered).
     * @retval Power level of the noise floor, in dBm.
     */
    int GetNoiseFloordBm();

    /**
     * Returns the noise floor estimate as a voltage at the RSSI ADC input, in the same units as the trigger level
     * offset (the trigger level is noise floor + TL offset).
     * @retval Noise floor RSSI voltage, in milliVolts.
     */
    int GetNoiseFloorMilliVolts();

    /**
     * Get the current temperature used in learning trigger level (simulated annealing). A temperature of 0 means
     * learning has completed.
     * @retval Current temperature used for simulated annealing, in milliVolts.
     */
    uint16_t GetTLLearningTemperatureMV();

    /**
     * Return the value of the low Minimum Trigger Level threshold in milliVolts.
     * @retval TL in milliVolts.
     */
    int GetTLOffsetMilliVolts() { return tl_offset_mv_; }

    inline uint32_t GetWatchdogTimeoutSec() { return watchdog_timeout_sec_; }

    /**
     * ISR for GPIO interrupts.
     */
    void OnDemodBegin(uint gpio);

    /**
     * ISR triggered by DECODE completing, via PIO0 IRQ0.
     */
    void OnDemodComplete();

    /**
     * ISR triggered by SysTick interrupt. Used to wrap the MLAT counter.
     */
    void OnSysTickWrap();

    /**
     * Resets the watchdog counter to the value set in SetWatchdogTimeoutSec().
     */
    inline void PokeWatchdog() { watchdog_update(); }

    /**
     * Returns the Receive Signal Strength Indicator (RSSI) of the signal currently provided by the RF power detector,
     * in mV.
     * @retval Voltage from the RF power detector, in mV.
     */
    inline int ReadSignalStrengthMilliVoltsBlocking();

    /**
     * Begins an ADC read of the Receive Signal Strength Indicator (RSSI) of the signal currently provided by the RF
     * power detector, in mV. Non-blocking; must call ReadSignalStrengthMilliVoltsNonBlockingComplete() to complete the
     * read. Ideally a bunch of other operations can be staged between begin and end to utilize the ADC read time.
     */
    inline void ReadSignalStrengthMilliVoltsNonBlockingBegin();

    /**
     * Completes a non-blocking ADC read of the Receive Signal Strength Indicator (RSSI) of the signal currently
     * provided by the RF power detector, in mV. Must be called after ReadSignalStrengthMilliVoltsNonBlockingBegin().
     * NOTE: This function blocks until the ADC read is complete.
     * @retval Voltage from the RF power detector, in mV.
     */
    inline int ReadSignalStrengthMilliVoltsNonBlockingComplete();

    /**
     * Returns the Receive Signal Strength Indicator (RSSI) of the message that is currently being provided by the RF
     * power detector, in dBm. makes use of ReadSignalStrengthMilliVolts().
     * @retval Voltage form the RF power detector converted to dBm using the chart in the AD8313 datasheet.
     */
    inline int ReadSignalStrengthdBmBlocking();

    /**
     * Read the low Minimum Trigger Level threshold via ADC.
     * @retval TL in milliVolts.
     */
    int ReadTLMilliVolts();

    /**
     * Reboots the RP2040 via the watchdog.
     * @param[in] delay_ms Milliseconds to wait befor rebooting. Defaults to 0 (immediate reboot).
     */
    inline void Reboot(uint16_t delay_ms = 0) { watchdog_reboot(0, 0, delay_ms); }

    /**
     * Returns whether ADS-B receiving is currently enabled.
     * @retval True if enabled, false otherwise.
     */
    bool Receiver1090IsEnabled() { return r1090_enabled_; }

    /**
     * Returns whether the 978MHz receiver is enabled.
     * @retval True if enabled, false otherwise.
     */
    bool SubGRadioIsEnabled() { return subg_radio.IsEnabled(); }

    /**
     * Enable or disable the bias tee to inject 3.3V at the RF IN connector.
     * @param[in] is_enabled True to enable the bias tee, false otherwise.
     */
    inline void SetBiasTeeEnable(bool is_enabled) {
        bias_tee_enabled_ = is_enabled;
        gpio_put(config_.bias_tee_enable_pin, !bias_tee_enabled_);
    }

    /**
     * Enables or disables the ADS-B receiver by hogging the demodulation completed interrupt.
     * @param[in] is_enabled True if ADS-B receiver should be enabled, false otherwise.
     */
    inline void SetReceiver1090Enable(bool is_enabled) {
        r1090_enabled_ = is_enabled;
#ifndef ISRS_ON_CORE1
        // Only enable / disable the IRQ from here if we are running the ISRs on core 0. Otherwise this will enable
        // interrupts on the current core, in addition to the core that's actually supposed to be running them.
        // Simultaneous dual-core ISRs for OnDemodComplete will corrupt all the packets.
        SyncReceiver1090IRQEnable();
#else
        // Core 1 update loop will check adsbee.Receiver1090IsEnabled() to see if IRQ should be enabled.
#endif  // ISRS_ON_CORE1
    }

    /**
     * This function allows core 0 or core 1 to enable the 1090 receiver IRQ which requires access to some private
     * config data. This is NOT the primary API for enabling/disabling the 1090 receiver; that is done through
     * SetReceiver1090Enable(). Note that this function will synchronize the IRQ enable state to match the
     * r1090_enabled_ member variable.
     */
    inline void SyncReceiver1090IRQEnable() {
        irq_set_enabled(config_.preamble_detector_demod_complete_irq, r1090_enabled_);
    }

    /**
     * Checks whether the 1090 receiver IRQ is currently enabled. Can only be called from the same core that runs the
     * IRQs.
     * @retval True if enabled, false otherwise.
     */
    inline bool Receiver1090IRQIsEnabled() { return irq_is_enabled(config_.preamble_detector_demod_complete_irq); }

    /**
     * Enables or disables the sub-GHz radio by powering the receiver chip on or off. Re-initializes the receiver chip
     * if it's being powered on from a previous off state.
     * @param[in] is_enabled True if 978MHz receiver should be enabled, false otherwise.
     */
    inline void SetSubGRadioEnable(SettingsManager::EnableState is_enabled) {
        SettingsManager::EnableState old_enabled = subg_radio_ll.IsEnabledState();
        subg_radio_ll.SetEnableState(is_enabled);
        if ((old_enabled == SettingsManager::EnableState::kEnableStateDisabled ||
             old_enabled == SettingsManager::EnableState::kEnableStateExternal) &&
            subg_radio_ll.IsEnabled()) {
            // Re-initialize the receiver when in ENABLED and EXTERNAL states.
            if (!subg_radio.Init() && is_enabled != SettingsManager::EnableState::kEnableStateExternal) {
                // If subg radio failed to initialize and it's not controlled by external debugger, disable it.
                CONSOLE_ERROR("ADSBee::SetSubGRadioEnable",
                              "Failed to initialize sub-GHz radio after enabling. Disabling sub-GHz radio.");
                subg_radio_ll.SetEnableState(SettingsManager::EnableState::kEnableStateDisabled);
            }
        }
    }

    /**
     * Sets the status LED to a given state. Does not record timestamps for turning off the LED.
     * @param[in] on True to turn on the LED, false to turn it off. Ignored if LEDs are disabled.
     */
    inline void SetStatusLED(bool on) {
        // led_force_blink_ bypasses the LED_ENABLE gate during a forced blink (AT+LED_BLINK), and also keeps a packet
        // flash arriving mid-forced-blink from re-gating the pin off.
        gpio_put(config_.r1090_led_pin, (on && (settings_manager.settings.led_enabled || led_force_blink_)) ? 1 : 0);
    }

    /**
     * Set the Minimum Trigger Level (TL) offset at the AD8313 output in milliVolts.
     * @param[in] tl_offset_mv Trigger level offset from the noise floor in milliVolts. Must be positive. Use a larger
     * value to decrease receiver sensitivity.
     * @retval True if succeeded, False if TL value was out of range.
     */
    bool SetTLOffsetMilliVolts(int tl_offset_mv);

    /**
     * Sets the watchdog timer and enables it.
     * @param[in] watchdog_timeout_sec Maximum interval between PokeWatchdog() calls before watchdog times out and
     * triggers a reboot. 0 = watchodg is disabled. Note that this value is in seconds, to hopefully prevent an
     * unrecoverable loop where the watchdog timer gets set too short and causes the device to reboot before the setting
     * can be changed.
     * @retval True if set successfully, false if invalid watchdog value.
     */
    inline bool SetWatchdogTimeoutSec(uint32_t watchdog_timeout_sec) {
        if (watchdog_timeout_sec > UINT32_MAX / kMsPerSec) {
            return false;  // Watchdog timeout value too big.
        }
        watchdog_timeout_sec_ = watchdog_timeout_sec;
        if (watchdog_timeout_sec_ > 0) {
            watchdog_enable(watchdog_timeout_sec_ * kMsPerSec, true);  // Pause the watchdog timer during debug.
        } else {
            watchdog_disable();
        }
        return true;
    }

    /**
     * Start learning the trigger level through Simulated Annealing. Will begin kTLLearningNumCycles annealing cycles
     * with an annealing interval of kTLLearningIntervalMs milliseconds. Can be provided with maximum and minimum
     * trigger level bounds to allow a narrower search.
     * @param[in] tl_learning_num_cycles Number of cycles to use while annealing trigger level (sets the amount that the
     * annealing temperature is decreased each cycle). Optional, defaults to kTLLearningNumCycles.
     * @param[in] tl_learning_start_temperature_mv Annealing temperature to start with, in mV.
     * @param[in] tl_min_mv Minimum trigger level to use while learning, in milliVolts. Optional, defaults to full scale
     * (kTLMinMV).
     * @param[in] tl_max_mv Maximum trigger level to use while learning, in milliVolts. Optional, defaults to full scale
     * (kTLMaxMV).
     */
    void StartTLLearning(uint16_t tl_learning_num_cycles = kTLLearningNumCycles,
                         uint16_t tl_learning_start_temperature_mv = kTLLearningStartTemperatureMV,
                         uint16_t tl_min_mv = kTLOffsetMinMV, uint16_t tl_max_mv = kTLOffsetMaxMV);

    // Raw packet intake queues.
    PFBQueue<RawModeSPacket> raw_mode_s_packet_queue = PFBQueue<RawModeSPacket>(
        {.buf_len_num_elements = kModeSPacketQueueDepth, .buffer = raw_mode_s_packet_queue_buffer_});
    PFBQueue<RawUATADSBPacket> raw_uat_adsb_packet_queue = PFBQueue<RawUATADSBPacket>(
        {.buf_len_num_elements = kRawUATADSBPacketQueueDepth, .buffer = raw_uat_adsb_packet_queue_buffer_});
    PFBQueue<RawUATUplinkPacket> raw_uat_uplink_packet_queue = PFBQueue<RawUATUplinkPacket>(
        {.buf_len_num_elements = kRawUATUplinkPacketQueueDepth, .buffer = raw_uat_uplink_packet_queue_buffer_});
    PFBQueue<RawRemoteIDPacket> raw_remote_id_packet_queue = PFBQueue<RawRemoteIDPacket>(
        {.buf_len_num_elements = kRawRemoteIDPacketQueueDepth, .buffer = raw_remote_id_packet_queue_buffer_});

    AircraftDictionary aircraft_dictionary;
    CC1312 subg_radio_ll = CC1312({});
    SPICoprocessor subg_radio = SPICoprocessor({.interface = subg_radio_ll, .tag_str = "CC1312"});

    SettingsManager::RxPosition rx_position;
    bool rx_position_available = false;

   private:
    void IngestAndForwardPackets();
    void MLATCounterInit();
    void PIOInit();
    void PIOEnable();
    void PruneAircraftDictionary();
    void Update1090LED();
    void UpdateNoiseFloor();
    void UpdateRxPosition();
    void UpdateTLLearning();

    ADSBeeConfig config_;

    uint32_t irq_wrapper_sm_ = 0;
    uint32_t preamble_detector_sm_[BSP::kMaxNumDemodStateMachines];
    uint32_t preamble_detector_offset_ = 0;
    uint32_t irq_wrapper_offset_ = 0;

    uint32_t message_demodulator_sm_[BSP::kMaxNumDemodStateMachines];
    uint32_t message_demodulator_offset_ = 0;

    uint32_t mlat_jitter_dma_channel_[BSP::kMaxNumDemodStateMachines];
    uint32_t mlat_jitter_pwm_slice_ = 0;
    uint16_t mlat_jitter_counts_on_demod_begin_[BSP::kMaxNumDemodStateMachines] = {0};
    uint16_t mlat_jitter_counts_on_fifo_pull_[BSP::kMaxNumDemodStateMachines] = {0};

    // Maps a demod GPIO number to its state machine index (kNoDemodStateMachine for other GPIOs). Built in PIOInit().
    uint8_t demod_pin_to_sm_index_[NUM_BANK0_GPIOS];

    // OnDemodComplete() duration statistics, only maintained when DEBUG_ISR_TIMING is defined.
    uint32_t isr_count_ = 0;
    uint32_t isr_duration_sum_counts_ = 0;
    uint16_t isr_duration_max_counts_ = 0;

    uint32_t led_on_timestamp_ms_ = 0;
    uint32_t led_on_duration_ms_ = kStatusLEDOnMs;
    bool led_force_blink_ = false;  // True while a forced blink (bypassing LED_ENABLE) is active.

    uint16_t tl_pwm_slice_ = 0;
    uint16_t tl_pwm_chan_ = 0;

    uint16_t tl_offset_mv_ = SettingsManager::Settings::kDefaultTLOffsetMV;
    uint16_t tl_pwm_count_ = 0;  // out of kTLMaxPWMCount

    uint16_t tl_adc_counts_ = 0;

    uint32_t tl_learning_cycle_start_timestamp_ms_ = 0;
    uint16_t tl_learning_temperature_mv_ = 0;  // Don't learn automatically.
    int16_t tl_learning_temperature_step_mv_ = 0;
    uint16_t tl_learning_max_offset_mv_ = kTLOffsetMaxMV;
    uint16_t tl_learning_min_offset_mv_ = kTLOffsetMinMV;
    int16_t tl_learning_num_valid_packets_ = 0;
    int16_t tl_learning_prev_num_valid_packets_ = 1;  // Set to 1 to avoid dividing by 0.
    uint16_t tl_learning_prev_tl_offset_mv_ = tl_offset_mv_;

    uint64_t mlat_counter_wraps_ = 0;

    // Buffer used to hold packets currently being received.
    RawModeSPacket rx_packet_[BSP::kMaxNumDemodStateMachines];
    // Buffer where packets are stored before being sent to the decoder.
    RawModeSPacket raw_mode_s_packet_queue_buffer_[kModeSPacketQueueDepth];

    // Buffer where packets are stored while incoming from the CC1312.
    RawUATADSBPacket raw_uat_adsb_packet_queue_buffer_[kRawUATADSBPacketQueueDepth];
    RawUATUplinkPacket raw_uat_uplink_packet_queue_buffer_[kRawUATUplinkPacketQueueDepth];
    RawRemoteIDPacket raw_remote_id_packet_queue_buffer_[kRawRemoteIDPacketQueueDepth];

    uint32_t last_aircraft_dictionary_update_timestamp_ms_ = 0;
    uint32_t last_rx_position_update_timestamp_ms_ = 0;

    bool r1090_enabled_ = true;
    bool r1090_packet_queue_overflowed_ = false;
    bool bias_tee_enabled_ = false;
    uint32_t watchdog_timeout_sec_ = SettingsManager::Settings::kDefaultWatchdogTimeoutSec;

    uint32_t subg_radio_last_update_timestamp_ms_ = 0;

    // Noise floor estimate. The filter runs on noise_floor_mv_fp_ (mV << kNoiseFloorFixedPointShift); noise_floor_mv_
    // mirrors it as a plain integer for the ISR (sigq) and status reporting.
    int32_t noise_floor_mv_fp_ = 0;
    volatile int32_t noise_floor_mv_ = 0;
    bool noise_floor_initialized_ = false;
    uint16_t noise_floor_consecutive_outliers_ = 0;
    uint32_t noise_floor_last_sample_timestamp_ms_ = 0;
    uint32_t noise_floor_last_accepted_timestamp_ms_ = 0;
    uint32_t demod_pins_mask_ = 0;  // GPIO bitmask of the demod pins, built in Init(). HI = demodulation in progress.
    // Written by OnDemodBegin() (ISR core), read by UpdateNoiseFloor() (main loop) to reject contaminated samples.
    volatile uint32_t last_demod_begin_timestamp_us_ = 0;

    /** 978MHz Receiver Parameters **/
};

extern ADSBee adsbee;

#endif /* _ADS_BEE_HH_ */
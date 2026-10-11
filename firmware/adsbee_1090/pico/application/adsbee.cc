#include "adsbee.hh"

#include <hardware/structs/scb.h>
#include <hardware/structs/systick.h>

#include "capture.pio.h"
#include "hal.hh"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/exception.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "mode_s_packet_decoder.hh"
#include "pico/binary_info.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "spi_coprocessor.hh"
#include "stdio.h"  // for printing

// #include <charconv>
#include <string.h>  // for strcat

#include "comms.hh"           // For debug prints.
#include "gnss_interface.hh"  // For the gnss receiver instance.

// Uncomment to measure the capture demodulation time in FinishCapture() and print avg/max once per second.
// #define DEBUG_ISR_TIMING

// Uncomment this to hold the status LED on for 5 seconds if the watchdog commanded a reboot.
// #define WATCHDOG_REBOOT_WARNING

#define ADC_COUNTS_TO_MV(adc_counts) ((adc_counts) * kVDDMV / 4095)
#define MLAT_SYSTEM_CLOCK_RATIO      48 / 125
// Scales 125MHz system clock into a 48MHz counter.
static const uint32_t kMLATWrapCounterIncrement = (1 << 24) * MLAT_SYSTEM_CLOCK_RATIO;

constexpr float kPreambleMatcherFreqHz = 48e6;  // Running at 48MHz (24 clock cycles per half bit).
constexpr float kPulseSamplerFreqHz = 48e6;     // Pulse sampler PIO clock: one comparator sample every 4 cycles.
// A matcher that fires this soon after another capture began is looking at that message's data pulses, not at a new
// preamble.
constexpr uint32_t kCaptureGuardLongUs = 118;   // 112 data bits after the preamble match, plus margin.
constexpr uint32_t kCaptureGuardShortUs = 60;   // 56 data bits, plus margin.
constexpr uint16_t kMidCheckWords = 6;          // Sample words looked at mid-message to drop captures of noise.
constexpr uint16_t kMidCheckMinHiSamples = 40;  // Of 192: a message has about 96, comparator noise a handful.
// Ending the capture of 56 bit messages early.
constexpr uint32_t kMidCheckFirstSample =
    mode_s_soft_demod::kNominalStartOffsetSamples - 1;  // First sample of the first data bit, leaning early.
constexpr int kShortCaptureMinConfidence = 3;           // HI sample margin (of 6) for calling the first bit a 0.
constexpr uint32_t kShortCaptureAlarmDelayUs = 17;  // From the mid-message interrupt (~43us) to past 56 bits (59us).
// The sampler starts about half a microsecond before the data block. Timestamps keep referring to the start of the
// data block, as with the earlier demodulator.
constexpr uint32_t kSamplerLead48MHzCounts = (mode_s_soft_demod::kNominalStartOffsetSamples + 1) * 4;

constexpr float kInt16MaxRecip = 1.0f / INT16_MAX;

ADSBee* isr_access = nullptr;

/** Begin pass-through functions for public access **/
void __time_critical_func(on_systick_exception)() { isr_access->OnSysTickWrap(); }

void __time_critical_func(on_demod_pin_change)(uint gpio, uint32_t event_mask) {
    switch (event_mask) {
        case GPIO_IRQ_EDGE_RISE:
            isr_access->OnDemodBegin(gpio);
            break;
        case GPIO_IRQ_EDGE_FALL:
            break;
        case GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL:
            break;
    }
    gpio_acknowledge_irq(gpio, event_mask);
}

void __time_critical_func(on_demod_mid)() { isr_access->OnDemodMid(); }

void __time_critical_func(on_capture_complete)() { isr_access->OnCaptureComplete(); }

void __time_critical_func(on_short_capture_alarm)(uint alarm_num) { isr_access->OnShortCaptureAlarm(); }

/** End pass-through functions for public access **/

ADSBee::ADSBee(ADSBeeConfig config_in) {
    config_ = config_in;

    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        preamble_matcher_sm_[sm_index] = pio_claim_unused_sm(config_.preamble_matcher_pio, true);
        pulse_sampler_sm_[sm_index] = pio_claim_unused_sm(config_.pulse_sampler_pio, true);
    }

    preamble_matcher_offset_ = pio_add_program(config_.preamble_matcher_pio, &preamble_matcher_program);
    pulse_sampler_offset_ = pio_add_program(config_.pulse_sampler_pio, &pulse_sampler_program);

    // Put IRQ parameters into the global scope for the ISRs.
    isr_access = this;

    // Figure out slice and channel values that will be used for setting PWM duty cycle.
    tl_pwm_slice_ = pwm_gpio_to_slice_num(config_.tl_pwm_pin);
    tl_pwm_chan_ = pwm_gpio_to_channel(config_.tl_pwm_pin);
}

bool ADSBee::Init() {
    gpio_init(config_.r1090_led_pin);
    gpio_set_dir(config_.r1090_led_pin, GPIO_OUT);
    gpio_put(config_.r1090_led_pin, 0);

    // Initialize the sync pin if it is defined.
    if (bsp.sync_pin != UINT16_MAX) {
        gpio_init(bsp.sync_pin);
        gpio_set_dir(bsp.sync_pin, GPIO_OUT);
        gpio_put(bsp.sync_pin, 0);  // Set to low.
    }

    // Disable the Sub-GHz SPI bus output.
    gpio_init(bsp.subg_cs_pin);
    gpio_set_dir(bsp.subg_cs_pin, GPIO_OUT);
    gpio_put(bsp.subg_cs_pin, 1);  // Disable is active LO.

    // Initialize the TL bias PWM output.
    gpio_set_function(config_.tl_pwm_pin, GPIO_FUNC_PWM);
    pwm_set_wrap(tl_pwm_slice_, kTLMaxPWMCount);

    SetTLOffsetMilliVolts(SettingsManager::Settings::kDefaultTLOffsetMV);
    pwm_set_enabled(tl_pwm_slice_, true);

    // Initialize the trigger level bias ADC input.
    adc_init();
    adc_gpio_init(config_.tl_adc_pin);
    adc_gpio_init(config_.rssi_adc_pin);

    // Bitmask of the demod pins, used by UpdateNoiseFloor() to avoid sampling RSSI during a demodulation.
    demod_pins_mask_ = 0;
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        demod_pins_mask_ |= 1u << config_.demod_pins[sm_index];
    }

    // Initialize I2C for talking to the EEPROM and rx gain digipot.
    if (config_.onboard_i2c_requires_init) {
        i2c_init(config_.onboard_i2c, config_.onboard_i2c_clk_freq_hz);
        gpio_set_function(config_.onboard_i2c_sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(config_.onboard_i2c_scl_pin, GPIO_FUNC_I2C);
    }

    // Initialize the bias tee.
    gpio_init(config_.bias_tee_enable_pin);
    gpio_put(config_.bias_tee_enable_pin, 1);  // Enable is active LO.
    gpio_set_dir(config_.bias_tee_enable_pin, GPIO_OUT);

    // Set the last dictionary update timestamp.
    last_aircraft_dictionary_update_timestamp_ms_ = get_time_since_boot_ms();

    // Initialize sub-GHz radio.
    if (config_.has_subg) {
        SetSubGRadioEnable(settings_manager.settings.subg_enabled);
    } else {
        SetSubGRadioEnable(SettingsManager::EnableState::kEnableStateDisabled);
    }

#ifdef WATCHDOG_REBOOT_WARNING
    // Throw a fit if the watchdog caused a reboot.
    if (watchdog_caused_reboot()) {
        CONSOLE_WARNING("ADSBee::Init", "Watchdog caused reboot.");
        DisableWatchdog();
        SetStatusLED(true);
        sleep_ms(5000);
        SetStatusLED(false);
        EnableWatchdog();
    }
#endif  // WATCHDOG_REBOOT_WARNING

    return true;
}

bool ADSBee::InitISRs() {
    PIOInit();
    MLATCounterInit();
    PIOEnable();
    return true;
}

bool ADSBee::DeInitISRs() {
    // Disable PIO state machines before cleaning up
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        pio_sm_set_enabled(config_.preamble_matcher_pio, preamble_matcher_sm_[sm_index], false);
        pio_sm_set_enabled(config_.pulse_sampler_pio, pulse_sampler_sm_[sm_index], false);
    }

    // Stop and unclaim DMA channels used for MLAT jitter counters
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        dma_channel_abort(mlat_jitter_dma_channel_[sm_index]);
        if (dma_channel_is_claimed(mlat_jitter_dma_channel_[sm_index])) {
            dma_channel_unclaim(mlat_jitter_dma_channel_[sm_index]);
        }
    }

    // Disable interrupts
    irq_set_enabled(config_.preamble_matcher_irq, false);
    irq_set_enabled(DMA_IRQ_1, false);
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        if (sample_dma_channel_[sm_index] >= 0) {
            dma_channel_set_irq1_enabled(sample_dma_channel_[sm_index], false);
            dma_channel_abort(sample_dma_channel_[sm_index]);
        }
    }
    if (short_capture_alarm_num_ >= 0) {
        hardware_alarm_cancel(short_capture_alarm_num_);
    }
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        gpio_set_irq_enabled(config_.demod_pins[sm_index], GPIO_IRQ_EDGE_RISE, false);
    }

    return true;
}

bool ADSBee::Update() {
    Update1090LED();

    PruneAircraftDictionary();

    UpdateRxPosition();

    IngestAndForwardPackets();

    if (r1090_packet_queue_overflowed_) {
        CONSOLE_ERROR("ADSBee::Update", "Mode S decoder input queue overflowed.");
        r1090_packet_queue_overflowed_ = false;
    }

    if (decoder.decoded_mode_s_packet_out_queue_overflowed_) {
        CONSOLE_ERROR("ADSBee::Update", "Mode S decoder output queue overflowed.");
        decoder.decoded_mode_s_packet_out_queue_overflowed_ = false;
    }

    UpdateTLLearning();

    UpdateNoiseFloor();

    // Update sub-GHz radio.
    if (config_.has_subg && subg_radio.IsEnabled()) {
        bool subg_link_was_down = subg_radio.IsLinkDown();
        if (subg_radio.Update()) {
            // Sub-GHz radio comms successful.
            subg_radio_last_update_timestamp_ms_ = get_time_since_boot_ms();
        } else {
            // Sub-GHz radio comms failed. Log an error and restart if enough time has elapsed since the last successful
            // Sub-GHz radio comms.
            if (!subg_link_was_down) CONSOLE_ERROR("ADSBee::Update", "Failed to update sub-GHz radio.");

            if (get_time_since_boot_ms() - subg_radio_last_update_timestamp_ms_ >= kSubGRadioFailRebootIntervalMs) {
                // Too much time elapsed since last valid SubG radio comms. Reboot it.
                CONSOLE_ERROR("ADSBee::Update", "%u ms elapsed since SubG comms. Rebooting SubG radio.",
                              kSubGRadioFailRebootIntervalMs);
                SettingsManager::EnableState prev_enable_state = subg_radio_ll.IsEnabledState();
                SetSubGRadioEnable(SettingsManager::EnableState::kEnableStateDisabled);
                sleep_ms_blocking(kSubGRadioFailRebootPowerOffDurationMs);
                SetSubGRadioEnable(prev_enable_state);
            }
            return false;
        }
    }
    return true;
}

void ADSBee::FlashStatusLED(uint32_t led_on_ms) {
    uint32_t timestamp_ms = get_time_since_boot_ms();
    // Later off-deadline wins: don't let a short packet flash cut an in-progress longer blink short.
    if ((int32_t)((timestamp_ms + led_on_ms) - (led_on_timestamp_ms_ + led_on_duration_ms_)) > 0) {
        led_on_timestamp_ms_ = timestamp_ms;
        led_on_duration_ms_ = led_on_ms;
    }
    SetStatusLED(true);
}

void ADSBee::ForceBlinkStatusLED(uint32_t duration_ms) {
    led_force_blink_ = true;  // Set before SetStatusLED so the LED_ENABLE bypass applies.
    FlashStatusLED(duration_ms);
}

uint64_t __time_critical_func(ADSBee::GetMLAT48MHzCounts)(uint16_t num_bits) {
    // Combine the wrap counter with the current value of the SysTick register and mask to 48 bits.
    // Note: 24-bit SysTick value is subtracted from UINT_24_MAX to make it count up instead of down.
    // The SysTick counter can reload without mlat_counter_wraps_ being incremented yet: either the wrap handler runs
    // between the two reads (called from thread mode), or it is pending because we were called from an ISR it can't
    // preempt (on the RP2040 only the top two priority bits are implemented, so the GPIO, demod complete and SysTick
    // priorities set in adsbee.hh are all equal). Either way the timestamp would jump back by a full wrap (~134ms).
    uint64_t wraps;
    uint32_t elapsed_sys_clk_counts;
    bool wrap_pending;
    do {
        wraps = mlat_counter_wraps_;
        elapsed_sys_clk_counts = 0xFFFFFF - systick_hw->cvr;
        wrap_pending = scb_hw->icsr & M0PLUS_ICSR_PENDSTSET_BITS;
        if (wrap_pending) {
            // The counter has reloaded but the wrap hasn't been counted. Re-read it so it's known to be post-reload.
            elapsed_sys_clk_counts = 0xFFFFFF - systick_hw->cvr;
        }
    } while (wraps != mlat_counter_wraps_);  // Wrap handler ran while we were reading, try again.
    if (wrap_pending) {
        wraps += kMLATWrapCounterIncrement;
    }
    return (wraps + (elapsed_sys_clk_counts * MLAT_SYSTEM_CLOCK_RATIO)) & (UINT64_MAX >> (64 - num_bits));
}

uint64_t ADSBee::GetMLAT12MHzCounts(uint16_t num_bits) {
    // Piggyback off the higher resolution 48MHz timer function.
    return GetMLAT48MHzCounts(50) >> 2;  // Divide 48MHz counter by 4, widen the mask by 2 bits to compensate.
}

uint16_t __time_critical_func(ADSBee::GetMLATJitterPWMSliceCounts)() {
    // Returns the current value of the MLAT jitter counter PWM slice.
    return pwm_hw->slice[mlat_jitter_pwm_slice_].ctr;
}

int __time_critical_func(ADSBee::GetNoiseFloordBm)() { return DetectorMilliVoltsTodBm(noise_floor_mv_); }

int ADSBee::GetNoiseFloorMilliVolts() { return noise_floor_mv_; }

void ADSBee::UpdateRxPosition() {
    // Rate limiting.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_rx_position_update_timestamp_ms_ < config_.rx_position_update_interval_ms) {
        return;
    }
    last_rx_position_update_timestamp_ms_ = timestamp_ms;

    // Update rx position based on selected position source. Note this only handles periodic automatic updates.
    // One-time changes are handled in AT comms.
    switch (rx_position.source) {
        case SettingsManager::RxPosition::PositionSource::kPositionSourceNone:
            // No position available.
            rx_position_available = false;
            break;
        case SettingsManager::RxPosition::PositionSource::kPositionSourceLowestAircraft:
            // Use position of lowest aircraft in dictionary.
            // Make variables to write into since we can't bind directly to the packed struct members.
            float latitude_deg, longitude_deg, heading_deg;
            int32_t gnss_altitude_ft, baro_altitude_ft, speed_kts;
            if (aircraft_dictionary.GetLowestAircraftPosition(
                    latitude_deg, longitude_deg, reinterpret_cast<int32_t&>(gnss_altitude_ft),
                    reinterpret_cast<int32_t&>(baro_altitude_ft), heading_deg, speed_kts)) {
                rx_position.latitude_deg = latitude_deg;
                rx_position.longitude_deg = longitude_deg;
                rx_position.gnss_altitude_ft = gnss_altitude_ft;
                rx_position.baro_altitude_ft = baro_altitude_ft;
                rx_position.heading_deg = heading_deg;
                rx_position.speed_kts = speed_kts;
                rx_position_available = true;
            } else {
                // No valid aircraft position available. Don't update receiver position.
                rx_position_available = false;
            }
            break;
        case SettingsManager::RxPosition::PositionSource::kPositionSourceGNSS:
            // Use the position reported by the GNSS receiver, if it has a valid, fresh fix.
            if (gnss->HasValidFix()) {
                const NMEAParser::GNSSFix& gnss_fix = gnss->fix();
                rx_position.latitude_deg = gnss_fix.latitude_deg;
                rx_position.longitude_deg = gnss_fix.longitude_deg;
                rx_position.gnss_altitude_ft = gnss_fix.altitude_ft;
                rx_position.heading_deg = gnss_fix.heading_deg;
                rx_position.speed_kts = gnss_fix.speed_kts;
                rx_position_available = true;
            } else {
                // No valid GNSS fix (incl. module absent / comms not working): mark unavailable,
                // mirroring the kPositionSourceLowestAircraft "no valid position" branch.
                rx_position_available = false;
            }
            break;
        case SettingsManager::RxPosition::PositionSource::kPositionSourceFixed:
            // Fixed position is always available.
            rx_position_available = true;
            break;
        case SettingsManager::RxPosition::PositionSource::kPositionSourceAircraftMatchingICAO: {
            // Try to find the aircraft with the matching ICAO address in the dictionary.
            // Check both Mode S and UAT, and use the most recently seen aircraft if both exist.
            uint32_t mode_s_uid = Aircraft::ICAOToUID(rx_position.icao_address, Aircraft::kAircraftTypeModeS);
            ModeSAircraft* mode_s_aircraft = aircraft_dictionary.GetAircraftPtr<ModeSAircraft>(mode_s_uid, false);
            bool mode_s_valid = mode_s_aircraft && mode_s_aircraft->HasBitFlag(ModeSAircraft::kBitFlagPositionValid);

            uint32_t uat_uid = Aircraft::ICAOToUID(rx_position.icao_address, Aircraft::kAircraftTypeUAT);
            UATAircraft* uat_aircraft = aircraft_dictionary.GetAircraftPtr<UATAircraft>(uat_uid, false);
            bool uat_valid = uat_aircraft && uat_aircraft->HasBitFlag(UATAircraft::kBitFlagPositionValid);

            // Determine which aircraft to use based on validity and recency.
            Aircraft* selected_aircraft = nullptr;
            if (mode_s_valid && uat_valid) {
                // Both exist and have valid positions; use the most recently seen.
                if (mode_s_aircraft->last_message_timestamp_ms >= uat_aircraft->last_message_timestamp_ms) {
                    selected_aircraft = mode_s_aircraft;
                } else {
                    selected_aircraft = uat_aircraft;
                }
            } else if (mode_s_valid) {
                selected_aircraft = mode_s_aircraft;
            } else if (uat_valid) {
                selected_aircraft = uat_aircraft;
            }

            if (selected_aircraft) {
                rx_position.latitude_deg = selected_aircraft->latitude_deg;
                rx_position.longitude_deg = selected_aircraft->longitude_deg;
                rx_position.gnss_altitude_ft = selected_aircraft->gnss_altitude_ft;
                rx_position.baro_altitude_ft = selected_aircraft->baro_altitude_ft;
                rx_position.heading_deg = selected_aircraft->direction_deg;
                rx_position.speed_kts = selected_aircraft->speed_kts;
                rx_position_available = true;
            } else {
                // No aircraft found with the matching ICAO address or no valid position.
                rx_position_available = false;
            }
            break;
        }
        default:
            CONSOLE_ERROR("ADSBee::GetRxPosition", "Unknown Rx position source %d.",
                          static_cast<int>(rx_position.source));
    }

    // Update the aircraft dictionary's reference location.
    if (rx_position_available) {
        aircraft_dictionary.SetReferencePosition(rx_position.latitude_deg, rx_position.longitude_deg);
    }
}

uint16_t ADSBee::GetTLLearningTemperatureMV() { return tl_learning_temperature_mv_; }

void __time_critical_func(ADSBee::OnDemodBegin)(uint gpio) {
    // Read MLAT counter at the beginning to reduce jitter after interrupt.
    uint16_t mlat_jitter_counts_now = GetMLATJitterPWMSliceCounts();
    uint64_t mlat_48mhz_64bit_counts = isr_access->GetMLAT48MHzCounts();

    uint16_t sm_index = gpio < NUM_BANK0_GPIOS ? demod_pin_to_sm_index_[gpio] : kNoDemodStateMachine;
    if (sm_index == kNoDemodStateMachine) {
        return;  // Ignore; wasn't the start of a demod interval for a known SM.
    }
    // Demodulation period is beginning! Store the MLAT counter.
    mlat_jitter_counts_on_demod_begin_[sm_index] = mlat_jitter_counts_now;
    rx_packet_[sm_index].mlat_48mhz_64bit_counts = mlat_48mhz_64bit_counts;  // Save this to modify later.
    last_demod_begin_timestamp_us_ = time_us_32();  // Lets the noise floor sampler avoid this packet.

    if (!gpio_get(gpio)) {
        // Stale edge: this capture was already dropped or finished (interrupts were held off for a while). The
        // matcher is hunting again and must not be touched.
        return;
    }
    if (!r1090_enabled_) {
        AbortCapture(sm_index);  // Receiver is off: keep the matchers idle-cycling, demodulate nothing.
        return;
    }
    uint32_t now_us = last_demod_begin_timestamp_us_;
    for (uint16_t other = 0; other < bsp.r1090_num_demod_state_machines; other++) {
        if (other == sm_index || !capture_active_[other]) {
            continue;
        }
        uint32_t guard_us = capture_is_short_[other] ? kCaptureGuardShortUs : kCaptureGuardLongUs;
        if (now_us - capture_begin_us_[other] < guard_us) {
            // Matched on the data pulses of the message that capture is taking: drop it, keep the matcher hunting.
            AbortCapture(sm_index);
            return;
        }
    }
    capture_begin_us_[sm_index] = now_us;
    capture_is_short_[sm_index] = false;
    capture_active_[sm_index] = true;
}

void __time_critical_func(ADSBee::OnDemodMid)() {
    PIO pio = config_.preamble_matcher_pio;
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        uint32_t source_mask = 1u << (PIO_INTR_SM0_LSB + sm_index);
        if (!(pio->inte0 & source_mask) || !pio_interrupt_get(pio, sm_index)) {
            continue;
        }
        if (!capture_active_[sm_index]) {
            // The begin interrupt for this capture was never handled: no timestamp. Drop it (the matcher is stalled
            // on its interrupt flag, so it is safe to send it back to hunting).
            AbortCapture(sm_index);
            continue;
        }
        // The RSSI low pass has settled on the message: sample it (~2us).
        ReadSignalStrengthMilliVoltsNonBlockingBegin();
        sample_rssi_mv_[sm_index] = ReadSignalStrengthMilliVoltsNonBlockingComplete();
        // The flag stays set (it holds the matcher) until the capture is in: mask it so this ISR doesn't refire.
        hw_clear_bits(&pio->inte0, source_mask);

        // About 500 samples are in. A message keeps the comparator HI half of the time; a capture with few HI
        // samples means the matcher fired on noise: drop it now instead of holding the matcher and sampler for the
        // full length.
        uint16_t hi_samples = 0;
        for (uint16_t i = 0; i < kMidCheckWords; i++) {
            uint32_t word = sample_buf_[sm_index][i];
            for (uint16_t shift = 0; shift < 32; shift += 6) {
                hi_samples += mode_s_soft_demod::kPopCount6[(word >> shift) & 0x3F];
            }
        }
        if (hi_samples < kMidCheckMinHiSamples) {
            hw_set_bits(&pio->inte0, source_mask);
            AbortCapture(sm_index);
            continue;
        }
        uint32_t first_bit = mode_s_soft_demod::BitSamples(sample_buf_[sm_index], kMidCheckFirstSample);
        int first_bit_margin = mode_s_soft_demod::kPopCount6[first_bit & 0x3F] -
                               mode_s_soft_demod::kPopCount6[first_bit >> mode_s_soft_demod::kSamplesPerChip];
        // A confident 0 in the first bit of the downlink format is a 56 bit message: end its capture as soon as its
        // samples are in (one alarm: if it is taken, this capture just runs its full length).
        if (first_bit_margin >= kShortCaptureMinConfidence) {
            capture_is_short_[sm_index] = true;
            if (short_capture_sm_index_ < 0) {
                short_capture_sm_index_ = sm_index;
                if (hardware_alarm_set_target(short_capture_alarm_num_,
                                              make_timeout_time_us(kShortCaptureAlarmDelayUs))) {
                    short_capture_sm_index_ = -1;  // Target already passed: no callback will come.
                }
            }
        }
    }
}

void __time_critical_func(ADSBee::OnCaptureComplete)() {
    uint32_t pending = dma_hw->ints1;
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        uint32_t channel_mask = 1u << sample_dma_channel_[sm_index];
        if (!(pending & channel_mask)) {
            continue;
        }
        dma_hw->ints1 = channel_mask;
        FinishCapture(sm_index, mode_s_soft_demod::kCaptureWords);
    }
}

void __time_critical_func(ADSBee::OnShortCaptureAlarm)() {
    int8_t sm_index = short_capture_sm_index_;
    short_capture_sm_index_ = -1;
    if (sm_index < 0) {
        return;
    }
    uint channel = sample_dma_channel_[sm_index];
    uint32_t words_done = mode_s_soft_demod::kCaptureWords - dma_channel_hw_addr(channel)->transfer_count;
    if (!dma_channel_is_busy(channel) || words_done < mode_s_soft_demod::kShortCaptureWords) {
        return;  // Already finished by the DMA interrupt, or late samples: let the capture run out.
    }
    // Stop the DMA channel. Its interrupt is masked around the abort (an abort can raise it) and cleared after.
    dma_channel_set_irq1_enabled(channel, false);
    dma_channel_abort(channel);
    dma_hw->ints1 = 1u << channel;
    dma_channel_set_irq1_enabled(channel, true);
    FinishCapture(sm_index, mode_s_soft_demod::kShortCaptureWords);
}

void __time_critical_func(ADSBee::RearmSampler)(uint16_t sm_index) {
    PIO sampler_pio = config_.pulse_sampler_pio;
    uint sampler_sm = pulse_sampler_sm_[sm_index];
    pio_sm_set_enabled(sampler_pio, sampler_sm, false);
    dma_channel_abort(sample_dma_channel_[sm_index]);
    dma_hw->ints1 = 1u << sample_dma_channel_[sm_index];
    pio_sm_clear_fifos(sampler_pio, sampler_sm);
    pio_sm_restart(sampler_pio, sampler_sm);
    pio_sm_exec(sampler_pio, sampler_sm, pio_encode_jmp(pulse_sampler_offset_ + pulse_sampler_offset_idle));
    // Refill the TX FIFO: the sampler's pull is the trigger for the timestamp DMA.
    while (!pio_sm_is_tx_fifo_full(sampler_pio, sampler_sm)) {
        pio_sm_put(sampler_pio, sampler_sm, 0xFFFFFFFF);
    }
    dma_channel_abort(mlat_jitter_dma_channel_[sm_index]);
    dma_channel_set_write_addr(mlat_jitter_dma_channel_[sm_index], &mlat_jitter_counts_on_fifo_pull_[sm_index], false);
    dma_channel_start(mlat_jitter_dma_channel_[sm_index]);
    dma_channel_set_write_addr(sample_dma_channel_[sm_index], sample_buf_[sm_index], false);
    dma_channel_set_trans_count(sample_dma_channel_[sm_index], mode_s_soft_demod::kCaptureWords, true);
    pio_sm_set_enabled(sampler_pio, sampler_sm, true);
}

void __time_critical_func(ADSBee::AbortCapture)(uint16_t sm_index) {
    // The matcher is in its wait after the match (or stalled on its interrupt flag): send it back to hunting. It
    // already passed the round robin token on at its edge, so the ring is unaffected.
    PIO pio = config_.preamble_matcher_pio;
    pio_sm_exec(pio, preamble_matcher_sm_[sm_index],
                pio_encode_jmp(preamble_matcher_offset_ + preamble_matcher_offset_follow_irq));
    pio_interrupt_clear(pio, sm_index);
    if (short_capture_sm_index_ == static_cast<int8_t>(sm_index)) {
        short_capture_sm_index_ = -1;
    }
    RearmSampler(sm_index);
    capture_active_[sm_index] = false;
}

void __time_critical_func(ADSBee::FinishCapture)(uint16_t sm_index, uint16_t num_words) {
    {
        if (short_capture_sm_index_ == static_cast<int8_t>(sm_index)) {
            short_capture_sm_index_ = -1;  // Full capture finished first: the alarm has nothing left to do.
        }
        // Take the samples and the timestamp jitter count, then free the sampler and its matcher before the (slower)
        // demodulation so they can take the next message.
        for (uint16_t i = 0; i <= mode_s_soft_demod::kCaptureWords; i++) {
            demod_samples_[i] = i < num_words ? sample_buf_[sm_index][i] : 0;
        }
        uint16_t jitter_sys_clk_counts =
            mlat_jitter_counts_on_demod_begin_[sm_index] - mlat_jitter_counts_on_fifo_pull_[sm_index];
        uint64_t mlat_48mhz_64bit_counts = rx_packet_[sm_index].mlat_48mhz_64bit_counts -
                                           MLATJitterCountsTo48MHzCounts(jitter_sys_clk_counts) +
                                           kSamplerLead48MHzCounts;
        int rssi_mv = sample_rssi_mv_[sm_index];

        // Release the preamble matcher (it drops the demod pin), then park the sampler for the next message.
        pio_interrupt_clear(config_.preamble_matcher_pio, sm_index);
        hw_set_bits(&config_.preamble_matcher_pio->inte0, 1u << (PIO_INTR_SM0_LSB + sm_index));
        RearmSampler(sm_index);
        capture_active_[sm_index] = false;

        aircraft_dictionary.Record1090Demod(sm_index);
#ifdef DEBUG_ISR_TIMING
        uint16_t demod_start_counts = GetMLATJitterPWMSliceCounts();
#endif
        mode_s_soft_demod::Result result;
        bool found = mode_s_soft_demod::Demodulate(demod_samples_, num_words, result);
#ifdef DEBUG_ISR_TIMING
        // 16-bit counter at 125MHz wraps every ~524us, longer than a demodulation.
        uint16_t demod_duration_counts = GetMLATJitterPWMSliceCounts() - demod_start_counts;
        if (demod_duration_counts > isr_duration_max_counts_) {
            isr_duration_max_counts_ = demod_duration_counts;
        }
        isr_duration_sum_counts_ += demod_duration_counts;
        isr_count_++;
#endif
        if (found) {
            RawModeSPacket packet;
            for (uint16_t i = 0; i < RawModeSPacket::kMaxPacketLenWords32; i++) {
                packet.buffer[i] = result.buffer[i];
            }
            packet.buffer_len_bytes = result.len_bits / kBitsPerByte;
            packet.sigs_dbm =
                r1090_rssi::PacketMilliVoltsTodBm(rssi_mv, noise_floor_mv_, bsp.r1090_rf_frontend_version);
            packet.sigq_db = packet.sigs_dbm - GetNoiseFloordBm();
            packet.source = sm_index;
            packet.mlat_48mhz_64bit_counts = mlat_48mhz_64bit_counts;
            if (result.len_bits == RawModeSPacket::kSquitterPacketLenBits) {
                aircraft_dictionary.Record1090RawSquitterFrame(sm_index);
            } else {
                aircraft_dictionary.Record1090RawExtendedSquitterFrame(sm_index);
            }
            if (!decoder.raw_mode_s_packet_in_queue.Enqueue(packet)) {
                r1090_packet_queue_overflowed_ = true;
            }
        }
    }
}

void __time_critical_func(ADSBee::OnSysTickWrap)() { mlat_counter_wraps_ += kMLATWrapCounterIncrement; }

int ADSBee::ReadSignalStrengthMilliVoltsBlocking() {
    adc_select_input(config_.rssi_adc_input);
    int rssi_adc_counts = adc_read();
    return ADC_COUNTS_TO_MV(rssi_adc_counts);
}

void __time_critical_func(ADSBee::ReadSignalStrengthMilliVoltsNonBlockingBegin)() {
    // The ADC is shared with the main loop (noise floor sampling, TL readback, temperature). If one of its conversions
    // is in progress, wait for it to finish (a single conversion is ~2us) so that START_ONCE isn't dropped and
    // ReadSignalStrengthMilliVoltsNonBlockingComplete() doesn't hand back the other conversion's result as this
    // packet's RSSI. Bounded so a wedged ADC can't hang the ISR.
    for (uint16_t spin = 0; spin < kADCReadySpinLimit && !(adc_hw->cs & ADC_CS_READY_BITS); spin++) {
        tight_loop_contents();
    }
    adc_select_input(config_.rssi_adc_input);
    hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
}

int __time_critical_func(ADSBee::ReadSignalStrengthMilliVoltsNonBlockingComplete)() {
    while (!(adc_hw->cs & ADC_CS_READY_BITS)) {
        // Wait for conversion to complete.
    }
    int rssi_adc_counts = adc_hw->result;
    return ADC_COUNTS_TO_MV(rssi_adc_counts);
}

int ADSBee::ReadSignalStrengthdBmBlocking() { return DetectorMilliVoltsTodBm(ReadSignalStrengthMilliVoltsBlocking()); }

int ADSBee::ReadTLMilliVolts() {
    // Read back the low level TL bias output voltage.
    adc_select_input(config_.tl_adc_input);
    tl_adc_counts_ = adc_read();
    return ADCCountsToMilliVolts(tl_adc_counts_);
}

bool ADSBee::SetTLOffsetMilliVolts(int tl_offset_mv) {
    if (tl_offset_mv > kTLOffsetMaxMV || tl_offset_mv < kTLOffsetMinMV) {
        CONSOLE_ERROR("ADSBee::SetTLOffsetMilliVolts",
                      "Unable to set tl_offset_mv_ to %d, outside of permissible range %d-%d.\r\n", tl_offset_mv,
                      kTLOffsetMinMV, kTLOffsetMaxMV);
        return false;
    }
    tl_offset_mv_ = tl_offset_mv;

    return true;
}

void ADSBee::StartTLLearning(uint16_t tl_learning_num_cycles, uint16_t tl_learning_start_temperature_mv,
                             uint16_t tl_min_mv, uint16_t tl_max_mv) {
    tl_learning_temperature_mv_ = tl_learning_start_temperature_mv;
    tl_learning_temperature_step_mv_ = tl_learning_start_temperature_mv / tl_learning_num_cycles;
    tl_learning_cycle_start_timestamp_ms_ = get_time_since_boot_ms();
}

/** Private Functions **/

void ADSBee::IngestAndForwardPackets() {
    // Ingest new Mode S packets into dictionary and report them.
    DecodedModeSPacket decoded_packet;
    // Explicitly count the number of packets to process so that we don't get stuck in this loop if the decoder
    // keeps outputting packets.
    uint16_t num_decoded_mode_s_packets = decoder.decoded_mode_s_packet_out_queue.Length();
    for (uint16_t i = 0;
         i < num_decoded_mode_s_packets && decoder.decoded_mode_s_packet_out_queue.Dequeue(decoded_packet); i++) {
        // CONSOLE_INFO("ADSBee::IngestAndForwardPackets", "\tdf=%d icao_address=0x%06x",
        // decoded_packet.downlink_format,
        //              decoded_packet.icao_address);

        if (aircraft_dictionary.IngestDecodedModeSPacket(decoded_packet)) {
            // Packet was used to update the dictionary or was silently ignored (but presumed to be valid).
            FlashStatusLED();
            // CONSOLE_INFO("ADSBee::IngestAndForwardPackets", "\taircraft_dictionary: %d aircraft",
            //              aircraft_dictionary.GetNumAircraft());
        }
        // Check for any new valid packets and push all decoded packets to the reporting queue, even if the aircraft
        // dictionary didn't know what to do with them.
        if (decoded_packet.is_valid) {
            if (comms_manager.mode_s_packet_reporting_queue.IsFull()) {
                comms_manager.ForceFlushRawPackets();
            }
            if (!comms_manager.mode_s_packet_reporting_queue.Enqueue(decoded_packet.raw)) {
                CONSOLE_ERROR("ADSBee::IngestAndForwardPackets", "Mode S packet reporting queue overflowed.");
            }
        }
    }

    // NOTE: The UAT packet queues are updated on this core, so we don't need to count the number of packets to
    // process before dequeueing.

    // Ingest new UAT packets into the dictionary and report them.
    RawUATADSBPacket uat_adsb_packet;
    while (raw_uat_adsb_packet_queue.Dequeue(uat_adsb_packet)) {
        DecodedUATADSBPacket decoded_uat_adsb_packet = DecodedUATADSBPacket(uat_adsb_packet);
        if (!decoded_uat_adsb_packet.is_valid) {
            CONSOLE_ERROR("ADSBee::IngestAndForwardPackets", "Invalid UAT ADS-B packet received.");
            continue;
        }

        if (aircraft_dictionary.IngestDecodedUATADSBPacket(decoded_uat_adsb_packet)) {
            // Packet was used to update the dictionary or was silently ignored (but presumed to be valid).
            // NOTE: Pushing to the reporting queue here means that we only will report validated packets!
            // comms_manager.uat_adsb_packet_reporting_queue.Enqueue(uat_adsb_packet);
            CONSOLE_INFO("ADSBee::IngestAndForwardPackets", "\taircraft_dictionary: %d aircraft",
                         aircraft_dictionary.GetNumAircraft());
        }
        // Push all decoded packets to the reporting queue, even if the aircraft dictionary didn't know what to do
        // with them.
        if (comms_manager.uat_adsb_packet_reporting_queue.IsFull()) {
            comms_manager.ForceFlushRawPackets();
        }
        if (!comms_manager.uat_adsb_packet_reporting_queue.Enqueue(uat_adsb_packet)) {
            CONSOLE_ERROR("ADSBee::IngestAndForwardPackets", "UAT ADS-B packet reporting queue overflowed.");
        }
    }

    // Report all UAT uplink packets.
    RawUATUplinkPacket uat_uplink_packet;
    while (raw_uat_uplink_packet_queue.Dequeue(uat_uplink_packet)) {
        // We don't do anything with uplink packets other than report them directly.
        if (comms_manager.uat_uplink_packet_reporting_queue.IsFull()) {
            comms_manager.ForceFlushRawPackets();
        }
        if (!comms_manager.uat_uplink_packet_reporting_queue.Enqueue(uat_uplink_packet)) {
            CONSOLE_ERROR("ADSBee::IngestAndForwardPackets", "UAT uplink packet reporting queue overflowed.");
        }
    }

    // Ingest Broadcast Remote ID (drone) packets pulled from the ESP32 into the aircraft dictionary. The ESP32 already
    // ingested these into its own dictionary for network output; here they populate the RP2040 dictionary so drones
    // appear in the serial reporting outputs (CSBee / GDL90 / MAVLINK / Aircraft JSON) alongside Mode S and UAT
    // traffic.
    RawRemoteIDPacket remote_id_packet;
    while (raw_remote_id_packet_queue.Dequeue(remote_id_packet)) {
        aircraft_dictionary.IngestRawRemoteIDPacket(remote_id_packet);
    }
}

void ADSBee::MLATCounterInit() {
    /**
     * MLAT Counter
     * A 48-MHz MLAT counter is synthesized from the 125MHz system clock using a the 24-bit SysTick timer.
     */
    /** MLAT Counter **/
    // Enable the MLAT timer using the 24-bit SysTick timer connected to the 125MHz processor clock.
    // SysTick Control and Status Register
    systick_hw->csr = 0b110;  // Source = Processor Clock, TickInt = Enabled, Counter = Disabled.
    // SysTick Reload Value Register
    systick_hw->rvr = 0xFFFFFF;  // Use the full 24 bit span of the timer value register.
    // 0xFFFFFF = 16777215 counts @ 125MHz = approx. 0.134 seconds.
    // Call the OnSysTickWrap function every time the SysTick timer hits 0.
    exception_set_exclusive_handler(SYSTICK_EXCEPTION, on_systick_exception);
    // Let the games begin!
    systick_hw->csr |= 0b1;  // Enable the counter.
    // Set the priority of the Systick exception to kMLATCounterWrapInterruptPriority.
    exception_set_priority(SYSTICK_EXCEPTION, kMLATCounterWrapInterruptPriority);

    /**
     * MLAT Jitter Offset Counter
     * There is timing jitter between the beginning of a message decode and the OnDemodBegin() interrupt firing. The
     * DMA peripheral is used to capture the timestamp of a PIO state machines first push onto its TX FIFO, allowing
     * the full 24-bit timestamp captured during the OnDemodBegin() interval to be de-jittered. a PIO state machine.
     */

    // PWM slice 5 is used for LEVEL_PWM, anything else is fine to use for the MLAT jitter counter.
    mlat_jitter_pwm_slice_ = pwm_gpio_to_slice_num(bsp.r1090_pulses_pin);  // Use pulses pin for slice 1.
    // Count clk_sys undivided and scale with MLATJitterCountsTo48MHzCounts(): the 8.4 fixed-point PWM divider can't
    // hold 125/48 exactly. The 16-bit counter wraps every 524us, well above the <30us jitter corrections.
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int_frac(&config, 1, 0);
    pwm_config_set_wrap(&config, 0xFFFF);             // Use the full 16-bit span.
    pwm_init(mlat_jitter_pwm_slice_, &config, true);  // Start immediately.

    // Enable a DMA DREQ for each pulse sampler PIO TX FIFO. This is used to capture the IRQ jitter offset counter
    // when a decode interval begins (32 bits into a message).
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        mlat_jitter_dma_channel_[sm_index] =
            dma_claim_unused_channel(true);  // Claim a DMA channel for each state machine.
        dma_channel_config config = dma_channel_get_default_config(mlat_jitter_dma_channel_[sm_index]);
        channel_config_set_read_increment(&config, false);            // Don't increment the read address.
        channel_config_set_write_increment(&config, false);           // Don't increment the write address.
        channel_config_set_transfer_data_size(&config, DMA_SIZE_16);  // Transfer 16 bits (width of PWM timer).
        // Transfer 16 bits once from the PWM counter to the jitter counter counts for the given state machine.
        channel_config_set_dreq(&config, DREQ_PIO1_TX0 + sm_index);  // Use the PIO1 TX FIFO DREQ for each SM.
        dma_channel_configure(mlat_jitter_dma_channel_[sm_index], &config, &mlat_jitter_counts_on_fifo_pull_[sm_index],
                              &pwm_hw->slice[mlat_jitter_pwm_slice_].ctr, 1, false);
    }
}

void ADSBee::PIOInit() {
    /** PREAMBLE MATCHER PIO **/
    // Calculate the PIO clock divider.
    float preamble_matcher_div = (float)clock_get_hz(clk_sys) / kPreambleMatcherFreqHz;
    // Build the demod pin -> state machine lookup table used by OnDemodBegin().
    for (uint16_t gpio = 0; gpio < NUM_BANK0_GPIOS; gpio++) {
        demod_pin_to_sm_index_[gpio] = kNoDemodStateMachine;
    }
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        if (config_.demod_pins[sm_index] < NUM_BANK0_GPIOS) {
            demod_pin_to_sm_index_[config_.demod_pins[sm_index]] = sm_index;
        }
    }
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        // The matchers take candidate preamble edges in turn: the first one starts, the others wait for theirs.
        bool make_sm_wait = sm_index > 0;
        preamble_matcher_program_init(config_.preamble_matcher_pio, preamble_matcher_sm_[sm_index],
                                      preamble_matcher_offset_, config_.pulses_pin, config_.demod_pins[sm_index],
                                      preamble_matcher_div, make_sm_wait);

        // Handle GPIO interrupts (for marking beginning of demod interval).
        gpio_set_irq_enabled_with_callback(config_.demod_pins[sm_index], GPIO_IRQ_EDGE_RISE, true, on_demod_pin_change);

        // Fill the sampler's TX FIFO: its pull at the start of a capture triggers the timestamp DMA.
        while (!pio_sm_is_tx_fifo_full(config_.pulse_sampler_pio, pulse_sampler_sm_[sm_index])) {
            pio_sm_put(config_.pulse_sampler_pio, pulse_sampler_sm_[sm_index],
                       0xFFFFFFFF);  // Non-blocking put.
        }
    }

    // PIO0 IRQ0: a preamble matcher is about 43us into a message (RSSI sample, noise and length checks). The flag
    // also holds the matcher until its capture is finished.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        pio_set_irq0_source_enabled(config_.preamble_matcher_pio,
                                    static_cast<pio_interrupt_source>(pis_interrupt0 + sm_index), true);
    }
    irq_set_exclusive_handler(config_.preamble_matcher_irq, on_demod_mid);
    irq_set_priority(config_.preamble_matcher_irq, kCaptureEventInterruptPriority);
    irq_set_enabled(config_.preamble_matcher_irq, true);

    /** PULSE SAMPLER PIO **/
    float pulse_sampler_div = (float)clock_get_hz(clk_sys) / kPulseSamplerFreqHz;
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        PIO pio = config_.pulse_sampler_pio;
        uint sm = pulse_sampler_sm_[sm_index];
        pulse_sampler_program_init(pio, sm, pulse_sampler_offset_, config_.pulses_pin, config_.demod_pins[sm_index],
                                   pulse_sampler_div);
        // DMA: sampler RX FIFO -> capture buffer, one interrupt per complete capture.
        if (sample_dma_channel_[sm_index] < 0) {
            sample_dma_channel_[sm_index] = dma_claim_unused_channel(true);
        }
        dma_channel_config dma_config = dma_channel_get_default_config(sample_dma_channel_[sm_index]);
        channel_config_set_read_increment(&dma_config, false);
        channel_config_set_write_increment(&dma_config, true);
        channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
        channel_config_set_dreq(&dma_config, pio_get_dreq(pio, sm, false));
        dma_channel_configure(sample_dma_channel_[sm_index], &dma_config, sample_buf_[sm_index], &pio->rxf[sm],
                              mode_s_soft_demod::kCaptureWords, true);
        dma_channel_set_irq1_enabled(sample_dma_channel_[sm_index], true);
        capture_active_[sm_index] = false;
    }
    // Timer alarm that ends the capture of a 56 bit message early.
    if (short_capture_alarm_num_ < 0) {
        short_capture_alarm_num_ = hardware_alarm_claim_unused(true);
    }
    short_capture_sm_index_ = -1;
    hardware_alarm_set_callback(short_capture_alarm_num_, on_short_capture_alarm);
    irq_set_priority(TIMER_IRQ_0 + short_capture_alarm_num_, kCaptureDemodInterruptPriority);
    irq_set_exclusive_handler(DMA_IRQ_1, on_capture_complete);
    irq_set_priority(DMA_IRQ_1, kCaptureDemodInterruptPriority);
    irq_set_enabled(DMA_IRQ_1, true);

    // The message begin interrupt (demod pin rising edge) shares the priority of the mid-message interrupt: both
    // must preempt the demodulation of an earlier capture.
    irq_set_priority(config_.demod_pin_irq, kCaptureEventInterruptPriority);
}

void ADSBee::PIOEnable() {
    // Enable the MLAT jitter counter DMA transfers.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        dma_channel_start(mlat_jitter_dma_channel_[sm_index]);
    }
    // Samplers first, so each one is parked at idle before its preamble matcher can raise the demod pin.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        pio_sm_set_enabled(config_.pulse_sampler_pio, pulse_sampler_sm_[sm_index], true);
    }
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        pio_sm_set_enabled(config_.preamble_matcher_pio, preamble_matcher_sm_[sm_index], true);
    }
}

void ADSBee::PruneAircraftDictionary() {
    // Prune aircraft dictionary. Need to do this up front so that we don't end up with a negative timestamp delta
    // caused by packets being ingested more recently than the timestamp we take at the beginning of this function.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - last_aircraft_dictionary_update_timestamp_ms_ > config_.aircraft_dictionary_update_interval_ms) {
        aircraft_dictionary.Update(timestamp_ms);
        if (esp32.IsEnabled()) {
            // Send fresh aircraft dictionary stats to ESPS32.
            esp32.Write(ObjectDictionary::kAddrAircraftDictionaryMetrics, aircraft_dictionary.metrics,
                        true);  // require ACK.
        }
        // Add the fresh metrics values to the pile used for TL learning.
        // If learning, add the number of valid packets received to the pile used for trigger level learning.
        if (tl_learning_temperature_mv_ > 0) {
            tl_learning_num_valid_packets_ += (aircraft_dictionary.metrics.valid_squitter_frames +
                                               aircraft_dictionary.metrics.valid_extended_squitter_frames);
        }
        last_aircraft_dictionary_update_timestamp_ms_ = timestamp_ms;

#ifdef DEBUG_ISR_TIMING
        // Snapshot and reset the ISR timing accumulators. Counts are system clock cycles (125MHz).
        uint32_t isr_count = isr_count_;
        uint32_t isr_sum_counts = isr_duration_sum_counts_;
        uint16_t isr_max_counts = isr_duration_max_counts_;
        isr_count_ = 0;
        isr_duration_sum_counts_ = 0;
        isr_duration_max_counts_ = 0;
        CONSOLE_WARNING("ADSBee::PruneAircraftDictionary", "Demodulate: %lu calls, avg %lu us, max %u us.", isr_count,
                        isr_count > 0 ? isr_sum_counts / isr_count / 125 : 0, isr_max_counts / 125);
#endif
    }
}

void ADSBee::Update1090LED() {
    uint32_t timestamp_ms = get_time_since_boot_ms();
    // Turn off the 1090 LED if it's been on for long enough.
    if (timestamp_ms - led_on_timestamp_ms_ > led_on_duration_ms_) {
        gpio_put(config_.r1090_led_pin, 0);
        led_force_blink_ = false;
        led_on_duration_ms_ = kStatusLEDOnMs;  // Restore the default packet-flash duration.
    }
}

void ADSBee::UpdateNoiseFloor() {
    // Periodically sample the RSSI line between packets and low-pass filter it to approximate the noise floor.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (timestamp_ms - noise_floor_last_sample_timestamp_ms_ < kNoiseFloorADCSampleIntervalMs) {
        return;
    }
    noise_floor_last_sample_timestamp_ms_ = timestamp_ms;

    // Refresh the trigger level PWM duty cycle every pass, even when the sample below gets skipped, so that a change
    // to the TL offset always takes effect promptly.
    tl_pwm_count_ = (noise_floor_mv_ + tl_offset_mv_) * kTLMaxPWMCount / kVDDMV;
    pwm_set_chan_level(tl_pwm_slice_, tl_pwm_chan_, tl_pwm_count_);

    // Snapshot the demod begin timestamp before checking the demod pins so that a demodulation starting anywhere
    // between here and the end of the ADC conversion is caught by the comparison after the sample.
    uint32_t demod_begin_us = last_demod_begin_timestamp_us_;

    // Never sample while a demodulation is in progress: the RSSI line is sitting at the packet's power level, and the
    // ISR owns the ADC (OnDemodMid() samples the packet's RSSI).
    if (gpio_get_all() & demod_pins_mask_) {
        return;
    }

    // The RC filter on the RSSI line takes tens of microseconds to decay after a packet ends, so also skip samples
    // taken shortly after a demodulation began. If the receiver is so busy that this guard never clears, drop it
    // (the demod pin check above still applies) rather than letting the estimate stall.
    bool guard_active = timestamp_ms - noise_floor_last_accepted_timestamp_ms_ < kNoiseFloorMaxHoldMs;
    if (guard_active && time_us_32() - demod_begin_us < kNoiseFloorDemodGuardUs) {
        return;
    }

    int32_t sample_mv = ReadSignalStrengthMilliVoltsBlocking();

    // Discard the sample if a demodulation began during the conversion: the RSSI line was rising and the ISR's
    // conversion collided with this one.
    if (last_demod_begin_timestamp_us_ != demod_begin_us) {
        return;
    }

    if (!noise_floor_initialized_) {
        // Seed the filter with the first clean sample instead of slewing up from 0mV.
        noise_floor_mv_fp_ = sample_mv << kNoiseFloorFixedPointShift;
        noise_floor_initialized_ = true;
    } else {
        int32_t delta_mv = sample_mv - noise_floor_mv_;
        if (delta_mv > kNoiseFloorOutlierMV || delta_mv < -kNoiseFloorOutlierMV) {
            // Far from the floor: above it, a pulse such as a packet whose preamble didn't trigger a demodulation;
            // below it, a glitched conversion (the floor is the minimum the detector can output, so a sample well
            // below it can't be real). Reject it unless samples have been out of range for long enough that the floor
            // itself moved (e.g. bias tee powered an external LNA on or off), in which case let it through (and keep
            // the counter saturated) so the estimate can track. The counter clears once the estimate catches up and
            // samples fall back within range.
            if (noise_floor_consecutive_outliers_ < kNoiseFloorMaxConsecutiveOutliers) {
                noise_floor_consecutive_outliers_++;
                return;
            }
        } else {
            noise_floor_consecutive_outliers_ = 0;
        }
        // Exponential low-pass filter in fixed point. Division (not shift) keeps negative deltas well-defined.
        noise_floor_mv_fp_ +=
            ((sample_mv << kNoiseFloorFixedPointShift) - noise_floor_mv_fp_) / (1 << kNoiseFloorFilterShift);
    }
    noise_floor_mv_ = noise_floor_mv_fp_ >> kNoiseFloorFixedPointShift;
    noise_floor_last_accepted_timestamp_ms_ = timestamp_ms;
}

void ADSBee::UpdateTLLearning() {
    // Update trigger level learning if it's active.
    uint32_t timestamp_ms = get_time_since_boot_ms();
    if (tl_learning_temperature_mv_ > 0 &&
        timestamp_ms - tl_learning_cycle_start_timestamp_ms_ > kTLLearningIntervalMs) {
        // Trigger level learning is active and due for an update.
        // Is this neighbor (current value for tl_mv) worth traversing to?
        float valid_packet_ratio = (tl_learning_num_valid_packets_ - tl_learning_prev_num_valid_packets_) /
                                   MAX(tl_learning_prev_num_valid_packets_, 1);  // Avoid divide by zero.
        float random_weight =
            static_cast<int16_t>(get_rand_32()) * kInt16MaxRecip * 0.25;  // Random value from [-0.25,0.25].
        if (valid_packet_ratio + random_weight > 0.0f) {
            // Transition to neighbor TL value.
            tl_learning_prev_tl_offset_mv_ = tl_offset_mv_;
            tl_learning_prev_num_valid_packets_ = tl_learning_num_valid_packets_;
        }
        // Else keep existing TL value in tl_learning_prev_tl_mv_.

        // DO STUFF HERE
        // Find a new neighbor by stepping trigger level with random value from [-1.0, 1.0] * temperature.
        uint16_t new_tl_offset_mv =
            tl_offset_mv_ + static_cast<int16_t>(get_rand_32()) * tl_learning_temperature_mv_ / INT16_MAX;
        if (new_tl_offset_mv > tl_learning_max_offset_mv_) {
            tl_offset_mv_ = tl_learning_max_offset_mv_;
        } else if (new_tl_offset_mv < tl_learning_min_offset_mv_) {
            tl_offset_mv_ = tl_learning_min_offset_mv_;
        } else {
            tl_offset_mv_ = new_tl_offset_mv;
        }

        // Update learning temperature. Decrement by the annealing temperature step or set to 0
        // if learning is complete.
        if (tl_learning_temperature_mv_ > tl_learning_temperature_step_mv_) {
            // Not done learning: step the trigger level learning temperature.
            tl_learning_temperature_mv_ -= tl_learning_temperature_step_mv_;
        } else {
            // Done learning: take the current best trigger level and yeet outta here.
            tl_learning_temperature_mv_ = 0;  // Set learning temperature to 0 to finish learnign trigger level.
            tl_offset_mv_ = tl_learning_prev_tl_offset_mv_;  // Set trigger level to the best value we've seen so far.
        }

        // Store timestamp as start of trigger learning cycle so we know when to come back.
        tl_learning_cycle_start_timestamp_ms_ = timestamp_ms;
        tl_learning_num_valid_packets_ = 0;  // Start the counter over.
    }
}

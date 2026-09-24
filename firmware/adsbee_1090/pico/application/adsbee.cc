#include "adsbee.hh"

#include <hardware/structs/systick.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/exception.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "stdio.h"  // for printing
// #include "hardware/dma.h"
#include "capture.pio.h"
#include "hal.hh"
#include "hardware/irq.h"
#include "mode_s_packet_decoder.hh"
#include "pico/binary_info.h"
#include "spi_coprocessor.hh"

// #include <charconv>
#include <string.h>  // for strcat

#include "comms.hh"          // For debug prints.
#include "gnss_interface.hh"  // For the gnss receiver instance.

// Uncomment the line below to enable preamble detector debugging on recovered_clk.
#define DEBUG_PREAMBLE_DETECTOR
// Uncomment the line below to enable demodulator debugging on recovered_clk.
// #define DEBUG_DEMODULATOR
// Uncomment to measure OnDemodComplete() ISR duration and print avg/max once per second (default log level).
// #define DEBUG_ISR_TIMING
// Uncomment to use the self-parking message demodulator PIO program (see capture.pio). The demodulator pads and parks
// itself at the end of each message, so OnDemodComplete() only drains the FIFO instead of padding bit-by-bit and
// restarting the state machine. Requires on-target validation before being made the default.
// #define DEMODULATOR_SELF_PARKING

#ifdef DEMODULATOR_SELF_PARKING
#define MESSAGE_DEMODULATOR_PROGRAM              message_demodulator_self_parking_program
#define MESSAGE_DEMODULATOR_PROGRAM_INIT         message_demodulator_self_parking_program_init
#define MESSAGE_DEMODULATOR_OFFSET_INITIAL_ENTRY message_demodulator_self_parking_offset_initial_entry
// Maximum number of PC polls to wait for the demodulator to finish padding and park after the demod pin drops
// (padding takes ~1.4us after the pin drops; each poll is a handful of system clock cycles).
static constexpr uint16_t kDemodulatorParkSpinLimit = 256;
#else
#define MESSAGE_DEMODULATOR_PROGRAM              message_demodulator_program
#define MESSAGE_DEMODULATOR_PROGRAM_INIT         message_demodulator_program_init
#define MESSAGE_DEMODULATOR_OFFSET_INITIAL_ENTRY message_demodulator_offset_initial_entry
#endif  // DEMODULATOR_SELF_PARKING

// Uncomment this to hold the status LED on for 5 seconds if the watchdog commanded a reboot.
// #define WATCHDOG_REBOOT_WARNING

#define ADC_COUNTS_TO_MV(adc_counts) ((adc_counts) * kVDDMV / 4095)
#define MLAT_SYSTEM_CLOCK_RATIO      48 / 125
// Scales 125MHz system clock into a 48MHz counter.
static const uint32_t kMLATWrapCounterIncrement = (1 << 24) * MLAT_SYSTEM_CLOCK_RATIO;
static constexpr float kMLATSystemClockDiv = 125.0f / 48.0f;  // Ratio of 48MHz MLAT clock to 125MHz system clock.

constexpr float kPreambleDetectorFreqHz = 48e6;    // Running at 48MHz (24 clock cycles per half bit).
constexpr float kMessageDemodulatorFreqHz = 48e6;  // Run at 48 MHz to demodulate bits at 1Mbps.

static const uint16_t kPreambleDetectorFIFODepthWords = 4;

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

void __time_critical_func(on_demod_complete)() { isr_access->OnDemodComplete(); }

/** End pass-through functions for public access **/

ADSBee::ADSBee(ADSBeeConfig config_in) {
    config_ = config_in;

    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        preamble_detector_sm_[sm_index] = pio_claim_unused_sm(config_.preamble_detector_pio, true);
        message_demodulator_sm_[sm_index] = pio_claim_unused_sm(config_.message_demodulator_pio, true);
    }

    preamble_detector_offset_ = pio_add_program(config_.preamble_detector_pio, &preamble_detector_program);
    message_demodulator_offset_ = pio_add_program(config_.message_demodulator_pio, &MESSAGE_DEMODULATOR_PROGRAM);

    // Put IRQ parameters into the global scope for the on_demod_complete ISR.
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
        pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], false);
        pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], false);
    }

    // Stop and unclaim DMA channels used for MLAT jitter counters
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        dma_channel_abort(mlat_jitter_dma_channel_[sm_index]);
        if (dma_channel_is_claimed(mlat_jitter_dma_channel_[sm_index])) {
            dma_channel_unclaim(mlat_jitter_dma_channel_[sm_index]);
        }
    }

    // Disable interrupts
    irq_set_enabled(config_.preamble_detector_demod_complete_irq, false);
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
        if (subg_radio.Update()) {
            // Sub-GHz radio comms successful.
            subg_radio_last_update_timestamp_ms_ = get_time_since_boot_ms();
        } else {
            // Sub-GHz radio comms failed. Log an error and restart if enough time has elapsed since the last successful
            // Sub-GHz radio comms.
            CONSOLE_ERROR("ADSBee::Update", "Failed to update sub-GHz radio.");

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
    return (mlat_counter_wraps_ + ((0xFFFFFF - systick_hw->cvr) * MLAT_SYSTEM_CLOCK_RATIO)) &
           (UINT64_MAX >> (64 - num_bits));
}

uint64_t ADSBee::GetMLAT12MHzCounts(uint16_t num_bits) {
    // Piggyback off the higher resolution 48MHz timer function.
    return GetMLAT48MHzCounts(50) >> 2;  // Divide 48MHz counter by 4, widen the mask by 2 bits to compensate.
}

uint16_t __time_critical_func(ADSBee::GetMLATJitterPWMSliceCounts)() {
    // Returns the current value of the MLAT jitter counter PWM slice.
    return pwm_hw->slice[mlat_jitter_pwm_slice_].ctr;
}

int __time_critical_func(ADSBee::GetNoiseFloordBm)() { return AD8313MilliVoltsTodBm(noise_floor_mv_); }

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

    ReadSignalStrengthMilliVoltsNonBlockingBegin();  // Kick off ADC read.
}

void __time_critical_func(ADSBee::OnDemodComplete)() {
#ifdef DEBUG_ISR_TIMING
    uint16_t isr_start_counts = GetMLATJitterPWMSliceCounts();
#endif
    int signal_strength_dbm = AD8313MilliVoltsTodBm(ReadSignalStrengthMilliVoltsNonBlockingComplete());

    // Figure out which state machines were triggered and get things set up to read packets from them. Don't stop
    // them, let them finish chewing since they run at a lower clock rate.
    bool sm_triggered[bsp.r1090_num_demod_state_machines] = {false};
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        if (!pio_interrupt_get(config_.preamble_detector_pio, sm_index)) {
            // Skip state machines that didn't finish demodulating.
            continue;
        }
        sm_triggered[sm_index] = true;
#ifndef DEMODULATOR_SELF_PARKING
        pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], false);
#endif
    }

    // Empty the triggered state machines' RX FIFOs one by one and create RawModeSPacket objects from them.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        if (!sm_triggered[sm_index]) {
            continue;
        }

        // Read the RSSI level of the current packet.
        rx_packet_[sm_index].sigs_dbm = signal_strength_dbm;
        rx_packet_[sm_index].sigq_db = rx_packet_[sm_index].sigs_dbm - GetNoiseFloordBm();
        rx_packet_[sm_index].source = sm_index;  // Record this state machine as the source of the packet.
        // Get the difference between the beginning of the demodulation period and the first FIFO push. The FIFO
        // push does not have jitter relative to the preamble but needs to be anchored to a full 24-bit timer value,
        // since it's from a 16-bit PWM peripheral.
        uint16_t& a = mlat_jitter_counts_on_fifo_pull_[sm_index];
        uint16_t& b = mlat_jitter_counts_on_demod_begin_[sm_index];
        uint16_t mlat_jitter_correction = b >= a ? b - a : (0xFFFF - a) + b;
        rx_packet_[sm_index].mlat_48mhz_64bit_counts -= mlat_jitter_correction;

        // Clear the transponder packet buffer.
        memset((void*)rx_packet_[sm_index].buffer, 0xFF, RawModeSPacket::kMaxPacketLenWords32 * sizeof(uint32_t));

        bool demodulator_parked = false;
#ifdef DEMODULATOR_SELF_PARKING
        // The demodulator pads its own partial word and parks at initial_entry once the demod pin drops. Wait
        // (briefly) for it to get there. If it doesn't (e.g. the RX FIFO filled up with noise before the pin dropped
        // and the SM is stalled on a push), fall back to the manual pad / restart path below.
        uint parked_pc = message_demodulator_offset_ + MESSAGE_DEMODULATOR_OFFSET_INITIAL_ENTRY;
        for (uint16_t spin = 0; spin < kDemodulatorParkSpinLimit; spin++) {
            if (pio_sm_get_pc(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]) == parked_pc) {
                demodulator_parked = true;
                break;
            }
        }
        if (!demodulator_parked) {
            pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], false);
        }
#endif  // DEMODULATOR_SELF_PARKING

        // If the FIFO is full, we got a bunch of garbage bits after ending the demodulation, but that's OK, we can
        // chop off the rest of them and see if we got a valid message. If it's not full, we need to carefully feed
        // in 0 bits until we join the last partial word of the message together with the rest of it.
        if (!demodulator_parked &&
            !pio_sm_is_rx_fifo_full(config_.message_demodulator_pio, message_demodulator_sm_[sm_index])) {
            // Shift 0 bits into the input shift register until it pushes into the RX FIFO.
            uint16_t rx_fifo_level =
                pio_sm_get_rx_fifo_level(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);
            while (pio_sm_get_rx_fifo_level(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]) ==
                   rx_fifo_level) {
                // NOTE: Shift in bits one by one since for some reason the autopush doesn't work if we shift in a
                // bunch of bits at the same time.
                pio_sm_exec_wait_blocking(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
                                          pio_encode_in(pio_null, 1));
            }

            // // Enqueue any partially complete 32-bit word onto the RX FIFO.
            // pio_sm_exec_wait_blocking(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
            //                           pio_encode_push(false, true));
        }

        // Pull all words out of the RX FIFO.
        volatile uint16_t packet_num_words =
            pio_sm_get_rx_fifo_level(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);
        if (packet_num_words > RawModeSPacket::kMaxPacketLenWords32) {
            // Packet length is invalid; dump everything and try again next time.
            // Only enable this print for debugging! Printing from the interrupt causes the USB library to crash.
            // CONSOLE_WARNING("ADSBee::OnDemodComplete", "Received a packet with %d 32-bit words, expected maximum
            // of %d.",
            //                 packet_num_words, RawModeSPacket::kExtendedSquitterPacketNumWords32);
            // pio_sm_clear_fifos(config_.message_demodulator_pio, message_demodulator_sm_);
            packet_num_words = RawModeSPacket::kMaxPacketLenWords32;
        }
        // Track that we attempted to demodulate something.
        aircraft_dictionary.Record1090Demod(sm_index);
        // Create a RawModeSPacket and push it onto the queue.
        for (uint16_t i = 0; i < packet_num_words; i++) {
            rx_packet_[sm_index].buffer[i] =
                pio_sm_get(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);
            if (packet_num_words > RawModeSPacket::kSquitterPacketNumWords32 - 1 && i == packet_num_words - 1) {
                // Packet has enough bits to be plausibly interesting, and we're about to read the last word.
                // Trim off extra ingested bit from last word in the packet.
                // FIXME: Extra bit is ingested non-deterministically, depending on whether there was a HI bit right
                // after the demod interval went LO. This happens because the demodulator state machine ends up in a
                // state where it's waiting for an edge regardless of whether the demod interval is active. Our
                // interrupts are too slow (1-3us) to get to the PIO state machine in time to stop it from doing
                // this. rx_packet_[sm_index].buffer[i] >>= 1; Mask and left align final word based on bit length.
                switch (packet_num_words) {
                    case RawModeSPacket::kSquitterPacketNumWords32:
                    case RawModeSPacket::kSquitterPacketNumWords32 + 1:
                        aircraft_dictionary.Record1090RawSquitterFrame(sm_index);
                        rx_packet_[sm_index].buffer[i] = rx_packet_[sm_index].buffer[i] & 0xFFFFFF00;
                        rx_packet_[sm_index].buffer_len_bytes = RawModeSPacket::kSquitterPacketLenBytes;
                        if (!decoder.raw_mode_s_packet_in_queue.Enqueue(rx_packet_[sm_index])) {
                            r1090_packet_queue_overflowed_ = true;
                        }
                        break;
                    case RawModeSPacket::kExtendedSquitterPacketNumWords32:
                    case RawModeSPacket::kExtendedSquitterPacketNumWords32 + 1:
                        aircraft_dictionary.Record1090RawExtendedSquitterFrame(sm_index);
                        rx_packet_[sm_index].buffer[i] = rx_packet_[sm_index].buffer[i] & 0xFFFF0000;
                        rx_packet_[sm_index].buffer_len_bytes = RawModeSPacket::kExtendedSquitterPacketLenBytes;
                        if (!decoder.raw_mode_s_packet_in_queue.Enqueue(rx_packet_[sm_index])) {
                            r1090_packet_queue_overflowed_ = true;
                        }
                        break;
                    default:
                        // Don't push partial packets.
                        // Printing to tinyUSB from within an interrupt causes crashes! Don't do it.
                        // CONSOLE_WARNING(
                        //     "ADSBee::OnDemodComplete",
                        //     "Unhandled case while creating RawModeSPacket, received packet with %d 32-bit
                        //     words.", packet_num_words);
                        break;
                }
            }
        }

        if (!demodulator_parked) {
            // Flush any remaining words in the RX FIFO. Any partial word left in the ISR is discarded by the restart
            // below.
            while (!pio_sm_is_rx_fifo_empty(config_.message_demodulator_pio, message_demodulator_sm_[sm_index])) {
                pio_sm_get(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);
            }

            // Reset the demodulator state machine to wait for the next decode interval, then enable it.
            pio_sm_restart(config_.message_demodulator_pio,
                           message_demodulator_sm_[sm_index]);  // Reset shift counters etc.
            uint demodulator_program_start = message_demodulator_offset_ + MESSAGE_DEMODULATOR_OFFSET_INITIAL_ENTRY;
            pio_sm_exec_wait_blocking(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
                                      pio_encode_jmp(demodulator_program_start));  // Jump to beginning of program.
            pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], true);
        }

        // Stuff the preamble detector TX FIFO full of garbage so that the preamble detector can use a pull to
        // signal the demod interval beginning.
        while (!pio_sm_is_tx_fifo_full(config_.message_demodulator_pio, message_demodulator_sm_[sm_index])) {
            pio_sm_put(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
                       0xFFFFFFFF);  // Non-blocking put.
        }

        // Re-enable the MLAT jitter counter for this state machine.
        // Reset the write address but don't start the DMA channel yet.
        dma_channel_set_write_addr(mlat_jitter_dma_channel_[sm_index], &mlat_jitter_counts_on_fifo_pull_[sm_index],
                                   false);
        dma_channel_start(mlat_jitter_dma_channel_[sm_index]);

        // // Release the preamble detector from its wait state.
        // if (sm_index == bsp.r1090_high_power_demod_state_machine_index) {
        //     // High power state machine operates alone and doesn't need to wait for any other SM to complete. It
        //     // would normally be enabled by one of the interleaved well formed preamble detector state machines
        //     // refreshing, but doing it here brings it up a little quicker and allows it to catch a subsequent
        //     high
        //     // power packet if it comes in quickly.
        //     pio_sm_exec_wait_blocking(
        //         config_.preamble_detector_pio, preamble_detector_sm_[sm_index],
        //         pio_encode_jmp(preamble_detector_offset_ + preamble_detector_offset_waiting_for_first_edge));
        // }

        pio_interrupt_clear(config_.preamble_detector_pio, sm_index);
    }

#ifdef DEBUG_ISR_TIMING
    // 16-bit counter at 48MHz wraps every ~1.4ms, far longer than any ISR execution.
    uint16_t isr_duration_counts = GetMLATJitterPWMSliceCounts() - isr_start_counts;
    if (isr_duration_counts > isr_duration_max_counts_) {
        isr_duration_max_counts_ = isr_duration_counts;
    }
    isr_duration_sum_counts_ += isr_duration_counts;
    isr_count_++;
#endif
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

int ADSBee::ReadSignalStrengthdBmBlocking() { return AD8313MilliVoltsTodBm(ReadSignalStrengthMilliVoltsBlocking()); }

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
    // appear in the serial reporting outputs (CSBee / GDL90 / MAVLINK / Aircraft JSON) alongside Mode S and UAT traffic.
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
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, kMLATSystemClockDiv);
    pwm_config_set_wrap(&config, 0xFFFF);             // Use the full 16-bit span.
    pwm_init(mlat_jitter_pwm_slice_, &config, true);  // Start immediately.

    // Enable a DMA DREQ for each demodulator PIO TX FIFO. This is used to capture the IRQ jitter offset counter
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
    /** PREAMBLE DETECTOR PIO **/
    // Calculate the PIO clock divider.
    float preamble_detector_div = (float)clock_get_hz(clk_sys) / kPreambleDetectorFreqHz;
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
        // Only make the state machine wait to start if it's part of the round-robin group of well formed preamble
        // detectors.
        bool make_sm_wait = sm_index > 0;  // State machines 1, 2, 3 follow state machine 0 in round robin.
        // Initialize the program using the .pio file helper function
        preamble_detector_program_init(
            config_.preamble_detector_pio,                     // Use PIO block 0.
            preamble_detector_sm_[sm_index],                   // State machines 0-2
            preamble_detector_offset_ /* + starting_offset*/,  // Program startin offset.
            config_.pulses_pin,                                // Pulses pin (input).
            config_.demod_pins[sm_index],                      // Demod pin (output).
#ifdef DEBUG_PREAMBLE_DETECTOR
            config_.recovered_clk_pins[sm_index],  // Use the recovered clk pin for preamble detector debugging.
#else
            UINT16_MAX,  // Don't use the recovered clk pin for side set.
#endif                              // DEBUG_PREAMBLE_DETECTOR
            preamble_detector_div,  // Clock divisor (for 48MHz).
            make_sm_wait            // Whether state machine should wait for an IRQ to begin.
        );

        // Handle GPIO interrupts (for marking beginning of demod interval).
        gpio_set_irq_enabled_with_callback(config_.demod_pins[sm_index], GPIO_IRQ_EDGE_RISE, true, on_demod_pin_change);

        /** Set the second chip of the preamble sequence into the ISR: 10100000.
         * Each "10" is represented by a 1 (since we don't actually care too muh about the trailing 0). A "10" half
         * nibble is 1us. Each "00" is represented by two 0 bits, since each 0 bit is 500ns of 0's.
         * from preamble sequence to allow the demodulator more time to start up.
         *
         * We actually ignore the last 500ns of 0's and do a dumb delay, since this part tends to get "dirty" due to
         * the LEVEL filter drifting downwards during the long LO. Taking into account the initial idle filter, we
         * are matching on a pattern that looks something like: 0b001X1X000X, where X indicates 500ns of ignored
         * bits.
         */

        // mov isr null ; Clear ISR.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_mov(pio_isr, pio_null));
        // set x 0b11       ; ISR = 0b00000000000000000000000000000000
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_set(pio_x, 0b11));
        // in x 2           ; ISR = 0b00000000000000000000000000000011
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_x, 2));
        // in null 3        ; ISR = 0b00000000000000000000000000011000
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_null, 3));
        // in x 2           ; ISR = 0b00000000000000000000000001100011
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_x, 2));
        // in null 3        ; ISR = 0b00000000000000000000001100011000
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_null, 3));
        // mov x null   ; Clear scratch x.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_mov(pio_x, pio_null));

        // Use this instruction to verify preamble was formed correctly (pushes ISR to RX FIFO).
        // pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_push(false,
        // true)); while (true) {
        //     // Stall core 1 so we can peek at the RX FIFO registers.
        // }

        // Stuff the demodulator TX FIFO full of garbage so that the demodulator can use a pull to signal
        // the demod interval beginning.
        while (!pio_sm_is_tx_fifo_full(config_.message_demodulator_pio, message_demodulator_sm_[sm_index])) {
            pio_sm_put(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
                       0xFFFFFFFF);  // Non-blocking put.
        }
    }

    // Enable the DEMOD interrupt on PIO0_IRQ_0 - PIO0_IRQ_n, where n is the number of preamble detector state
    // machines.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        pio_set_irq0_source_enabled(config_.preamble_detector_pio,
                                    static_cast<pio_interrupt_source>(pis_interrupt0 + sm_index),
                                    true);  // PIO0 state machine 0+i
    }

    // Handle PIO0 IRQ0.
    irq_set_exclusive_handler(config_.preamble_detector_demod_complete_irq, on_demod_complete);
    irq_set_enabled(config_.preamble_detector_demod_complete_irq, true);
    irq_set_priority(config_.preamble_detector_demod_complete_irq, kDemodCompleteInterruptPriority);

    /** MESSAGE DEMODULATOR PIO **/
    float message_demodulator_div = (float)clock_get_hz(clk_sys) / kMessageDemodulatorFreqHz;
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        MESSAGE_DEMODULATOR_PROGRAM_INIT(
            config_.message_demodulator_pio, message_demodulator_sm_[sm_index], message_demodulator_offset_,
            config_.pulses_pin, config_.demod_pins[sm_index],
#ifdef DEBUG_DEMODULATOR
            config_.recovered_clk_pins[sm_index],  // Side set on recovered clk pin on the designated state machine.
#else
            UINT16_MAX,  // Don't side set on recovered clk pin.
#endif  // DEBUG_DEMODULATOR
            message_demodulator_div);
    }

    // Set GPIO interrupts to be higher priority than the DEMOD complete interrupt to allow RSSI measurement.
    irq_set_priority(config_.preamble_detector_demod_pin_irq, kGPIOInterruptPriority);
}

void ADSBee::PIOEnable() {
    // Enable the MLAT jitter counter DMA transfers.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        dma_channel_start(mlat_jitter_dma_channel_[sm_index]);
    }

    // Enable the state machines.
    // Need to enable the demodulator SMs first, since if the preamble detector trips the IRQ but the demodulator
    // isn't enabled, we end up in a deadlock (I think, this maybe should be verified again).
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        // pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], true);
        pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], true);
    }
    // Enable round robin well formed preamble detectors.
    // NOTE: These need to be enable to allow the high power preamble detector to run, since they reset the IRQ that
    // the high power preamble detector relies on. This is a vestige of the fact that the high power preamble
    // detector uses the same PIO code that does round-robin for the well formed preamble detectors.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++) {
        pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], true);
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
        // Snapshot and reset the ISR timing accumulators. Counts are at 48MHz.
        uint32_t isr_count = isr_count_;
        uint32_t isr_sum_counts = isr_duration_sum_counts_;
        uint16_t isr_max_counts = isr_duration_max_counts_;
        isr_count_ = 0;
        isr_duration_sum_counts_ = 0;
        isr_duration_max_counts_ = 0;
        CONSOLE_WARNING("ADSBee::PruneAircraftDictionary", "OnDemodComplete: %lu calls, avg %lu us, max %u us.",
                        isr_count, isr_count > 0 ? isr_sum_counts / isr_count / 48 : 0, isr_max_counts / 48);
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
    // ISR owns the ADC (OnDemodBegin() kicked off a conversion that OnDemodComplete() reads back).
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

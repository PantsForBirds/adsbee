#pragma once

#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "settings.hh"
#include "stdint.h"

class BSP {
   public:
    static const uint16_t kMaxNumDemodStateMachines = 4;

    BSP(bool has_eeprom_in) : has_eeprom(has_eeprom_in) {
        if (!has_eeprom) {
            // Default value overrides for non-EEPROM version of ADSBee: ADSBee 1090U, ADSBee m1090.

            gnss_pps_pin = 2;
            gnss_enable_pin = 3;

            r1090_num_demod_state_machines = 4;
            for (uint16_t i = 0; i < r1090_num_demod_state_machines; i++) {
                r1090_demod_pins[i] = 20 + i;
                // Set RECOVERED_CLK to fake pin for high power preamble detector. Will be overridden by
                // higher priority (lower index) SM.
                r1090_recovered_clk_pins[i] = UINT16_MAX;  // Set to UINT16_MAX to indicate not connected.
            }
            // Set one pair of state machines to use recovered_clk for debugging.
            r1090_recovered_clk_pins[0] = 24;  // Connect SM0 to recovered_clk output for debugging.
            r1090_recovered_clk_pins[2] = 24;  // Connect SM2 to recovered_clk output for debugging.
            r1090_tl_pwm_pin = 26;
            r1090_tl_adc_pin = 27;
            r1090_tl_adc_input = 1;
            r1090_rssi_adc_pin = 28;
            r1090_rssi_adc_input = 2;

            sync_pin = 25;  // Used for sync and CC1312 bootloader backdoor.
        }

        // Board-specific value overrides based on part number stored in flash or EEPROM.
        SettingsManager::DeviceInfo device_info;
        if (settings_manager.GetDeviceInfo(device_info)) {
            switch (device_info.GetPartNumber()) {
                case SettingsManager::DeviceInfo::kPNADSBeeWinglet:
                    gnss_module_type = kGNSSModuleUbloxMAXM10;
                    [[fallthrough]];  // Winglet is a 1090 variant with a MAX-M10 populated; shares ESP32 / Sub-GHz
                                      // support below.
                case SettingsManager::DeviceInfo::kPNADSBee1090:
                case SettingsManager::DeviceInfo::kPNADSBee1090U:
                case SettingsManager::DeviceInfo::kPNADSBee1090UIndoorPoEFeeder:
                case SettingsManager::DeviceInfo::kPNGS3MPoE:
                case SettingsManager::DeviceInfo::kPNADSBeem1090:
                case SettingsManager::DeviceInfo::kPNADSBeem1090EvalBoard:
                    // Devices with recognized part numbers can support an external ESP32 and/or Sub-GHz radio.
                    has_esp32 = true;
                    has_subg = true;
                    break;
                default:
                    // All devices start out without ESP32 or Sub-GHz capability. Once they are flashed with device
                    // info, they can adopt a CC1312 and/or ESP32.
                    has_esp32 = false;
                    has_subg = false;
                    break;
            }

            r1090_rf_frontend_version = device_info.GetRFFrontendVersion();
        }
    }

    bool has_eeprom = false;

    // GNSS module populated on this board, selected from the part number in the constructor switch above.
    enum GNSSModuleType : uint8_t {
        kGNSSModuleNone = 0,     // No GNSS module populated.
        kGNSSModuleGeneric,      // Generic receiver emitting standard NMEA messages.
        kGNSSModuleUbloxMAXM10,  // u-blox MAX-M10(S).
    };
    GNSSModuleType gnss_module_type = kGNSSModuleNone;  // Default: no module.

    uint16_t gnss_uart_tx_pin = 0;
    uint16_t gnss_uart_rx_pin = 1;
    uint16_t gnss_pps_pin = 26;
    uint16_t gnss_enable_pin = UINT16_MAX;  // Set to UINT16_MAX to indicate not connected.

    uint16_t comms_uart_tx_pin = 4;
    uint16_t comms_uart_rx_pin = 5;

    spi_inst_t* copro_spi_handle = spi1;
    uint32_t copro_spi_clk_freq_hz = 20e6;  // 20 MHz (originally 40MHz, but turned down to work with flex adapter PCB).
    uint16_t copro_spi_clk_pin = 10;
    uint16_t copro_spi_mosi_pin = 11;
    uint16_t copro_spi_miso_pin = 12;
    gpio_drive_strength copro_spi_drive_strength = GPIO_DRIVE_STRENGTH_12MA;
    bool copro_spi_pullup = false;
    bool copro_spi_pulldown = true;

    bool has_esp32 = false;
    uint16_t esp32_enable_pin = 14;
    uint16_t esp32_spi_cs_pin = 9;
    uint16_t esp32_spi_handshake_pin = 13;
    uart_inst_t* esp32_uart_handle = uart0;
    uint16_t esp32_uart_tx_pin = 16;
    uint16_t esp32_uart_rx_pin = 17;

    PIO preamble_detector_pio = pio0;
    uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
    PIO message_demodulator_pio = pio1;
    uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;

    uint16_t r1090_led_pin = 15;
    uint16_t r1090_num_demod_state_machines = 4;

    // These pin values are only for old hardware.
    uint16_t r1090_pulses_pin = 19;
    uint16_t r1090_demod_pins[kMaxNumDemodStateMachines] = {20, 21, 23, 24};
    // No recovered_clk pin on old hardware (we ran out of pins when adding an additional demodulator).
    uint16_t r1090_recovered_clk_pins[kMaxNumDemodStateMachines] = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};

    uint16_t r1090_tl_pwm_pin = 25;     // Pin for Trigger Level PWM output.
    uint16_t r1090_tl_adc_pin = 27;     // Pin for reading filtered Trigger Level.
    uint16_t r1090_tl_adc_input = 1;    // ADC input for reading filtered Trigger Level.
    uint16_t r1090_rssi_adc_pin = 28;   // Pin for reading RSSI.
    uint16_t r1090_rssi_adc_input = 2;  // ADC input for reading RSSI.
    uint16_t r1090_bias_tee_enable_pin = 18;

    SettingsManager::DeviceInfo::ADSBee1090RFFrontendVersion r1090_rf_frontend_version;

    bool has_subg = false;                // Set to true if the sub-GHz receiver is present.
    uint32_t subg_spi_clk_freq_hz = 4e6;  // CC1312 has SPI clk limited to 1/12 its clock frequency. 48MHz / 12 = 4MHz.
    uint16_t sync_pin = UINT16_MAX;       // Set to UINT16_MAX to indicate not connected.
    uint16_t subg_enable_pin = 6;
    uint16_t subg_irq_pin = 7;
    uint16_t subg_cs_pin = 8;

    i2c_inst_t* onboard_i2c = i2c1;            // I2C peripheral used to talk to EEPROM (if supported).
    uint16_t onboard_i2c_sda_pin = 2;          // SDA pin for I2C.
    uint16_t onboard_i2c_scl_pin = 3;          // SCL pin for I2C.
    uint32_t onboard_i2c_clk_freq_hz = 400e3;  // 400kHz
    bool onboard_i2c_requires_init = false;    // In case I2c is shared with something else that already initializes it.

    uint16_t subg_bootloader_backdoor_pin =
        5;  // Pin for CC1312 bootloader backdoor (CC1312 DIO number, NOT RP2040 GPIO number).
};

extern BSP bsp;

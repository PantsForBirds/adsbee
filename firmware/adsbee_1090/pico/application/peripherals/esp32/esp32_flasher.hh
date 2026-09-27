#ifndef ESP32_SERIAL_FLASHER_HH_
#define ESP32_SERIAL_FLASHER_HH_

#include "adsbee.hh"
#include "bsp.hh"
#include "comms.hh"
#include "hal.hh"  // For system time stuff.
#include "hardware/uart.h"
#include "led_flasher.hh"
#include "pico/stdlib.h"
#include "unit_conversions.hh"

class ESP32SerialFlasher {
   public:
    static constexpr uint32_t kSerialFlasherResetHoldTimeMs = 100;
    static constexpr uint32_t kSerialFlasherBootHoldTimeMs = 50;

    struct ESP32SerialFlasherConfig {
        uart_inst_t *esp32_uart_handle = bsp.esp32_uart_handle;
        uint16_t esp32_uart_tx_pin = bsp.esp32_uart_tx_pin;
        uint16_t esp32_uart_rx_pin = bsp.esp32_uart_rx_pin;
        uint16_t esp32_enable_pin = bsp.esp32_enable_pin;
        uint16_t esp32_gpio0_boot_pin = bsp.esp32_spi_handshake_pin;
        uint32_t esp32_baudrate = 115200;         // Previously 115200
        uint32_t esp32_higher_baudrate = 921600;  // Previously 230400
        bool enable_md5_check = true;             // Enable MD5 checksum check after flashing.
    };

    enum ESP32SerialFlasherStatus { kESP32FlasherOkay = 1, kESP32FlasherErrorTimeout, kESP32FlasherError };

    // Number of connect + flash attempts FlashESP32() makes before giving up.
    static constexpr uint16_t kFlashAttempts = 3;

    ESP32SerialFlasher(ESP32SerialFlasherConfig config_in)
        : config_(config_in), initial_baudrate_(config_in.esp32_baudrate) {};

    bool DeInit() {
        CONSOLE_INFO("ESP32SerialFlasher::DeInit", "De-Initializing ESP32 firmware upgrade peripherals.");
        uart_deinit(config_.esp32_uart_handle);

        gpio_deinit(config_.esp32_uart_tx_pin);
        gpio_deinit(config_.esp32_uart_rx_pin);

        RestoreSharedUartPins();

        // Re-enable receiver after update if it was previously enabled.
        adsbee.SetReceiver1090Enable(receiver_was_enabled_before_update_);
        return true;
    }

    bool Init() {
        CONSOLE_INFO("ESP32SerialFlasher::Init", "Initializing ESP32 firmware upgrade peripherals.");
        // Other pins can be muxed to the same UART: GPIO 0/1 connect the GNSS module to uart0, which is also the
        // flasher's UART. The RP2040 combines the RX inputs of every pin selected for a UART, so the module's NMEA
        // output would reach the flasher's RX FIFO and break the ESP32 bootloader handshake. Park those pins first.
        ParkSharedUartPins();
        // Initialize the UART.
        gpio_set_function(config_.esp32_uart_tx_pin, GPIO_FUNC_UART);
        gpio_set_function(config_.esp32_uart_rx_pin, GPIO_FUNC_UART);
        uart_set_translate_crlf(config_.esp32_uart_handle, false);
        uart_set_fifo_enabled(config_.esp32_uart_handle, true);
        uart_init(config_.esp32_uart_handle, config_.esp32_baudrate);
        uart_tx_wait_blocking(config_.esp32_uart_handle);  // Wait for the UART tx buffer to drain.

        // Initialize the enable and boot pins.
        gpio_init(config_.esp32_enable_pin);
        gpio_set_dir(config_.esp32_enable_pin, GPIO_OUT);
        gpio_init(config_.esp32_gpio0_boot_pin);
        gpio_set_dir(config_.esp32_gpio0_boot_pin, GPIO_OUT);

        receiver_was_enabled_before_update_ = adsbee.Receiver1090IsEnabled();
        adsbee.SetReceiver1090Enable(false);  // Disable receiver to avoid interrupts during update.

        // Set up LED flasher with unique pattern to indicate ESP32 firmware is updating.
        led_flasher_.SetFlashPattern(0b101000000000, 12, 50);  // Flash pattern 10100000, 8 bits long, 100ms per bit.

        return true;
    }

    bool FlashESP32();

    /**  Helper functions exposed to the esp32_serial_flasher functions. **/

    ESP32SerialFlasherStatus SerialWrite(const uint8_t *src, size_t len, uint32_t timeout_ms) {
        uint32_t start_timestamp_ms = get_time_since_boot_ms();
        // Cheeky LED update.
        led_flasher_.Update();
        // Block until the UART is writeable.
        while (!uart_is_writable(config_.esp32_uart_handle)) {
            if (get_time_since_boot_ms() - start_timestamp_ms > timeout_ms) return kESP32FlasherErrorTimeout;
        }
        // NOTE: The timeout implementation is janky and doesn't exit the uart blocking write early.
        uart_write_blocking(config_.esp32_uart_handle, src, len);
        if (get_time_since_boot_ms() - start_timestamp_ms > timeout_ms) {
            return kESP32FlasherErrorTimeout;
        }
        return kESP32FlasherOkay;
    }

    inline bool uart_rx_not_timed_out_yet(uart_inst_t *uart_handle, uint32_t start_timestamp_ms, uint32_t timeout_ms) {
        int32_t time_remaining_ms = start_timestamp_ms + timeout_ms - get_time_since_boot_ms();
        if (time_remaining_ms < 0) return false;
        return uart_is_readable_within_us(config_.esp32_uart_handle, time_remaining_ms * kUsPerMs);
    }

    ESP32SerialFlasherStatus SerialRead(uint8_t *dest, size_t len, uint32_t timeout_ms) {
        uint64_t start_timestamp_ms = get_time_since_boot_ms();
        uint8_t *dest_start = dest;
        while (dest < dest_start + len &&
               uart_rx_not_timed_out_yet(config_.esp32_uart_handle, start_timestamp_ms, timeout_ms)) {
            uart_read_blocking(config_.esp32_uart_handle, dest++, 1);
            if (dest - dest_start == len) {
                return kESP32FlasherOkay;  // Finished reading all the characters.
            }
        }
        return kESP32FlasherErrorTimeout;
    }

    void ResetTarget() {
        gpio_put(config_.esp32_enable_pin, 0);
        busy_wait_ms(kSerialFlasherResetHoldTimeMs);
        gpio_put(config_.esp32_enable_pin, 1);
    }

    void EnterBootloader() {
        gpio_put(config_.esp32_gpio0_boot_pin, 0);
        ResetTarget();
        busy_wait_ms(kSerialFlasherBootHoldTimeMs);
        gpio_put(config_.esp32_gpio0_boot_pin, 1);
        busy_wait_ms(100);
        // Flush the rx uart.
        while (uart_is_readable_within_us(config_.esp32_uart_handle, 100)) {
            uart_getc(config_.esp32_uart_handle);
        }
    }

    void SetBaudRate(uint32_t baudrate) {
        config_.esp32_baudrate = baudrate;
        uart_set_baudrate(config_.esp32_uart_handle, config_.esp32_baudrate);
    }

   private:
    // UART instance (0 or 1) a GPIO connects to when its function is GPIO_FUNC_UART (RP2040 datasheet, GPIO function
    // table: GPIO 0-3 -> UART0, 4-11 -> UART1, 12-19 -> UART0, 20-27 -> UART1, 28-29 -> UART0).
    static uint GpioUartIndex(uint gpio) { return ((gpio + 4) >> 3) & 1; }

    /**
     * Switches every pin other than the flasher's own TX/RX that is muxed to the flasher's UART over to a plain SIO
     * input with a pull-up (idle level for a UART line), remembering its previous pulls so RestoreSharedUartPins()
     * can undo it.
     */
    void ParkSharedUartPins() {
        parked_pins_mask_ = 0;
        uint uart_index = uart_get_index(config_.esp32_uart_handle);
        for (uint gpio = 0; gpio < NUM_BANK0_GPIOS; gpio++) {
            if (gpio == config_.esp32_uart_tx_pin || gpio == config_.esp32_uart_rx_pin) continue;
            if (gpio_get_function(gpio) != GPIO_FUNC_UART || GpioUartIndex(gpio) != uart_index) continue;
            parked_pins_mask_ |= 1u << gpio;
            parked_pin_pulled_up_[gpio] = gpio_is_pulled_up(gpio);
            parked_pin_pulled_down_[gpio] = gpio_is_pulled_down(gpio);
            gpio_set_dir(gpio, GPIO_IN);
            gpio_set_function(gpio, GPIO_FUNC_SIO);
            gpio_set_pulls(gpio, true, false);
            CONSOLE_INFO("ESP32SerialFlasher::ParkSharedUartPins",
                         "Disconnected GPIO %u from uart%u for the ESP32 flash.", gpio, uart_index);
        }
    }

    /**
     * Returns the pins parked by ParkSharedUartPins() to GPIO_FUNC_UART with their original pulls. Their owner (e.g.
     * the GNSS receiver) still has to re-initialize the UART itself, since DeInit() deinitializes it.
     */
    void RestoreSharedUartPins() {
        for (uint gpio = 0; gpio < NUM_BANK0_GPIOS; gpio++) {
            if (!(parked_pins_mask_ & (1u << gpio))) continue;
            gpio_set_pulls(gpio, parked_pin_pulled_up_[gpio], parked_pin_pulled_down_[gpio]);
            gpio_set_function(gpio, GPIO_FUNC_UART);
        }
        parked_pins_mask_ = 0;
    }

    ESP32SerialFlasherConfig config_;
    uint32_t initial_baudrate_;  // Bootloader sync baud rate; config_.esp32_baudrate changes during a flash.
    uint32_t parked_pins_mask_ = 0;
    bool parked_pin_pulled_up_[NUM_BANK0_GPIOS] = {};
    bool parked_pin_pulled_down_[NUM_BANK0_GPIOS] = {};
    bool receiver_was_enabled_before_update_;
    LEDFlasher led_flasher_{LEDFlasher::LEDFlasherConfig{}};
};

extern ESP32SerialFlasher esp32_flasher;

#endif /*  */
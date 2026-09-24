#include "target_uart.hh"

#include "board.hh"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "status.hh"
#include "tusb.h"

static uart_inst_t* const kUart = uart0;

// Lock-free single-producer (IRQ) / single-consumer (main loop) rings; sizes are powers of two.
static const size_t kRxRingSize = 8192;
static const size_t kTxRingSize = 1024;

static uint8_t rx_ring[kRxRingSize];
static volatile uint32_t rx_head = 0;  // Written by IRQ.
static volatile uint32_t rx_tail = 0;  // Written by consumer.
static volatile uint32_t rx_drops = 0;

static uint8_t tx_ring[kTxRingSize];
static uint32_t tx_head = 0;
static uint32_t tx_tail = 0;

static void OnUartRx() {
    while (uart_is_readable(kUart)) {
        uint8_t byte = uart_getc(kUart);
        uint32_t head = rx_head;
        if (head - rx_tail >= kRxRingSize) {
            rx_drops = rx_drops + 1;  // Ring full: drop newest.
        } else {
            rx_ring[head % kRxRingSize] = byte;
            rx_head = head + 1;
        }
    }
}

void TargetUartInit(uint32_t baud) {
    uart_init(kUart, baud);
    gpio_set_function(kPinUartTx, GPIO_FUNC_UART);
    gpio_set_function(kPinUartRx, GPIO_FUNC_UART);
    // Replace the pad-default pull-down with a pull-up so RX idles high (UART idle level) when
    // the module is absent, matching the device-side console RX pull-up convention.
    gpio_pull_up(kPinUartRx);
    uart_set_format(kUart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(kUart, true);

    irq_set_exclusive_handler(UART0_IRQ, OnUartRx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(kUart, true /* rx */, false /* tx */);
}

void TargetUartSetBaud(uint32_t baud) {
    uart_set_baudrate(kUart, baud);
    TargetUartFlushInput();
}

size_t TargetUartRead(uint8_t* buf, size_t max_len) {
    size_t count = 0;
    while (count < max_len && rx_tail != rx_head) {
        buf[count++] = rx_ring[rx_tail % kRxRingSize];
        rx_tail = rx_tail + 1;
    }
    return count;
}

int TargetUartReadByteTimeout(uint32_t timeout_ms) {
    absolute_time_t deadline = delayed_by_ms(get_absolute_time(), timeout_ms);
    while (true) {
        if (rx_tail != rx_head) {
            uint8_t byte = rx_ring[rx_tail % kRxRingSize];
            rx_tail = rx_tail + 1;
            return byte;
        }
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) return -1;
        tud_task();       // Keep USB enumerated/responsive through long bootloader operations.
        StatusUpdate();   // Keep the LED blinking.
    }
}

void TargetUartWriteBlocking(const uint8_t* data, size_t len) {
    while (tx_tail != tx_head) TargetUartPumpTx();  // Preserve ordering vs. queued bridge bytes.
    uart_write_blocking(kUart, data, len);
}

size_t TargetUartWriteNonblocking(const uint8_t* data, size_t len) {
    size_t queued = 0;
    while (queued < len && tx_head - tx_tail < kTxRingSize) {
        tx_ring[tx_head % kTxRingSize] = data[queued++];
        tx_head++;
    }
    TargetUartPumpTx();
    return queued;
}

void TargetUartPumpTx() {
    while (tx_tail != tx_head && uart_is_writable(kUart)) {
        uart_putc_raw(kUart, tx_ring[tx_tail % kTxRingSize]);
        tx_tail++;
    }
}

size_t TargetUartTxFree() { return kTxRingSize - (tx_head - tx_tail); }

void TargetUartFlushInput() {
    while (uart_is_readable(kUart)) (void)uart_getc(kUart);
    rx_tail = rx_head;
}

uint32_t TargetUartRxDropCount() { return rx_drops; }

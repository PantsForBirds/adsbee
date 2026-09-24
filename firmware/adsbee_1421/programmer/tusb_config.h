// TinyUSB configuration: single full-speed CDC device, driven directly (no stdio_usb) so the
// bridge owns the line-coding and line-state callbacks. CFG_TUSB_MCU / CFG_TUSB_OS come from
// the pico-sdk tinyusb integration.

#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#define CFG_TUD_ENABLED 1
// Required for tusb_init() to actually start the device stack: TUD_OPT_RHPORT (and thus the
// tud_init() call inside tusb_init()) is only defined when the root-hub port mode is set.
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// FIFO sizes on the device side; the bridge drains these into the UART rings.
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 512
#define CFG_TUD_CDC_EP_BUFSIZE 64

#endif  // TUSB_CONFIG_H_

// USB descriptors: single CDC-ACM function. VID/PID are the Raspberry Pi CDC defaults so stock
// host drivers bind; the product string identifies the jig.

#include "pico/unique_id.h"
#include "tusb.h"

#define USB_VID 0x2E8A  // Raspberry Pi
#define USB_PID 0x000A  // CDC

enum {
    kStrIdxLangId = 0,
    kStrIdxManufacturer,
    kStrIdxProduct,
    kStrIdxSerial,
    kStrIdxCdcInterface,
};

static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    // Use the Interface Association Descriptor class triplet so the CDC pair enumerates cleanly.
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = kStrIdxManufacturer,
    .iProduct = kStrIdxProduct,
    .iSerialNumber = kStrIdxSerial,
    .bNumConfigurations = 1,
};

const uint8_t* tud_descriptor_device_cb(void) { return (const uint8_t*)&device_descriptor; }

enum {
    kItfNumCdc = 0,
    kItfNumCdcData,
    kItfNumTotal,
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, kItfNumTotal, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(kItfNumCdc, kStrIdxCdcInterface, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
                       EPNUM_CDC_IN, 64),
};

const uint8_t* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return configuration_descriptor;
}

const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    static uint16_t desc_str[34];

    char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
    const char* str;
    switch (index) {
        case kStrIdxLangId: {
            desc_str[1] = 0x0409;  // English (US).
            desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | 4);
            return desc_str;
        }
        case kStrIdxManufacturer:
            str = "ADSBee";
            break;
        case kStrIdxProduct:
            str = "ADSBee 1421 Programmer";
            break;
        case kStrIdxSerial:
            pico_get_unique_board_id_string(serial, sizeof(serial));
            str = serial;
            break;
        case kStrIdxCdcInterface:
            str = "ADSBee 1421 Console";
            break;
        default:
            return NULL;
    }

    size_t len = 0;
    while (str[len] != '\0' && len < 32) {
        desc_str[1 + len] = (uint16_t)str[len];
        len++;
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return desc_str;
}

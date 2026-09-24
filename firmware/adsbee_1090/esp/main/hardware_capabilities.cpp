#include "hardware_capabilities.hh"

#include "comms.hh"  // Logging.
#include "object_dictionary.hh"
#include "sdkconfig.h"

#ifdef CONFIG_SPIRAM
#include "esp_psram.h"
#endif

bool HardwareCapabilities::has_psram_ = false;
uint32_t HardwareCapabilities::psram_total_bytes_ = 0;

void HardwareCapabilities::Detect() {
#ifdef CONFIG_SPIRAM
    // With CONFIG_SPIRAM_IGNORE_NOTFOUND the startup code has already probed the PSRAM chip; on a module without PSRAM
    // esp_psram_is_initialized() is false and no SPIRAM heap region exists.
    if (esp_psram_is_initialized()) {
        psram_total_bytes_ = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    }
#endif
    has_psram_ = psram_total_bytes_ >= kMinPSRAMHeapBytes;

    if (has_psram_) {
        CONSOLE_INFO("HardwareCapabilities::Detect", "PSRAM detected: %lu KB heap (%lu KB free). Internal RAM: %lu KB free.",
                     (unsigned long)(psram_total_bytes_ / 1024), (unsigned long)(GetPSRAMFreeBytes() / 1024),
                     (unsigned long)(GetInternalFreeBytes() / 1024));
    } else if (psram_total_bytes_ > 0) {
        CONSOLE_WARNING("HardwareCapabilities::Detect",
                        "PSRAM detected but only %lu KB (< %lu KB), treating as no PSRAM for feature gating.",
                        (unsigned long)(psram_total_bytes_ / 1024), (unsigned long)(kMinPSRAMHeapBytes / 1024));
    } else {
        CONSOLE_INFO("HardwareCapabilities::Detect", "No PSRAM. Internal RAM: %lu KB free.",
                     (unsigned long)(GetInternalFreeBytes() / 1024));
    }
}

uint8_t HardwareCapabilities::GetCapabilitiesBitfield() {
    uint8_t caps = ObjectDictionary::kESP32HWCapReported;
    if (has_psram_) caps |= ObjectDictionary::kESP32HWCapPSRAM;
    return caps;
}

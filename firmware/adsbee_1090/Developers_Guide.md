# ADSBee Developer Guide

## Critical Information
Correct as of Version 0.8.2

### Quick Architecture Overview

ADSBee uses a multi-processor architecture:
- **RP2040 (Pico)**: Main processor, handles RF decoding, manages other processors
- **ESP32-S3**: WiFi/network processor, handles TCP/IP, web server, feeds
- **CC1312**: Sub-GHz radio for UAT (978 MHz)

Communication between processors:
- RP2040 ↔ ESP32: SPI interface with custom protocol
- RP2040 ↔ CC1312: UART interface

### Build System and combined.uf2

The firmware build process creates a `combined.uf2` file that contains:
1. RP2040 bootloader
2. RP2040 application firmware
3. ESP32 firmware binary (embedded)
4. CC1312 firmware binary (embedded)

**IMPORTANT**: The ESP32 and CC1312 binaries are embedded INSIDE the RP2040 firmware.

### Flashing Process

`./build.sh build_and_flash` handles the whole sequence for an attached ADSBee 1090/1090U: it builds
all three targets, finds the device on USB, reboots it into the RP2040 bootloader with
`AT+BOOT_USB_UF2` (no BOOTSEL press), copies `combined.uf2`, watches the console, and then confirms
the RP2040 and ESP32 report the version you just built. Pass a CDC node —
`./build.sh build_and_flash /dev/cu.usbmodem21201` — to pick between several attached devices. If a
serial monitor (VS Code, a Web Serial page, `screen`) still holds the port, the script skips it and
says so; close it and re-run.

`./build.sh flash` does the same device handling but skips every build step, flashing the
`combined.uf2` that is already on disk. Use it to re-flash after a failed copy, or to push one build
onto several boards, without paying for three container builds. It warns first if that uf2 is older
than the source tree — the post-flash version check compares against the version in
`object_dictionary.cpp` **source**, so flashing a stale image fails that check.

When you flash `combined.uf2` to the RP2040:
1. RP2040 firmware is updated immediately
2. On boot, RP2040 checks firmware versions of ESP32 and CC1312
3. **If versions differ**, RP2040 flashes the embedded binaries to the coprocessors
4. **If versions match**, no reflashing occurs (You will scratch your head for a very long time)

### Version Management

Firmware version is defined in: `firmware/common/coprocessor/object_dictionary.cpp`
```cpp
const uint8_t ObjectDictionary::kFirmwareVersionMajor = 0;
const uint8_t ObjectDictionary::kFirmwareVersionMinor = 8;
const uint8_t ObjectDictionary::kFirmwareVersionPatch = 3;
const uint8_t ObjectDictionary::kFirmwareVersionReleaseCandidate = 1;
```

**CRITICAL RULES:**

1. **When changing shared structures (like Settings)**:
   - MUST increment `kSettingsVersion` in `firmware/common/settings/settings.hh`
   - MUST increment firmware version (patch or RC) to force ESP32 reflash
   - Both changes must be committed together

2. **When to increment firmware version**:
   - Major: Breaking changes, new features
   - Minor: Significant improvements, protocol changes
   - Patch: Bug fixes, minor improvements
   - RC: Development builds (0 = release, 1+ = RC number)

3. **Settings version (`kSettingsVersion`)**:
   - Located in: `firmware/common/settings/settings.hh`
   - Increment when ANY field is added/removed/changed in Settings struct
   - Causes settings reset on both processors
   - Prevents settings corruption from mismatched structures

### Common Development Pitfalls

#### Problem 1: Settings Size Mismatch
**Symptom**:
```
SPICoprocessor::ExecuteSCCommandRequest: Settings write requested with len 1048, sending 2404 instead.
```

**Cause**: One processor has old firmware, other has new firmware with different Settings structure.

**Solution**:
1. Increment firmware version (forces ESP32 reflash)
2. Rebuild
3. Flash new combined.uf2

#### Problem 2: ESP32 Not Updating
**Symptom**: ESP32 keeps old behavior despite flashing new combined.uf2

**Cause**: Firmware version unchanged, so RP2040 doesn't reflash ESP32

**Solution**: Always increment firmware version when ESP32 code changes

#### Problem 3: Build Order Matters
**Correct build sequence**:
```bash
//TODO Come back and fix build process.
# 1. Build ESP32 FIRST (creates adsbee_esp.bin)


# 2. Build CC1312 (creates sub_ghz_radio.bin)


# 3. Build RP2040 LAST (embeds the other binaries)

```

**GitHub Actions handles this automatically**, but local builds must follow this order.

### GitHub Actions Workflow

The CI/CD pipeline:
1. Builds ESP32 firmware → uploads as artifact
2. Builds CC1312 firmware → uploads as artifact
3. Downloads both artifacts
4. Builds RP2040 firmware with embedded binaries
5. Produces `combined.uf2` in the firmware artifact

**To get the latest build**:
1. Push changes to your branch
2. Go to GitHub Actions tab
3. Find your workflow run
4. Download "firmware" artifact
5. Extract and flash `combined.uf2`

### SPI Communication Protocol

The RP2040 and ESP32 communicate over SPI with a custom protocol:
- Settings synchronization happens on boot
- Settings must have matching size and version
- Version mismatch triggers settings reset
- Size mismatch causes communication errors

### Adding New Features to Settings

When adding new settings fields:

1. **Update Settings struct** in `firmware/common/settings/settings.hh`:
```cpp
// Add your fields
bool my_new_feature_enabled = false;
uint32_t my_new_parameter = 42;
```

2. **Increment kSettingsVersion**:
```cpp
static constexpr uint32_t kSettingsVersion = 12;  // Was 11
```

3. **Increment firmware version** in `firmware/common/coprocessor/object_dictionary.cpp`:
```cpp
const uint8_t ObjectDictionary::kFirmwareVersionPatch = 4;  // Was 3
// OR for development:
const uint8_t ObjectDictionary::kFirmwareVersionReleaseCandidate = 2;  // Was 1
```

4. **Commit all changes together**:
```bash
git add firmware/common/settings/settings.hh
git add firmware/common/coprocessor/object_dictionary.cpp
git commit -m "Add new feature settings, bump versions for ESP32 reflash"
```

### Debugging Tips

1. **Check processor communication**:
   - Look for SPI timeout errors in console
   - Settings size mismatch indicates version skew

2. **Verify firmware versions**:
   - Web interface shows version at bottom
   - Console shows versions on boot

3. **Force ESP32 reflash**:
   - Increment firmware version
   - Or hold BOOT button on ESP32 during RP2040 reset

4. **Settings reset**:
   - Increment kSettingsVersion forces reset
   - Or use AT command: `AT+RSTS`

### Testing Checklist

- [ ] Settings struct changes? → Increment kSettingsVersion
- [ ] ESP32 code changed? → Increment firmware version
- [ ] CC1312 code changed? → Increment firmware version
- [ ] Build passes on all three processors?
- [ ] GitHub Actions workflow succeeds?
- [ ] combined.uf2 flashes successfully?
- [ ] No SPI communication errors after flash?
- [ ] Web interface loads correctly?
- [ ] Settings persist across reboot?

### Quick Reference

| File | Purpose | When to Change |
|------|---------|----------------|
| `firmware/common/settings/settings.hh` | Settings structure and version | Adding/changing settings |
| `firmware/common/coprocessor/object_dictionary.cpp` | Firmware version | Any code changes |
| `firmware/pico/application/main.cc` | Main RP2040 logic | Core functionality |

### Emergency Recovery

If firmware is corrupted:
1. Hold BOOTSEL button while connecting USB
2. RP2040 appears as USB drive
3. Copy known-good combined.uf2
4. Device auto-reboots after copy
5. ESP32 will be reflashed if version differs

# Check a Unit's Configuration

`AT+CONFIG_HASH?` prints hashes of a unit's settings, CCFG and firmware. Two units set up the same way print the same values, so one line tells you whether a unit matches your reference unit. It works on the ADSBee 1090 and the ADSBee m1421.

## Read the hashes

Send `AT+CONFIG_HASH?` on the [CLI](../getting-started/quick-start.md#connect-to-cli). An ADSBee 1090 answers:

```
AT+CONFIG_HASH?
CONFIG_HASH=0x01B412E7
SETTINGS_FLEET=0x41982E33 (settings version 19, 1792 Bytes)
SETTINGS_FULL=0x34C6C16A
CCFG_CC1312=0x4BB06FBD
FIRMWARE=0x2AE30B6C
FIRMWARE_RP2040=0x17AD7E0C (UF2 image, partition 0, 3099392 Bytes)
FIRMWARE_ESP32=0x62127DBE
FIRMWARE_CC1312=0x4F0E5507
```

An ADSBee m1421 answers:

```
AT+CONFIG_HASH?
CONFIG_HASH=0xDC99B0AC
SETTINGS_FLEET=0x77C7FD9F (settings version 6, 78 Bytes)
SETTINGS_FULL=0x82DBA15A
CCFG_CC1314=0x6886ABE1
FIRMWARE=0xB8A29C75
FIRMWARE_CC1314=0x80AB341C (577348 Bytes)
```

Every hash is 32 bits, printed as 8 hexadecimal digits. All are CRC-32 values except `FIRMWARE_ESP32`. A line reads `UNKNOWN` when a processor is off or didn't answer; it then counts as `0x00000000` in the lines above it.

| Line | What it covers |
| --- | --- |
| `CONFIG_HASH` | `SETTINGS_FLEET`, the CCFG and `FIRMWARE` together. Compare this one first. |
| `SETTINGS_FLEET` | The settings that units with the same job share. |
| `SETTINGS_FULL` | Every setting. |
| `CCFG_CC1312`, `CCFG_CC1314` | The CCFG (customer configuration) of the TI chip, which holds its bootloader and debug access settings. Read from the chip's flash. |
| `FIRMWARE` | The `FIRMWARE_<processor>` lines together. |
| `FIRMWARE_RP2040` | The application image in the flash partition the RP2040 runs from. |
| `FIRMWARE_ESP32` | The application the ESP32 runs: the first 4 bytes of the SHA-256 the ESP32 build stores in its image. |
| `FIRMWARE_CC1312`, `FIRMWARE_CC1314` | The application image in the TI chip's flash. |

Settings are hashed as the unit is running now, saved or not. Run `AT+SETTINGS=SAVE` first if you want the check to hold after a reboot.

## What the settings hashes leave out

`SETTINGS_FLEET` leaves out the settings that normally differ between units:

- ADSBee 1090: hostname, WiFi access point SSID and channel, feed receiver IDs, Remote ID UAS ID, and the receiver position values. The receiver position source (`AT+RX_POSITION`) still counts.
- ADSBee m1421: the receiver position values.

`SETTINGS_FULL` covers those too. The receiver position values count only for a `FIXED` position, and the ICAO address only for the `ICAO` source, because the other sources update the position while the unit runs.

Neither hash covers the device information (`AT+DEVICE_INFO?`: part code, OTA keys), or the checksums stored with the settings. Both include the settings version, which the first line also prints.

## What the firmware hashes leave out

- Settings and device information storage.
- ADSBee 1090: the RP2040 bootloader, the flash partition that is not running, and the ESP32 bootloader and partition table.
- ADSBee m1421: the ADSBee 1421 Programmer.

An ADSBee 1090 holds one of three RP2040 images for the same firmware version, each with its own hash. `FIRMWARE_RP2040` names it: the image from the .uf2 file (USB update), or the .ota image for flash partition 0 or 1 (network update). Units updated the same way print the same value.

## Get the expected values

Each [firmware release](https://github.com/PantsForBirds/adsbee/releases) from this version on comes with a `.hashes.txt` file. It lists the firmware and CCFG lines a unit running that release prints:

```
CCFG_CC1312=0x4BB06FBD
FIRMWARE_ESP32=0x62127DBE
FIRMWARE_CC1312=0x4F0E5507

FIRMWARE=0x2AE30B6C
FIRMWARE_RP2040=0x17AD7E0C (UF2 image, partition 0, 3099392 Bytes)

FIRMWARE=0xD4DC6999
FIRMWARE_RP2040=0xB1240193 (OTA image, partition 0, 3100472 Bytes)

FIRMWARE=0xF4009D73
FIRMWARE_RP2040=0xB2571605 (OTA image, partition 1, 3100472 Bytes)
```

`firmware/scripts/firmware_hashes.py` prints the same lines for images you built yourself:

```bash
python3 firmware/scripts/firmware_hashes.py adsbee_1421 --hex adsbee_1421-<version>.hex
python3 firmware/scripts/firmware_hashes.py adsbee_1090 --uf2 adsbee_1090-<version>.uf2 --ota adsbee_1090-<version>.ota \
    --esp32 adsbee_esp.bin --cc1312 sub_ghz_radio.bin
```

The ADSBee 1090 build runs it and writes `adsbee_1090-<version>.hashes.txt` next to the .uf2 file.

The values belong to one build. An ADSBee 1090 image built again from the same source has other `FIRMWARE_ESP32` and `FIRMWARE_RP2040` values, because the ESP32 image carries its build time.

The settings hashes and `CONFIG_HASH` depend on your settings, so take their expected values from a unit you have set up and checked.

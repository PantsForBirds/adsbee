# ADSBee 1421 Programmer + Pass-Through Jig

A Waveshare RP2040-Zero application that keeps an attached ADSBee m1421 (TI CC1314R10) flashed
with the firmware image baked into the jig, then acts as a transparent USB serial adapter to the
module's console (normally at the factory-default 1,000,000 baud).

At power-up the jig:

1. Enters the CC1314's factory ROM UART bootloader (SYNC backdoor held high through a reset
   pulse — same mechanism as `software/adsbee_1421_flasher/`).
2. Compares the on-chip flash against the baked-in image using the bootloader's `CRC32` command
   (per-segment CRCs are computed at build time). On blank or mismatched devices, only the 2 KB
   flash sectors the image actually covers are erased (`SECTOR_ERASE`, never `BANK_ERASE`), then
   programmed and CRC-verified. The Settings (`0x000FC000`) and Device Info / OTA key
   (`0x000FE000`) sectors at the top of the bank are never touched, so both survive a reflash
   (Settings still reset themselves if the firmware's settings version changed). Matching devices
   are left untouched. `hex_to_c.py` refuses to bake, and the jig refuses to flash, an image that
   reaches into those reserved sectors.
3. Resets the module into the application, finds the console by sweeping the firmware's baud
   whitelist ({1000000, 921600, 460800, 230400, 115200}, factory-default-first — the app boots
   at its saved console baud, persisted via `AT+SETTINGS=SAVE`), drives its live rate to 1 M if
   it answered elsewhere, and becomes a USB-CDC ↔ UART pass-through.

The ROM bootloader leg always runs at 1 M (the ROM auto-bauds to it; ceiling ~1.2 M). The jig
only moves the console's *live* rate — it never issues `AT+SETTINGS=SAVE`, so the module's
persisted baud setting is untouched. If no module responds, the jig retries forever (attach a
module any time) and reports a diagnosis on the CDC port every few seconds.

## Wiring

| RP2040-Zero | ADSBee m1421                | Notes                              |
|-------------|-----------------------------|------------------------------------|
| GP28        | SURX (pin 20, DIO_2 UART RX)| UART0 TX                           |
| GP29        | SUTX (pin 21, DIO_3 UART TX)| UART0 RX                           |
| GP27        | SYNC (pin 28, DIO_5)        | Bootloader backdoor, active high   |
| GP26        | ~SRST (pin 17)              | Reset, active low, driven open-drain|
| GND         | GND                         |                                    |

The module's firmware must have the CCFG bootloader backdoor enabled (all builds since it was
encoded in `firmware/adsbee_1421/ti/syscfg/adsbee_1421.syscfg`). Older boards need one
JTAG flash or the `AT+BOOT_UART_BOOTLOADER=1DEADBEE` self-erase path first.

## Build

The baked firmware image comes from the adsbee_1421 Release build, so build that first:

```sh
cd firmware/adsbee_1421
./build.sh ti
./build.sh programmer
```

Artifact: `firmware/adsbee_1421/programmer/build/Release/adsbee_1421_programmer.uf2` —
hold BOOT on the RP2040-Zero while plugging it in and drag the file onto the `RPI-RP2` drive.
Or run `./build.sh build_and_flash`, which builds both apps, prompts for bootloader mode, copies
the uf2, and watches the jig's console while it reflashes the attached m1421. `./build.sh flash`
does the same without building, using the uf2 already on disk; because the ti hex is baked in at
configure time, it warns if that hex is newer than the jig image.
A version-stamped copy (`adsbee_1421_programmer-fw<version>.uf2`, named after the **baked**
firmware version parsed from `object_dictionary.cpp`) is produced alongside it for release/CI.
To bake a different image, pass `-DADSBEE_1421_HEX=<path>` to CMake.

## Buttons

- **BOOTSEL held at power-up**: force a reflash even if the CRC matches.
- **BOOTSEL tapped during pass-through**: rerun the check/flash cycle and re-negotiate the
  console (sweep + move to 1 M). This is also the recovery path after in-band desyncs (see
  limitations).
- **BOOTSEL held for 3 s (at any time)**: arm a **settings erase**. The LED blinks white, and at
  the next bootloader entry the jig erases the four Settings sectors
  (`0x000FC000`–`0x000FDFFF`); the device then boots with factory defaults. Device Info
  (`0x000FE000`, part code and OTA keys) is never touched.

  This is the escape hatch for a device whose saved settings stop the console coming up. Because
  it is armed from the wait loops — not just from pass-through — it works while the jig is stuck
  reporting `Device console not responding at any whitelisted baud rate`, which is exactly when
  `AT+SETTINGS=RESET` and `AT+BOOT_UART_BOOTLOADER` are unavailable (both need a console that
  already answers). It is never triggered automatically: it discards the user's settings.

## LED legend (WS2812)

| Color         | Meaning                                  |
|---------------|------------------------------------------|
| yellow blink  | waiting for a device / bootloader entry  |
| orange        | forced reflash armed (BOOTSEL at boot)   |
| white blink   | settings erase armed (BOOTSEL held 3 s)  |
| cyan          | CRC check against the baked image        |
| magenta blink | erasing + programming                    |
| blue blink    | post-program CRC verify                  |
| blue          | console baud negotiation                 |
| green         | pass-through (normally 1 Mbaud)          |
| red           | error (automatic retry follows)          |

## Pass-through behavior

The bridge emulates a TTL USB-UART adapter wired the way the existing host tools expect
(`adapter RTS → SYNC`, `DTR → RESET_N`, where asserting a modem-control line drives the
physical pin low):

- **RTS asserted → SYNC low** (device awake); **RTS deasserted → SYNC high** (device asleep /
  backdoor armed). Most terminals assert RTS on open.
- **DTR assert edge → 50 ms reset pulse.** Edge-triggered rather than level-held so terminals
  that keep DTR asserted don't hold the device in reset.
- **Host baud changes are applied to the UART directly**, so tools that manage their own baud
  (the web console, the python flasher's bootloader phase) work through the jig unmodified.
- After a host-driven reset with SYNC low the device console reboots at its *saved* baud
  (factory default 1 M). If the host's line coding matches the rate the console was last
  negotiated to, the jig stays transparent; otherwise it automatically re-negotiates (sweep +
  `AT+BAUD_RATE`) to the host's rate. A module saved at some other rate is also recoverable by
  the host probing the whitelist itself (line-coding changes retune the jig UART live) or by
  the BOOTSEL recheck.

Both `software/adsbee_1421_flasher/` (default flags) and the web console's built-in flasher can
flash a module *through* the jig; during those sessions SYNC is high, so the jig stays fully
transparent and never injects traffic.

## Troubleshooting (yellow blink / no green)

Open the jig's CDC port (any terminal, any baud — e.g. `python3 -m serial.tools.miniterm`) to
see per-attempt diagnostics. The jig re-prints its last diagnosis every ~5 s while waiting.

| Message | Meaning |
|---------|---------|
| `SBL sync at <baud>` | Bootloader entry works; that rate is reused for the session. |
| `... failed (timed out ...); no late bytes` | Silence from the module: reset, UART, or SYNC not reaching it. |
| `... failed (unexpected byte ...)` / `late bytes: ...` | The module answered with garbage: baud/framing issue on the link. |
| `App console responds at <baud> ... SBL entry fails` | Reset + UART wiring are good. Check SYNC (GP27 → module pin 28) and that the module firmware has the CCFG backdoor enabled. |
| `No response ... RESET_N(GP26)=LOW (stuck in reset ...)` | Something is holding reset low with the jig's driver released — wiring short or drive conflict on ~SRST. |
| `No response ... UART RX(GP29)=LOW (module TX not driving ...)` | Module unpowered, held in reset, or SUTX wiring wrong (RX should idle high when the module runs). |
| `No response ... RESET_N=high, UART RX=high (link plausible)` | Lines look electrically sane; suspect TX leg (GP28 → SURX) or module-side UART config. |
| `Device console not responding at any whitelisted baud rate` | SBL/flash worked but the app's AT console never answered `AT+DEVICE_INFO?` at any of the five whitelisted rates within ~6 s of boot. If it repeats on every boot with a CRC-verified image, the saved settings are the likely cause — hold BOOTSEL for 3 s to erase them (see [Buttons](#buttons)). |
| `Settings erase ARMED` / `Settings erased; ...` | A BOOTSEL long press was registered, and the Settings sectors were erased at the next bootloader entry. The device now boots with factory defaults. |

Modules running pre-backdoor firmware can't be entered via SYNC at all: flash them once via
JTAG, or connect a console directly and send `AT+BOOT_UART_BOOTLOADER=1DEADBEE` (erases the
app and drops into the ROM bootloader; the jig will then flash it on the next check).

## Limitations

- Sending `AT+BAUD_RATE=CONSOLE,...` through the bridge desyncs the link (the jig doesn't
  parse bridged traffic), and `AT+REBOOT` desyncs it when the device's saved baud differs from
  the current link rate. Recovery: press BOOTSEL once, change the host line coding to what the
  device is actually running, or power-cycle the jig.
- Status text (flash progress, negotiation results) appears on the same CDC port before
  pass-through starts; anything typed during those phases is ignored.

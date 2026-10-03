# ADSBee 1421 Programmer

A Waveshare RP2040-Zero application that keeps an attached ADSBee m1421 (TI CC1314R10) flashed
with the firmware image baked into the Programmer, then acts as a USB serial adapter to the module's
console. Open the port at any baud rate; the Programmer follows the console's rate (see
[Baud rate](#baud-rate)).

At power-up the Programmer:

1. Enters the CC1314's factory ROM UART bootloader (SYNC backdoor held high through a reset
   pulse; see [Reflashing over UART: the SYNC bootloader backdoor](../README.md#reflashing-over-uart-the-sync-bootloader-backdoor)).
2. Compares the on-chip flash against the baked-in image using the bootloader's `CRC32` command
   (per-segment CRCs are computed at build time). On blank or mismatched devices, only the 2 KB
   flash sectors the image actually covers are erased (`SECTOR_ERASE`, never `BANK_ERASE`), then
   programmed and CRC-verified. The Settings (`0x000FC000`) and Device Info / OTA key
   (`0x000FE000`) sectors at the top of the bank are never touched, so both survive a reflash
   (Settings still reset themselves if the firmware's settings version changed). Matching devices
   are left untouched. Images that reach into those sectors are refused.
3. Resets the module into the application, finds the console (see
   [Finding the console](#finding-the-console)), and becomes a USB ↔ UART pass-through.

- If the bootloader can't be entered but the console answers (e.g. the backdoor is off), the
  Programmer warns and enters pass-through without the check. Forced reflash and settings erase
  wait until the bootloader works again.
- The Programmer never changes the console's rate or sends `AT+SETTINGS=SAVE`.
- If no module responds, it retries forever (attach a module any time) and prints a diagnosis on
  its USB serial port every few seconds.

## Wiring

| RP2040-Zero | ADSBee m1421                | Notes                              |
|-------------|-----------------------------|------------------------------------|
| GP28        | SURX (pin 20, DIO_2 UART RX)| UART0 TX                           |
| GP29        | SUTX (pin 21, DIO_3 UART TX)| UART0 RX                           |
| GP27        | SYNC (pin 28, DIO_5)        | Bootloader backdoor, active high   |
| GP26        | ~SRST (pin 17)              | Reset, active low, driven open-drain|
| GND         | GND                         |                                    |

The module's firmware must have the bootloader backdoor on (every build from this repository,
`adsbee_1421-0.3.7` onward, unless `AT+BOOTLOADER_PIN=0,DEADBEE` turned it off). Older boards need
one JTAG flash first. See
[`../README.md`](../README.md#reflashing-over-uart-the-sync-bootloader-backdoor).

## Build

The baked firmware image comes from the adsbee_1421 build of the same configuration, so build that first:

```sh
cd firmware/adsbee_1421
./build.sh ti
./build.sh programmer
```

With `-d` (`./build.sh -d ti`, `./build.sh -d programmer`, or `./build.sh -d build_and_flash`) both are
built in Debug and the Programmer bakes in `ti/build/Debug/adsbee_1421.hex`. Only Debug images have the RF test
command `AT+TX_CW`; see [firmware/README.md](../../README.md#debug-builds-and-rf-test-commands).

Artifact: `firmware/adsbee_1421/programmer/build/Release/adsbee_1421_programmer.uf2` —
hold BOOT on the RP2040-Zero while plugging it in (or open its port at 233495534 baud, see below) and
drag the file onto the `RPI-RP2` drive.
Or run `./build.sh build_and_flash`, which builds both apps, prompts for bootloader mode, copies
the uf2, and watches the Programmer's console while it reflashes the attached m1421. `./build.sh flash`
does the same without building, using the uf2 already on disk; it warns if the ti hex is newer than the Programmer image.
A version-stamped copy (`adsbee_1421_programmer-fw<version>.uf2`, named after the **baked**
firmware version parsed from `object_dictionary.cpp`) is produced alongside it for release/CI.
To bake a different image, pass `-DADSBEE_1421_HEX=<path>` to CMake.

## Buttons

- **BOOTSEL held at power-up**: force a reflash even if the CRC matches.
- **BOOTSEL tapped during pass-through**: rerun the check/flash cycle and find the console again.
- **BOOTSEL held for 3 s (at any time)**: arm a **settings erase**. The LED blinks white, and at
  the next bootloader entry the Programmer erases the four Settings sectors
  (`0x000FC000`–`0x000FDFFF`); the device then boots with factory defaults. Device Info
  (`0x000FE000`, part code and OTA keys) is never touched.

  Use it when saved settings stop the console from coming up (`Device console not responding ...`),
  where `AT+SETTINGS=RESET` can't be sent. It is never triggered automatically.

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
| green         | pass-through                             |
| red           | error (automatic retry follows)          |

## Pass-through behavior

The bridge acts like a USB-UART adapter wired `RTS → SYNC`, `DTR → RESET_N` (asserting a line drives
the pin low), so a host tool can enter the ROM bootloader and reflash the module with no buttons (see the
[pyserial example](../README.md#using-it-from-a-host)). RTS and DTR are honored only in pass-through
(green LED):

- **RTS asserted → SYNC low** (device awake); **RTS deasserted → SYNC high** (device asleep /
  backdoor armed) while DTR is asserted. Opening the port asserts both, so deasserting RTS on an
  open port puts the module to sleep.
- **DTR assert edge → 50 ms reset pulse**. SYNC follows RTS at the edge and is held 250 ms after
  the pulse. Holding DTR asserted doesn't hold the device in reset.
- **With DTR deasserted, SYNC is low**, so **closing the port leaves the module awake**.
- **The host's baud rate is ignored** (see [Baud rate](#baud-rate)), also for a ROM bootloader client.
- **Except 233495534 baud (`0xDEADBEE`)**, which reboots the Programmer into its USB bootloader
  (`RPI-RP2`) for updates without pressing BOOT, e.g.
  `python3 -c "import serial, time; s = serial.Serial('/dev/ttyACM0', 115200); s.baudrate = 0xDEADBEE; time.sleep(1); s.close()"`
  (keep the port open for a second). Same magic baud as the ADSBee 1090. 1200 baud is an ordinary
  rate. Older Programmer images need BOOT held while plugging in once.

### Baud rate

The Programmer's UART runs at the console's rate (9600 to 3,000,000), measured from the `UU` the
module sends (see [Console autobaud](../README.md#console-autobaud)).

To change the rate, send `AT+BAUD_RATE=CONSOLE,<n>` and keep using the same open port:

- **Wait for the `OK` before sending the next command.** Commands sent before it are lost.
- The new rate is live only until `AT+SETTINGS=SAVE`; the Programmer never saves it.
- After a reboot or `AT+SETTINGS=RESET` the Programmer follows the module back to its saved or
  default rate.
- Rates outside 9600 to 3,000,000 return `ERROR` and change nothing.

### Finding the console

The Programmer timestamps every edge on its RX pin. When reception looks wrong (framing errors,
breaks, edges that don't fit), it retunes to the newest `UU`. If there is none, it sends a NUL and a
2 ms break and locks onto the answer: 3 tries, then once a second.

- It never sends a break while SYNC is high or in the ROM bootloader.
- A break drops the module's queued output, so it only asks when the rate is unknown.
- Console output reaches the host 1.5 ms late, so bytes misread before a rate change are dropped.

### ROM bootloader through the Programmer

- Open the port first (opening resets the module into its application), then enter the bootloader
  with RTS deasserted and a DTR edge. The web console's **Enter bootloader** does this.
- If a different image gets flashed, the Programmer restores its baked image at its next check
  (power-up, BOOTSEL tap, or failed console search).

## Troubleshooting (yellow blink / no green)

Open the Programmer's USB serial port (any terminal, any baud, e.g. `python3 -m serial.tools.miniterm`) to
see diagnostics. The last one repeats every ~5 s while waiting.

| Message | Meaning |
|---------|---------|
| `SBL sync at <baud>` | Bootloader entry works; that rate is reused for the session. |
| `... failed (timed out ...); no late bytes` | Silence from the module: reset, UART, or SYNC not reaching it. |
| `... failed (unexpected byte ...)` / `late bytes: ...` | The module answered with garbage: baud/framing issue on the link. |
| `App console responds at <baud> ... SBL entry fails` | Reset and UART wiring are good. Check SYNC (GP27 → module pin 28) and that the backdoor is on (`AT+BOOTLOADER_PIN?` answers `BOOTLOADER_PIN=1`). Pass-through follows without the image check (next row). |
| `WARNING: ROM bootloader entry failed but the application console answers ...` | The module runs whatever it has. If the backdoor is off, send `AT+BOOTLOADER_PIN=1,DEADBEE`, then tap BOOTSEL. |
| `No response ... RESET_N(GP26)=LOW (stuck in reset ...)` | Something is holding reset low with the Programmer's driver released: a wiring short or a drive conflict on ~SRST. |
| `No response ... UART RX(GP29)=LOW (module TX not driving ...)` | Module unpowered, held in reset, or SUTX wiring wrong (RX should idle high when the module runs). |
| `No response ... RESET_N=high, UART RX=high (link plausible)` | Lines look electrically sane; suspect TX leg (GP28 → SURX) or module-side UART config. |
| `Device console not responding` | Flashing worked but no `UU` came. If it repeats every boot, saved settings are the likely cause: hold BOOTSEL 3 s to erase them (see [Buttons](#buttons)). |
| `[ADSBee 1421 Programmer] Console not found; UART left at <baud> baud, asking again every 1 s.` | The console stopped answering (unplugged, in reset, or firmware without `UU`). Host data still flows at that rate. |
| `Settings erase ARMED` / `Settings erased; ...` | A BOOTSEL long press was registered, and the Settings sectors were erased at the next bootloader entry. The device now boots with factory defaults. |

Modules running pre-backdoor firmware can't be entered via SYNC at all: flash them once via
JTAG. For a module with the backdoor turned off, see
[Turning the backdoor off](../README.md#turning-the-backdoor-off-atbootloader_pin).

## Limitations

- Commands sent right after `AT+BAUD_RATE=CONSOLE,<n>`, before its `OK`, are lost.
- Console output reaches the host 1.5 ms late.
- Status text (flash progress, negotiation results) appears on the same CDC port before
  pass-through starts; anything typed during those phases is ignored.
- 233495534 baud reboots the Programmer into its USB bootloader (see
  [Pass-through behavior](#pass-through-behavior)).

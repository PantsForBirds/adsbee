# ADSBee 1421 Programmer

A Waveshare RP2040-Zero application that keeps an attached ADSBee m1421 (TI CC1314R10) flashed
with the firmware image baked into the Programmer, then acts as a transparent USB serial adapter to the
module's console. The host can open the port at any baud rate; the Programmer follows the console's rate (see
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
   are left untouched. `hex_to_c.py` refuses to bake, and the Programmer refuses to flash, an image that
   reaches into those reserved sectors.
3. Resets the module into the application, finds the console at its saved baud rate (see
   [Finding the console](#finding-the-console)), and becomes a USB ↔ UART pass-through.

If the bootloader can't be entered but the application console answers (for example after
`AT+BOOTLOADER_PIN=0,DEADBEE` turned the backdoor off), the Programmer prints a warning, skips steps 1 and
2, and enters pass-through anyway, so the console stays reachable. A forced reflash or an armed
settings erase can't run in that state; the settings erase stays armed.

The ROM bootloader always runs at 1,000,000 baud. The Programmer never changes the console's rate or
sends `AT+SETTINGS=SAVE` on its own. If no module responds, the Programmer retries forever (attach a
module any time) and reports a diagnosis on the CDC port every few seconds.

## Wiring

| RP2040-Zero | ADSBee m1421                | Notes                              |
|-------------|-----------------------------|------------------------------------|
| GP28        | SURX (pin 20, DIO_2 UART RX)| UART0 TX                           |
| GP29        | SUTX (pin 21, DIO_3 UART TX)| UART0 RX                           |
| GP27        | SYNC (pin 28, DIO_5)        | Bootloader backdoor, active high   |
| GP26        | ~SRST (pin 17)              | Reset, active low, driven open-drain|
| GND         | GND                         |                                    |

The module's firmware must have the CCFG bootloader backdoor enabled: every adsbee_1421 build
from this repository does (release `adsbee_1421-0.3.7` onward; it is set in
`firmware/adsbee_1421/ti/syscfg/adsbee_1421.syscfg`), unless `AT+BOOTLOADER_PIN=0,DEADBEE` turned it
off on that module. Older boards need one JTAG flash first. The backdoor itself, its wiring
convention, `AT+BOOTLOADER_PIN` and the SYNC sleep interaction are documented in
[`../README.md`](../README.md#reflashing-over-uart-the-sync-bootloader-backdoor).

## Build

The baked firmware image comes from the adsbee_1421 build of the same configuration, so build that first:

```sh
cd firmware/adsbee_1421
./build.sh ti
./build.sh programmer
```

With `-d` (`./build.sh -d ti`, `./build.sh -d programmer`, or `./build.sh -d build_and_flash`) both are
built in Debug and the Programmer bakes in `ti/build/Debug/adsbee_1421.hex`. Only Debug images contain the RF test
command `AT+TX_CW`; see [firmware/README.md](../../README.md#debug-builds-and-rf-test-commands).

Artifact: `firmware/adsbee_1421/programmer/build/Release/adsbee_1421_programmer.uf2` —
hold BOOT on the RP2040-Zero while plugging it in (or, on a Programmer already running a build with the
magic-baud reboot, open its port at 233495534 baud, see below) and drag the file onto the `RPI-RP2` drive.
Or run `./build.sh build_and_flash`, which builds both apps, prompts for bootloader mode, copies
the uf2, and watches the Programmer's console while it reflashes the attached m1421. `./build.sh flash`
does the same without building, using the uf2 already on disk; because the ti hex is baked in at
configure time, it warns if that hex is newer than the Programmer image.
A version-stamped copy (`adsbee_1421_programmer-fw<version>.uf2`, named after the **baked**
firmware version parsed from `object_dictionary.cpp`) is produced alongside it for release/CI.
To bake a different image, pass `-DADSBEE_1421_HEX=<path>` to CMake.

## Buttons

- **BOOTSEL held at power-up**: force a reflash even if the CRC matches.
- **BOOTSEL tapped during pass-through**: rerun the check/flash cycle and find the console again.
  This is also the recovery path after in-band desyncs (see limitations).
- **BOOTSEL held for 3 s (at any time)**: arm a **settings erase**. The LED blinks white, and at
  the next bootloader entry the Programmer erases the four Settings sectors
  (`0x000FC000`–`0x000FDFFF`); the device then boots with factory defaults. Device Info
  (`0x000FE000`, part code and OTA keys) is never touched.

  This is the escape hatch for a device whose saved settings stop the console coming up. It can
  be armed from the wait loops as well as from pass-through, so it works while the Programmer is
  stuck reporting `Device console not responding ...`. In that state `AT+SETTINGS=RESET` is
  unavailable, because it needs a console that already answers. It is never triggered
  automatically, because it discards the user's settings.

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

The bridge emulates a TTL USB-UART adapter wired the way the existing host tools expect
(`adapter RTS → SYNC`, `DTR → RESET_N`, where asserting a modem-control line drives the
physical pin low), so a host tool that drives RTS/DTR can put the module into the ROM bootloader
and reflash it through the Programmer with no button presses (see the
[pyserial example](../README.md#using-it-from-a-host)). The modem-control lines are honored only
in pass-through (green LED):

- **RTS asserted → SYNC low** (device awake); **RTS deasserted → SYNC high** (device asleep /
  backdoor armed) while DTR is asserted. Opening the port asserts both, so deasserting RTS on an
  open port puts the module to sleep.
- **DTR assert edge → 50 ms reset pulse**, with SYNC set from RTS at the edge and held for 250 ms
  after the pulse so the boot ROM samples it. The reset is edge-triggered, so a terminal that
  keeps DTR asserted doesn't hold the device in reset.
- **With DTR deasserted, SYNC is low** (outside that post-reset hold), so **closing the port
  leaves the module awake**: the OS drops DTR and RTS together on close (Linux and macOS with the
  default HUPCL), which used to read as RTS deasserted and put the module to sleep until the next
  open. Once in the ROM bootloader the module no longer looks at SYNC, and a host still gets back
  in with RTS deasserted and another DTR edge.
- **The host's baud rate is virtual** (see [Baud rate](#baud-rate)), including for a host-side
  ROM bootloader client.
- **Except 233495534 baud (`0xDEADBEE`)**, which reboots the Programmer's RP2040 into its USB bootloader
  (`RPI-RP2`) so the Programmer can be updated without pressing BOOT, e.g.
  `python3 -c "import serial, time; s = serial.Serial('/dev/ttyACM0', 115200); s.baudrate = 0xDEADBEE; time.sleep(1); s.close()"`
  (keep the port open for a second; on some Linux hosts an immediate close doesn't reach it). It is the same
  magic baud as the ADSBee 1090 (`PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE` in
  `firmware/adsbee_1090/pico/CMakeLists.txt`; the Programmer's copy is `kRebootToBootselBaud` in
  `host_line_coding.hh`, and the host test checks they match). 1200 baud is an ordinary rate here,
  as on the 1090, because tools such as pymavlink open ports at 1200. Programmer images without the magic baud need BOOT held while plugging in
  once to update.

### Baud rate

The host's baud rate is ignored: the Programmer's UART runs at the module console's rate, any rate
from 9600 to 3,000,000. The module sends `UU` after every rate change, at boot, and in answer to a
break (see [Console autobaud](../README.md#console-autobaud)); the Programmer measures the rate from
it and follows.

To change the rate, send `AT+BAUD_RATE=CONSOLE,<n>` and keep using the same open port:

- **Wait for the `OK` before sending the next command.** Commands sent before it are lost.
- The new rate is live only until `AT+SETTINGS=SAVE`; the Programmer never saves it.
- After a reboot or `AT+SETTINGS=RESET` the Programmer follows the module back to its saved or
  default rate.
- Rates outside 9600 to 3,000,000 return `ERROR` and change nothing.

### Finding the console

A PIO state machine timestamps every edge on the Programmer's RX pin into a DMA ring
(`edge_capture.cc`, `autobaud_edges.pio`). `rate_watch.cc` checks the ring when something looks
wrong (framing or break errors, or edges that don't fit the current rate) and retunes to the rate
of the newest `UU`. If it finds none, it asks: it sends a NUL, then a 2 ms break, and locks onto the
module's `UU` answer, up to 3 tries, then once a second.

- It never sends a break while SYNC is high (module may be asleep) or in the ROM bootloader.
- A break makes the module drop its queued output, so the Programmer only asks when it doesn't
  know the rate.
- Console output reaches the host 1.5 ms late, so bytes misread before a rate change are dropped.
- Lock takes about 2 ms (5 ms at 9600); the boot `UU` comes about 45 ms after reset.

### ROM bootloader through the Programmer

A host-side ROM bootloader client can flash a module through the Programmer: enter the bootloader
with RTS deasserted and a DTR edge, after the port is open (opening it resets the module into its
application). While SYNC is high the Programmer is fully transparent. The web console's **Enter
bootloader** button works this way. If a different image gets flashed, the Programmer restores its
baked image at its next check (power-up, BOOTSEL tap, or failed console search).

## Troubleshooting (yellow blink / no green)

Open the Programmer's CDC port (any terminal, any baud, e.g. `python3 -m serial.tools.miniterm`) to
see per-attempt diagnostics. The Programmer re-prints its last diagnosis every ~5 s while waiting.

| Message | Meaning |
|---------|---------|
| `SBL sync at <baud>` | Bootloader entry works; that rate is reused for the session. |
| `... failed (timed out ...); no late bytes` | Silence from the module: reset, UART, or SYNC not reaching it. |
| `... failed (unexpected byte ...)` / `late bytes: ...` | The module answered with garbage: baud/framing issue on the link. |
| `App console responds at <baud> ... SBL entry fails` | Reset + UART wiring are good. Check SYNC (GP27 → module pin 28) and that the module firmware has the CCFG backdoor enabled (`AT+BOOTLOADER_PIN?` answers `BOOTLOADER_PIN=1`). The Programmer then enters pass-through without the image check (next row). |
| `WARNING: ROM bootloader entry failed but the application console answers ...` | Pass-through without the image check: the module runs whatever it has, which may differ from the baked image. If the backdoor is off, send `AT+BOOTLOADER_PIN=1,DEADBEE`, then tap BOOTSEL to rerun the check. |
| `No response ... RESET_N(GP26)=LOW (stuck in reset ...)` | Something is holding reset low with the Programmer's driver released: a wiring short or a drive conflict on ~SRST. |
| `No response ... UART RX(GP29)=LOW (module TX not driving ...)` | Module unpowered, held in reset, or SUTX wiring wrong (RX should idle high when the module runs). |
| `No response ... RESET_N=high, UART RX=high (link plausible)` | Lines look electrically sane; suspect TX leg (GP28 → SURX) or module-side UART config. |
| `Device console not responding` | Flashing worked but the console sent no `UU` at boot or after 3 breaks. If it repeats on every boot with a CRC-verified image, the saved settings are the likely cause: hold BOOTSEL for 3 s to erase them (see [Buttons](#buttons)). |
| `[ADSBee 1421 Programmer] Console not found; UART left at <baud> baud, asking again every 1 s.` | The console stopped answering (unplugged, in reset, or firmware without `UU`). Host data still flows at that rate. |
| `Settings erase ARMED` / `Settings erased; ...` | A BOOTSEL long press was registered, and the Settings sectors were erased at the next bootloader entry. The device now boots with factory defaults. |

Modules running pre-backdoor firmware can't be entered via SYNC at all: flash them once via
JTAG. A module whose backdoor was turned off with `AT+BOOTLOADER_PIN=0,DEADBEE` behaves the same way, and
the Programmer enters pass-through without the image check (`WARNING: ROM bootloader entry failed
but the application console answers ...`). Send `AT+BOOTLOADER_PIN=1,DEADBEE` through it, then tap BOOTSEL
to rerun the check. See
[Turning the backdoor off](../README.md#turning-the-backdoor-off-atbootloader_pin).

## Limitations

- Commands sent right after `AT+BAUD_RATE=CONSOLE,<n>`, before its `OK`, are lost.
- Console output reaches the host 1.5 ms late.
- Status text (flash progress, negotiation results) appears on the same CDC port before
  pass-through starts; anything typed during those phases is ignored.
- Opening the CDC port at 233495534 baud reboots the Programmer into its USB bootloader (see
  [Pass-through behavior](#pass-through-behavior)). Every other baud works for reading its
  diagnostics.

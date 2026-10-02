# ADSBee 1421 Programmer

A Waveshare RP2040-Zero application that keeps an attached ADSBee m1421 (TI CC1314R10) flashed
with the firmware image baked into the Programmer, then acts as a transparent USB serial adapter to the
module's console, at whatever baud rate the host opens the port at (see [Baud rate](#baud-rate)).

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
3. Resets the module into the application, finds the console (it boots at its saved console baud,
   persisted via `AT+SETTINGS=SAVE`; see [Finding the console](#finding-the-console)), drives its
   live rate to the host's rate (1 M if no host has set one), and becomes a USB-CDC ↔ UART
   pass-through.

If the bootloader can't be entered but the application console answers (for example after
`AT+BOOTLOADER_PIN=0,DEADBEE` turned the backdoor off), the Programmer prints a warning, skips steps 1 and
2, and enters pass-through anyway, so the console stays reachable. A forced reflash or an armed
settings erase can't run in that state; the settings erase stays armed.

The ROM bootloader leg always runs at 1 M (the ROM auto-bauds to it; ceiling ~1.2 M). The
Programmer only moves the console's *live* rate. It never issues `AT+SETTINGS=SAVE`, so the
module's persisted baud setting is untouched. If no module responds, the Programmer retries forever (attach a
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
- **BOOTSEL tapped during pass-through**: rerun the check/flash cycle and re-negotiate the
  console (find it, then move it to the host's rate or 1 M). This is also the recovery path after
  in-band desyncs (see limitations).
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
- **Host baud changes move the module console to the host's rate** (see [Baud rate](#baud-rate)),
  so a ground station that opens the port at 57600 reads the console at 57600. In the ROM
  bootloader they go to the UART directly, so a host-side bootloader client works unmodified.
- **Except 233495534 baud (`0xDEADBEE`)**, which reboots the Programmer's RP2040 into its USB bootloader
  (`RPI-RP2`) so the Programmer can be updated without pressing BOOT, e.g.
  `python3 -c "import serial, time; s = serial.Serial('/dev/ttyACM0', 115200); s.baudrate = 0xDEADBEE; time.sleep(1); s.close()"`
  (on some Linux hosts, opening the port at that rate and closing it at once often doesn't reach the
  Programmer; setting the rate on an open port and keeping it open for a second does). It is the same
  magic baud as the ADSBee 1090 (`PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE` in
  `firmware/adsbee_1090/pico/CMakeLists.txt`; the Programmer's copy is `kRebootToBootselBaud` in
  `host_line_coding.hh`, and the host test checks they match). That baud is never forwarded to the
  module. 1200 baud is an ordinary rate here, as on the 1090, because tools such as pymavlink open
  ports at 1200. Programmer images without the magic baud need BOOT held while plugging in
  once to update.

### Baud rate

The ADSBee 1421 console accepts any rate from 9600 to 3,000,000 baud that its UART generates
within 2% (`AT+BAUD_RATE=CONSOLE,<baud>`; every rate in that range qualifies, see
`firmware/adsbee_1421/ti/comms/console_baud.hh`). The host's line coding is what a ground station
or terminal expects on the wire, so when the host sets a rate in that range during pass-through, the
Programmer moves the console there: it sends `AT+BAUD_RATE=CONSOLE,<new>` at the console's current
rate, waits for the `OK`, retunes its own UART and checks that the console answers at the new rate.
Hosts often set several rates while opening a port (pymavlink sets 1200, then its own rate), so the
Programmer waits until the line coding has been stable for 100 ms and only acts on the last rate.
Host data sent meanwhile stays in the USB buffer and reaches the console at the new rate. The
Programmer only moves the *live* rate; it never sends `AT+SETTINGS=SAVE`.

- **Opening the port resets the module** (DTR edge, SYNC low), and the console comes back at its
  saved rate, which the Programmer measures (see [Finding the console](#finding-the-console)). If
  that is the host's rate, the Programmer just retunes its UART; otherwise it moves the console
  right away. A tool can talk to the console about 0.1 s after opening.
- **Rates outside 9600–3,000,000** (the 1200 baud pymavlink opens ports at, 300, 4 M, ...) go to
  the Programmer's UART directly, as does every rate while the module is in the ROM bootloader
  (after a reset with SYNC high, until the next reset with SYNC low).
- **If the console doesn't acknowledge** the move within 1 s, the Programmer finds it again with
  the autobaud trigger on a SYNC wake, then moves it from there.
- **If the console can't be moved** (it isn't found, or runs an image that refuses the rate),
  pass-through carries on at the rate the console is at and the Programmer writes one warning line
  (`[ADSBee 1421 Programmer] Console stays at <baud> baud ...`) to the host. The USB rate is
  virtual, so the host still reads clean data. The next rate change tries again.
- **Commands that move the console** are recognized in the host's traffic:
  `AT+BAUD_RATE=CONSOLE,<n>` (the next move tries `<n>` first, so a host that sends it and then
  changes its own rate, like the web console, resyncs at once), `AT+SETTINGS=RESET` (back to 1 M)
  and `AT+REBOOT` (back to the saved rate: the Programmer resets the module and locks again, as
  when the port is opened).

### Finding the console

The Programmer measures the console's rate with the module's autobaud trigger (see
[Console autobaud](../README.md#console-autobaud)): it holds the module's RX line (SURX, GP28)
low through a reset into the application, releases it 30 ms after the reset, and the module answers
with `UU` at its console rate once the console is ready (about 50 ms after the reset). A PIO state
machine (`autobaud_edges.pio`, on PIO1; the LED uses PIO0) timestamps the edges on GP29 at the
125 MHz system clock while the UART keeps the pin. `autobaud.cc` finds the square wave of the
`U`s (each bit an edge) among them and averages the bit time over an even number of bits, so a
duty-cycle distortion of the line cancels. Glitches, other bytes and rates outside the console's
range are rejected (host tests: `host_test/autobaud_test.cc`, against a cycle-level model of the
PIO program). In the host tests the measurement is within 0.21% of the rate the module's UART
generates, at any rate from 9600 to 3,000,000. On the bench it is within 0.05% after a reset, and
up to 0.5% high after a SYNC wake, when the module still runs on its RC oscillator. The Programmer
snaps it to the nearest rate the module's
UART can generate (near 3 M those are 1.5% apart), confirms it with one `AT+BAUD_RATE?`, and then
uses the exact rate in the answer.

The same trigger on a SYNC wake (SYNC high for 20 ms, RX held low until 40 ms after SYNC drops)
finds a console that was lost in pass-through without resetting the module, so its live settings
stay.

Module firmware without the trigger (0.3.11-rc3 and earlier) sends no `UU`. Those images only
accept 1 M, 921600, 460800, 230400 and 115200, so after 70 ms without an answer the Programmer
probes each of those once with `AT+BAUD_RATE?`.

Times, measured on a module behind the Programmer (typical) and the bound from the timeouts (worst
case). A probe (`AT+BAUD_RATE?`) waits up to 0.25 s plus the time for 2 KB at the probed rate, at
most 1 s; `AT+BAUD_RATE=CONSOLE,<n>` waits up to 1 s for its `OK`.

| Step | Typical | Worst case |
|---|---|---|
| Reset and lock (startup, port open, `AT+REBOOT`) | 0.10 s (1 M) to 0.15 s (9600) | 0.15 s without a `UU`, plus the probes below |
| Then move the console to the host's rate | a few ms + 50 ms switch + one probe | 1 s + 50 ms + 2 probes |
| Host rate change, console where expected | 0.1 s settle + a few ms + 50 ms + one probe | as above, then the lost-console path |
| Lost console (no `OK`): lock on a SYNC wake, then move | 1.3 s in total (mostly the 1 s `OK` timeout) | 1 s + 0.11 s + 2 probes + the move |
| Module firmware without the trigger: legacy probe pass | 0.15 s + the probes up to the saved rate | 0.15 s + 5 probes (1.6 s) |

Each command the Programmer sends starts with a blank line, which closes any garbage the held-low
line or a wrong-rate byte left in the console's AT line buffer.

### ROM bootloader through the Programmer

A host-side ROM bootloader client can flash a module *through* the Programmer: it enters the
bootloader with RTS deasserted and a DTR edge, and while SYNC is high the Programmer stays fully
transparent and never injects traffic. The web console's **Enter bootloader** button works this
way. Opening the port is itself a DTR edge (the OS asserts DTR and RTS), so it resets the module
into its application; enter the bootloader after the port is open. If any tool flashes a
different image, the Programmer reflashes its baked image at its next recheck (power-up, BOOTSEL
tap, or failed console negotiation).

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
| `Device console not responding` | SBL/flash worked but the app's console neither answered the autobaud trigger nor `AT+BAUD_RATE?` at the legacy rates (see [Finding the console](#finding-the-console)). If it repeats on every boot with a CRC-verified image, the saved settings are the likely cause: hold BOOTSEL for 3 s to erase them (see [Buttons](#buttons)). |
| `Settings erase ARMED` / `Settings erased; ...` | A BOOTSEL long press was registered, and the Settings sectors were erased at the next bootloader entry. The device now boots with factory defaults. |

Modules running pre-backdoor firmware can't be entered via SYNC at all: flash them once via
JTAG. A module whose backdoor was turned off with `AT+BOOTLOADER_PIN=0,DEADBEE` behaves the same way, and
the Programmer enters pass-through without the image check (`WARNING: ROM bootloader entry failed
but the application console answers ...`). Send `AT+BOOTLOADER_PIN=1,DEADBEE` through it, then tap BOOTSEL
to rerun the check. See
[Turning the backdoor off](../README.md#turning-the-backdoor-off-atbootloader_pin).

## Limitations

- `AT+BAUD_RATE=CONSOLE,<n>` sent through the bridge moves the console to `<n>` while the
  Programmer's UART stays at the host's rate, so the link is garbled until the host sets its line
  coding (to `<n>`, or any rate: the Programmer finds the console and moves it) or reopens the port.
  BOOTSEL also recovers.
- Status text (flash progress, negotiation results) appears on the same CDC port before
  pass-through starts; anything typed during those phases is ignored.
- Opening the CDC port at 233495534 baud reboots the Programmer into its USB bootloader (see
  [Pass-through behavior](#pass-through-behavior)). Every other baud works for reading its
  diagnostics.

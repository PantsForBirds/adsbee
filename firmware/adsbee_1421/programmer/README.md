# ADSBee 1421 Programmer

A Waveshare RP2040-Zero application that keeps an attached ADSBee m1421 (TI CC1314R10) flashed
with the firmware image baked into the Programmer, then acts as a transparent USB serial adapter to the
module's console. The USB baud rate is virtual: the host can open the port at any rate, and the Programmer's UART
runs at the console's rate and follows it when the host changes it (see [Baud rate](#baud-rate)).

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
   persisted via `AT+SETTINGS=SAVE`; see [Finding the console](#finding-the-console)), sets its UART
   to that rate, and becomes a USB-CDC ↔ UART pass-through.

If the bootloader can't be entered but the application console answers (for example after
`AT+BOOTLOADER_PIN=0,DEADBEE` turned the backdoor off), the Programmer prints a warning, skips steps 1 and
2, and enters pass-through anyway, so the console stays reachable. A forced reflash or an armed
settings erase can't run in that state; the settings erase stays armed.

The ROM bootloader leg always runs at 1 M (the ROM auto-bauds to it; ceiling ~1.2 M). The
Programmer never changes the console's rate or sends `AT+SETTINGS=SAVE` on its own, so the module's
settings are the user's. If no module responds, the Programmer retries forever (attach a
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
- **The host's baud rate is virtual** (see [Baud rate](#baud-rate)): a ground station that opens
  the port at 57600 reads the console at whatever rate the module runs at. In the ROM bootloader the
  UART runs at 1 M, and the ROM auto-bauds to it, so a host-side bootloader client works at any host
  rate.
- **Except 233495534 baud (`0xDEADBEE`)**, which reboots the Programmer's RP2040 into its USB bootloader
  (`RPI-RP2`) so the Programmer can be updated without pressing BOOT, e.g.
  `python3 -c "import serial, time; s = serial.Serial('/dev/ttyACM0', 115200); s.baudrate = 0xDEADBEE; time.sleep(1); s.close()"`
  (on some Linux hosts, opening the port at that rate and closing it at once often doesn't reach the
  Programmer; setting the rate on an open port and keeping it open for a second does). It is the same
  magic baud as the ADSBee 1090 (`PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE` in
  `firmware/adsbee_1090/pico/CMakeLists.txt`; the Programmer's copy is `kRebootToBootselBaud` in
  `host_line_coding.hh`, and the host test checks they match). 1200 baud is an ordinary rate here,
  as on the 1090, because tools such as pymavlink open ports at 1200. Programmer images without the magic baud need BOOT held while plugging in
  once to update.

### Baud rate

The USB CDC baud rate is virtual. The host can open the Programmer's port at any rate (9600,
57600, 115200, 3 M, ...) and the Programmer ignores it: its UART runs at the module console's
rate, which it measures after every reset (see [Finding the console](#finding-the-console)). The
ADSBee 1421 console accepts any rate from 9600 to 3,000,000 baud that its UART generates within 2%
(`AT+BAUD_RATE=CONSOLE,<baud>`; every rate in that range qualifies, see
`firmware/adsbee_1421/ti/comms/console_baud.hh`).

**Changing the rate on the fly.** Send `AT+BAUD_RATE=CONSOLE,<n>` through the Programmer and keep
talking on the same open port, at the same host rate. The Programmer (`rate_tracker.hh`):

1. recognizes the command in the host's traffic. The module runs a line when its `\n` arrives, so
   the Programmer forwards the bytes up to that `\n` and holds everything the host sends after it
   in the USB buffer;
2. watches the console's output for the reply. The module answers `OK` at the old rate, finishes
   sending it, then switches. The Programmer forwards the output up to the end of `OK\r\n` and
   retunes its UART to `<n>` right away, before the module sends anything at the new rate;
3. releases the held host data, which reaches the console at the new rate.

`ERROR` (the console refuses the rate) ends the hold with no retune. If neither comes within the
timeout (0.5 s plus twice the time the module's 8 kB TX buffer takes at the old rate, at most
10 s), the Programmer finds the console with the autobaud trigger on a SYNC wake, which keeps the
module's live settings, and then releases the held data. `AT+SETTINGS=RESET` is handled the same
way (back to the factory 1 M after its `OK`).

**Persisting it.** The new rate is live only. `AT+SETTINGS=SAVE` persists it, as on any device.
The Programmer sends neither command itself. Every reset the Programmer sees (startup, port open,
a host DTR edge, `AT+REBOOT`) is followed by a lock, so after a save the Programmer comes back at
the saved rate, and after a reboot without a save it follows the console back to its old rate. A
module saved at a new rate keeps it when it moves to another host: that host opens its UART at the
saved rate.

- `AT+REBOOT` holds the host's data, and 100 ms later the Programmer resets the module itself and
  locks (the module's own reboot doesn't give it the trigger).
- **The module answers at once.** The Programmer holds host data for 10 ms after the retune,
  while the module reopens its UART at the new rate (it discards what it receives meanwhile).
- **Safety net:** if the console's rate changes without the Programmer seeing the command (a
  command it doesn't parse, another module plugged in, a module power-cycled back to its saved
  rate), the Programmer finds it with a wake lock, at most once every 3 s, when either gives it
  away: framing errors (8 within 200 ms, the module talks at another rate), or silence (an `AT`
  command line got no console output at all within 2 s, because the module received garbage).
- Send `AT+BAUD_RATE=CONSOLE,<n>` once earlier commands are answered. The Programmer retunes on
  the first `OK` after the command, so an `OK` still owed to an earlier command makes it retune
  early, and the command's own `OK` reaches the host garbled (the rate itself ends up right).
- Rates the console rejects (outside 9600 to 3,000,000) pass through untouched; the console answers
  `ERROR`.

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
stay. The module drops console output it still had queued.

Module firmware without the trigger (0.3.11-rc3 and earlier) sends no `UU`. Those images only
accept 1 M, 921600, 460800, 230400 and 115200, so after 70 ms without an answer the Programmer
probes each of those once with `AT+BAUD_RATE?`.

Times, measured on a module behind the Programmer (typical) and the bound from the timeouts (worst
case). A probe (`AT+BAUD_RATE?`) waits up to 0.25 s plus the time for 2 KB at the probed rate, at
most 1 s.

| Step | Typical | Worst case |
|---|---|---|
| Reset and lock (startup, port open, DTR edge, `AT+REBOOT`) | 0.10 s (1 M) to 0.15 s (9600) | 0.15 s without a `UU`, plus the probes below |
| `AT+BAUD_RATE=CONSOLE,<n>`: host data held | until the `OK` arrives | the reply timeout, then a wake lock |
| Lost console: lock on a SYNC wake | 0.06 to 0.11 s | 0.11 s + 2 probes, then the legacy probes |
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

- The rate tracking relies on the module's `OK` (see [Baud rate](#baud-rate)). A rate change the
  Programmer misses is caught by the safety net once the module sends something or the host sends a
  command; reopening the port or tapping BOOTSEL also recovers.
- With the ROM bootloader backdoor turned off (`AT+BOOTLOADER_PIN=0,DEADBEE`), a reset with SYNC
  high starts the application, while the Programmer's UART stays at the bootloader's 1 M until the
  next reset with SYNC low.
- Status text (flash progress, negotiation results) appears on the same CDC port before
  pass-through starts; anything typed during those phases is ignored.
- Opening the CDC port at 233495534 baud reboots the Programmer into its USB bootloader (see
  [Pass-through behavior](#pass-through-behavior)). Every other baud works for reading its
  diagnostics.

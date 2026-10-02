# ADSBee 1421 Programmer

A Waveshare RP2040-Zero application that keeps an attached ADSBee m1421 (TI CC1314R10) flashed
with the firmware image baked into the Programmer, then acts as a transparent USB serial adapter to the
module's console. The USB baud rate is virtual: the host can open the port at any rate, and the Programmer's UART
runs at the console's rate and follows it when it changes (see [Baud rate](#baud-rate)).

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
rate. The ADSBee 1421 console accepts any rate from 9600 to 3,000,000 baud that its UART generates
within 2% (`AT+BAUD_RATE=CONSOLE,<baud>`; every rate in that range qualifies, see
`firmware/adsbee_1421/ti/comms/console_baud.hh`).

The Programmer doesn't read the host's commands. It follows the console itself: the module says
`UU` (0x55 0x55) at its new rate right after every rate change, at every boot, and in answer to a
UART break (see [Console autobaud](../README.md#console-autobaud)), and the Programmer measures
the rate from the edges of that square wave (see [Finding the console](#finding-the-console)).

**Changing the rate on the fly.** Send `AT+BAUD_RATE=CONSOLE,<n>` through the Programmer, wait
for its `OK`, and keep talking on the same open port at the same host rate. The module sends the
`OK` at the old rate, then switches and says `UU` at `<n>`; the Programmer sees it, retunes, and
the host's next command reaches the module at `<n>`. The same goes for `AT+SETTINGS=RESET` (back
to the factory 1 M) and for anything else that changes the rate: `AT+REBOOT` or a power cycle back
to the saved rate, another module plugged in.

- **Wait for the `OK` before sending the next command**, as on any serial line: the module reopens
  its UART at the new rate after the `OK` and flushes what it received meanwhile, so commands
  sent in the same write as the switch are lost.
- **Persisting it.** The new rate is live only. `AT+SETTINGS=SAVE` persists it, as on any device;
  the Programmer never sends it. After a reboot without a save the module comes back at its saved
  rate, and the Programmer follows its boot `UU` there. A module saved at a new rate keeps it on
  any other host, which opens its UART at the saved rate.
- Rates the console rejects (outside 9600 to 3,000,000) change nothing; the console answers `ERROR`.

### Finding the console

A PIO state machine (`autobaud_edges.pio`, on PIO1; the LED uses PIO0) timestamps every edge on
the Programmer's RX pin (GP29) at the 125 MHz system clock while the UART keeps the pin, and DMA
keeps the last 4096 in a ring (`edge_capture.cc`; a second DMA channel re-arms the first, so the
capture never stops and costs no CPU time). `rate_watch.cc` (host-tested against a bit-level model
of both UART lines and the module, `host_test/test_rate_watch.cc`) looks at the ring only when it
has a reason to:

- **Hints.** UART framing or break errors, or the newest 32 edges showing data at another rate:
  intervals shorter than 85% of a bit, intervals off the bit grid, or a `U` at another rate (a `UU`
  at exactly 1/8 of the UART's rate frames without errors). The edge check runs every 100 µs while
  edges come in, and once the line goes quiet.
- **Resolving a hint.** Host data waits while the Programmer looks for the newest `UU` around the
  hint (`autobaud.cc`: a run of 17 or more alternating one-bit intervals, averaged over an even
  number of bits so a duty-cycle distortion of the line cancels). It snaps the measurement to the
  nearest rate the module's UART generates (near 3 M those are 1.5% apart) and retunes if that
  differs from its own. Without a `UU` within 6 ms, it asks, unless the edges fit its rate (a
  glitch) or the module's factory 1 M (boot output: the module is rebooting, and its boot `UU`
  follows).
- **Asking.** When unsure (after a reset if the boot `UU` doesn't come within 150 ms, after a hint
  that didn't resolve, while the rate is unknown), the Programmer sends a NUL, waits 1.1 ms, holds
  its TX low for 2 ms (a break: longer than a frame at 9600) and locks onto the `UU` the module
  answers with. Up to 3 breaks, then the rate is unknown: the Programmer says so on the CDC port
  and asks again every second, with host data flowing in between. It never asks while SYNC is high
  (the module may be asleep), and never in the ROM bootloader.
- **Console output reaches the host 1.5 ms after it arrives,** long enough for the edge check to
  see a `U` even at 9600. A `UU` at a slower rate reads as plausible bytes at the old rate (0x00,
  0x80, ... with no framing error); this way they are dropped by the retune instead of reaching the
  host. Bytes that arrived before the `UU` started are forwarded first.
- After every lock the Programmer sends one NUL at the new rate. The module ignores NULs, and its
  UART only flags a break after a valid character (see [Console autobaud](../README.md#console-autobaud)).

The module answers a break whatever it is doing and drops the console output it still had
queued, so a break costs the host that output. The Programmer only asks when it doesn't know the
rate, when the output was unreadable anyway.

Measured on a module behind the Programmer:

| Step | Time |
|---|---|
| Reset into the application to lock (startup, port open, DTR edge, `AT+REBOOT`) | 43 ms after RESET_N is released (the boot `UU`) |
| `AT+BAUD_RATE=CONSOLE,<n>`: `OK` drained to `UU` on the line (the module reopens its UART) | 0.16 to 1 ms |
| `UU` start to retune | the `UU` (174 µs at 115200, 2.1 ms at 9600) plus up to 0.2 ms |
| Break to the module's first `UU` edge, idle module | 0.08 to 0.35 ms (1.2 ms at 9600, where a break takes a 1.04 ms frame to detect) |
| Ask to lock (NUL, idle, break, `UU`), idle module | 1.3 to 1.8 ms (5.6 ms at 9600) |

Module firmware 0.3.11-rc3 and earlier never says `UU`, and the Programmer doesn't look for those
images' consoles. It never has to: at startup it compares the module's flash with its baked image
before it looks for the console, and reflashes the module on any mismatch, so the console it looks
for always runs the baked firmware. The exception is a module whose bootloader backdoor is off
(the pass-through fallback above), which skips the image check; such a module must run firmware
that has `AT+BOOTLOADER_PIN` and so says `UU`.

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
| `Device console not responding` | SBL/flash worked but the app's console said no `UU` at boot or after 3 breaks (see [Finding the console](#finding-the-console)). If it repeats on every boot with a CRC-verified image, the saved settings are the likely cause: hold BOOTSEL for 3 s to erase them (see [Buttons](#buttons)). |
| `[ADSBee 1421 Programmer] Console not found; UART left at <baud> baud, asking again every 1 s.` | In pass-through, the console stopped answering breaks (unplugged, held in reset, or running firmware without `UU`). Host data still flows at that rate. |
| `Settings erase ARMED` / `Settings erased; ...` | A BOOTSEL long press was registered, and the Settings sectors were erased at the next bootloader entry. The device now boots with factory defaults. |

Modules running pre-backdoor firmware can't be entered via SYNC at all: flash them once via
JTAG. A module whose backdoor was turned off with `AT+BOOTLOADER_PIN=0,DEADBEE` behaves the same way, and
the Programmer enters pass-through without the image check (`WARNING: ROM bootloader entry failed
but the application console answers ...`). Send `AT+BOOTLOADER_PIN=1,DEADBEE` through it, then tap BOOTSEL
to rerun the check. See
[Turning the backdoor off](../README.md#turning-the-backdoor-off-atbootloader_pin).

## Limitations

- Commands pipelined after `AT+BAUD_RATE=CONSOLE,<n>` in the same write are lost (the module
  flushes its input when it reopens its UART). Wait for the `OK`.
- Console output reaches the host 1.5 ms late (see [Finding the console](#finding-the-console)).
- A break makes the module drop the console output it had queued. The Programmer only sends one
  when it doesn't know the console's rate.
- Status text (flash progress, negotiation results) appears on the same CDC port before
  pass-through starts; anything typed during those phases is ignored.
- Opening the CDC port at 233495534 baud reboots the Programmer into its USB bootloader (see
  [Pass-through behavior](#pass-through-behavior)). Every other baud works for reading its
  diagnostics.

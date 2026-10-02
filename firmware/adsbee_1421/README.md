# ADSBee 1421 / m1421 firmware

Firmware for the ADSBee m1421 module (TI CC1314R10 + Semtech LR2021) and the ADSBee 1421
Developer Kit.

- [`AGENTS.md`](AGENTS.md): build guide (Docker toolchain, `build.sh` targets, outputs, version
  rules), JTAG flashing, and the SYNC sleep / LR2021 bus-handoff contract.
- [`ti/`](ti/): the CC1314R10 application. [`ti/README.md`](ti/README.md) covers flashing over
  JTAG with a J-Link.
- [`programmer/`](programmer/README.md): the ADSBee 1421 Programmer, an RP2040-Zero board
  included in the Developer Kit. It keeps the module's firmware up to date and acts as its USB
  serial adapter.
- [`software/adsbee_1421_console/`](../../software/adsbee_1421_console/README.md): the
  single-file Web Serial console, which includes a UART firmware uploader.
- [m1421 datasheet](../../word/exports/datasheet_adsbee_m1421.pdf): pinout and the AT command
  reference.

## Console autobaud

The console runs at its saved baud rate (`AT+BAUD_RATE=CONSOLE,<baud>`, then `AT+SETTINGS=SAVE`):
any rate from 9600 to 3,000,000, factory default 1,000,000. It says `UU` (0x55 0x55) at its current
rate:

- **after every rate change**, right after the switch: `AT+BAUD_RATE=CONSOLE,<n>` (the `OK` still
  goes out at the old rate, then the console switches and says `UU` at `<n>`), `AT+SETTINGS=RESET`,
  and the saved rate applied at boot;
- **at every boot**, once the saved rate is applied (about 45 ms after RESET_N is released);
- **in answer to a break** on its RX line (SURX, pin 20): the line held low for longer than a frame
  at the console's rate. The main loop notices it and drops the console output it still had
  queued, since a host that sends a break doesn't know the rate. The `UU` starts 0.06 to 0.45 ms
  after the break on an idle module (1.1 to 1.3 ms at 9600, where the UART needs a whole frame to
  see a break), and a few ms later when the main loop is busy sending reports. A host that holds
  SURX low through a SYNC sleep gets `UU` on wake.

A `U` framed 8N1 is a square wave with an edge at every bit (start 0, data 1 0 1 0 1 0 1 0, stop
1), and `UU` back to back is 20 bits of it, so a host times the edges and has the rate. This is how
a LIN bus slave finds the master's rate (a break, then a 0x55 sync field), and how many
microcontroller UARTs auto-baud. The [ADSBee 1421 Programmer](programmer/README.md#finding-the-console)
follows the console this way.

- The console ignores NUL bytes. Each break leaves one in the input, and the AT parser would end a
  line there. No AT command takes binary input.
- A break that follows another with no character in between reaches the CC1314's UART as a plain
  NUL, with neither its break nor its framing error flag set, and goes unanswered. A host that may
  send breaks back to back sends a character (a NUL) after each answer and before the next break.
- Wait for the `OK` of `AT+BAUD_RATE=CONSOLE,<n>` before sending the next command: the console
  reopens its UART at the new rate and drops what it received meanwhile.
- `UU` after a rate change or at boot also reaches a host connected directly; a terminal shows it
  as text.
- In normal operation nothing changes: an idle or unconnected SURX reads high (internal pull-up),
  and checking for a break costs one register read per main loop iteration. UART2CC26X2 never
  enables the PL011's break interrupt, so the firmware reads its raw interrupt status.
- Firmware 0.3.11-rc3 and earlier doesn't say `UU`.

## Reflashing over UART: the SYNC bootloader backdoor

The m1421 can be reflashed over its console UART with no debugger and no button presses. Every
release build enables this bootloader backdoor on purpose, and it is a supported way to update the
firmware. A product that must not be reflashable this way can turn the backdoor off per module with
`AT+BOOTLOADER_PIN=0,DEADBEE`; see [Turning the backdoor off](#turning-the-backdoor-off-atbootloader_pin).

### What it is

The CC1314R10 has TI's serial bootloader in ROM. The ADSBee firmware's CCFG (the chip's boot
configuration, set in [`ti/syscfg/adsbee_1421.syscfg`](ti/syscfg/adsbee_1421.syscfg) and
generated into `ti/syscfg/ti_devices_config.c`) enables its **bootloader backdoor** on the SYNC
pin:

```js
CCFG.enableBootloader         = true;
CCFG.enableBootloaderBackdoor = true;
CCFG.dioBootloaderBackdoor    = 5;              // SYNC
CCFG.levelBootloaderBackdoor  = "Active high";
```

The boot ROM samples SYNC (DIO_5) once, when the chip comes out of reset. If SYNC is high, the
ROM skips the application and runs the serial bootloader on the console UART pins (DIO_2 RX,
DIO_3 TX). If SYNC is low, the application boots normally. After boot, SYNC is the
application's sleep request line (see [SYNC and sleep](#sync-and-sleep)).

The ROM bootloader speaks the protocol documented in the CC13x4 Technical Reference Manual
(SWCU194, "ROM Serial Bootloader"). It auto-detects the baud rate from the host's first sync
bytes (`0x55 0x55`), up to about 1.2 Mbaud. A bootloader session can read the chip ID, compute
CRC32 over flash, erase 2 KB sectors, and program flash.

### Pins and wiring

| Module pin | Signal | CC1314 pin | Role in the backdoor | USB-UART adapter | ADSBee 1421 Programmer |
|---|---|---|---|---|---|
| 20 | SURX | DIO_2 (UART RX) | ROM bootloader RX | TX | GP28 |
| 21 | SUTX | DIO_3 (UART TX) | ROM bootloader TX | RX | GP29 |
| 28 | SYNC | DIO_5 | Backdoor, **active high**, sampled at reset. Internal pull-down (plus an external 120 kΩ pull on PCBA Rev D and later), so it idles low | RTS | GP27 (push-pull) |
| 17 | ~SRST | RESET_N | Reset, **active low**. Module pull-up | DTR | GP26 (open-drain) |
| — | GND | GND | | GND | GND |

The convention is **RTS → SYNC, DTR → RESET_N**. On a typical USB-UART adapter (FTDI, CP210x,
CH340), asserting a modem-control line drives its pin **low**, so:

| Host control line | SYNC / RESET_N | Effect |
|---|---|---|
| RTS asserted | SYNC low | Application runs (awake) |
| RTS deasserted | SYNC high | Application sleeps; the next reset enters the ROM bootloader |
| DTR asserted | RESET_N low | Module held in reset (plain adapter) or a 50 ms reset pulse (Programmer, see below) |
| DTR deasserted | RESET_N released | Module runs |

Use an adapter whose I/O voltage matches the module's supply, and if you wire RESET_N to a
plain adapter's DTR, remember that a terminal which asserts DTR on open (most do) holds the
module in reset for as long as the port is open.

### Entry sequence

1. Drive SYNC high.
2. Pull RESET_N low for a few milliseconds (the ADSBee 1421 Programmer uses 50 ms), or power-cycle the module.
3. Release RESET_N while SYNC is still high. The ROM samples SYNC and starts the bootloader.
   SYNC may stay high for the rest of the session.
4. Send `0x55 0x55` at any baud rate up to about 1.2 M. The ROM answers with an ACK
   (`0x00 0xCC`) and keeps that rate for the session.
5. Erase, program, and verify.
6. Drive SYNC **low**, then reset the module (a RESET_N pulse, or the bootloader's `RESET`
   command). With SYNC still high, the reset lands back in the bootloader.

### The ADSBee 1421 Programmer does this for you

The [ADSBee 1421 Programmer](programmer/README.md) wires SYNC and RESET_N to its own GPIOs and
runs the whole sequence itself at power-up: it enters the bootloader, CRC-checks the module
against the image baked into the Programmer, flashes only the sectors that differ, and resets the
module into the application.

Once the Programmer's LED is green (pass-through), it also passes the modem-control lines of its USB
serial port through to the module, emulating a TTL USB-UART adapter wired as above:

- **RTS asserted → SYNC low; RTS deasserted → SYNC high while DTR is asserted.** Level,
  applied immediately. With DTR deasserted SYNC is low, so closing the port, which drops both
  lines, leaves the module awake.
- **DTR assert edge → 50 ms RESET_N pulse**, with SYNC taken from RTS at the edge and held for
  250 ms after the pulse so the ROM samples it. The reset is edge-triggered, so a terminal that
  holds DTR asserted doesn't hold the module in reset.
- The host's baud rate is virtual: in the ROM bootloader the Programmer's UART runs at 1 M, and the
  ROM auto-bauds to it, so a host tool can run the bootloader protocol through the Programmer at
  whatever rate it opened the port at.

Any host tool that can set RTS and DTR can therefore put the module into the bootloader and
reflash it through the Programmer remotely, with no buttons or jumpers. The Programmer ignores
RTS/DTR until it reaches pass-through, for example while it is still flashing or can't find the
module.

The Programmer treats its baked image as the source of truth: whenever it rechecks the module
(power-up, a BOOTSEL tap, or a failed console negotiation) and the module's flash doesn't match
the baked image, it reflashes the baked image. To run a different image long-term behind the
Programmer, bake it in (`-DADSBEE_1421_HEX=<path>`, see [programmer/README.md](programmer/README.md#build)).

### Using it from a host

This pyserial snippet works through the ADSBee 1421 Programmer and through a plain adapter wired
RTS → SYNC, DTR → RESET_N:

```python
import time
import serial  # pip install pyserial

s = serial.Serial()
s.port = "/dev/ttyACM0"  # the Programmer's USB serial port, or your adapter
s.baudrate = 1000000     # the ROM auto-bauds; use 115200 if your adapter can't do 1 M
s.timeout = 1
s.rts = False            # deasserted: backdoor armed for the next reset
s.dtr = False
s.open()
time.sleep(0.1)


def pulse_reset():
    s.dtr = True   # Programmer: assert edge -> 50 ms RESET_N pulse. Plain adapter: RESET_N low.
    time.sleep(0.05)
    s.dtr = False  # plain adapter: release RESET_N
    time.sleep(0.1)


pulse_reset()               # SYNC is high, so the ROM starts the serial bootloader
s.reset_input_buffer()
s.write(b"\x55\x55")        # auto-baud sync
ack = s.read(2)
assert b"\xcc" in ack, f"no bootloader ACK: {ack!r}"
print("In the ROM bootloader")

# ... erase / program / verify with a CC13x4 ROM bootloader client ...

s.rts = True                # SYNC low
pulse_reset()               # boot the application
s.close()
```

Tools that already drive the backdoor:

| Tool | Drives RTS/DTR itself? |
|---|---|
| [ADSBee 1421 Programmer](programmer/README.md) | Yes. It drives SYNC and RESET_N directly at power-up and on a BOOTSEL tap, and passes host RTS/DTR through during pass-through. |
| [Web console](../../software/adsbee_1421_console/README.md#firmware-upload-wiring) **Upload Firmware** | Yes. **Enter bootloader** drives RTS/DTR with Web Serial `setSignals()` (SYNC high, DTR deasserted → asserted 50 ms → deasserted, which suits both the Programmer and a plain adapter), checks the ROM answers, and after flashing drives SYNC low and restarts the module. On adapters without RTS/DTR wired, put the module into the bootloader by hand and press the same button; it then only runs the check. |
| Your own script | Yes, with the snippet above in front of any client that speaks the CC13x4 ROM bootloader protocol. |

Opening a serial port resets a module behind the Programmer or a DTR-wired adapter: the
operating system asserts DTR and RTS on open, which is a DTR edge with SYNC low. A module that is
in the bootloader only because of the backdoor boots back into its application. A tool should
therefore enter the bootloader after opening the port and keep the port open for the whole
session, as the web console and the snippet above do.

### SYNC and sleep

SYNC has two jobs. At reset the ROM reads it as the backdoor pin. While the application runs,
SYNC high is a request to sleep: the firmware enters STANDBY and hands the LR2021 bus to an
external MCU (see [SYNC low-power sleep](AGENTS.md#sync-low-power-sleep)). As a result:

- Return SYNC low after flashing. Otherwise the next reset lands back in the bootloader, and a
  running application treats SYNC high as a sleep request.
- Any reset while SYNC is high enters the bootloader, including a watchdog reset. If the module
  sleeps for longer than the watchdog timeout (10 s by default) and the watchdog fires, the
  module wakes up in the bootloader. For long sleeps, disable the watchdog with `AT+WATCHDOG=0`.
- On a plain adapter, closing the port can put the module to sleep. On Linux and macOS, closing
  a serial port with HUPCL set (the default) deasserts RTS and DTR. RTS deasserted is SYNC high,
  so the module sleeps until something drives SYNC low again. To keep the lines as they are after
  closing, clear HUPCL (`stty -F /dev/ttyUSB0 -hupcl`) and leave RTS asserted. The ADSBee 1421
  Programmer reads DTR deasserted as the port being closed and holds SYNC low, so a module behind
  it stays awake. Programmer images from 0.3.11-rc2 and earlier behave like a plain adapter here.

### Prerequisites

- Firmware with the backdoor enabled. Every adsbee_1421 build from this repository has it,
  starting with the first release here, `adsbee_1421-0.3.7`. `AT+DEVICE_INFO?` reports the
  running version. Older firmware from before the move into this repository may not have it.
- The backdoor still enabled in the module's CCFG: `AT+BOOTLOADER_PIN?` answers
  `BOOTLOADER_PIN=1`. See [Turning the backdoor off](#turning-the-backdoor-off-atbootloader_pin).
- For firmware without the backdoor, or a module whose SYNC pin isn't reachable, flash once over
  JTAG ([`ti/README.md`](ti/README.md)).

### Turning the backdoor off: AT+BOOTLOADER_PIN

`AT+BOOTLOADER_PIN` reads and writes the backdoor setting in the module's CCFG flash. A product
that ships the m1421 can use it to stop anyone reflashing the module by holding SYNC high through a
reset.

| Command | Effect |
|---|---|
| `AT+BOOTLOADER_PIN?` | Reads `BL_CONFIG` from the CCFG flash: `BOOTLOADER_PIN=1` when the backdoor is enabled, `0` when it isn't, followed by the raw word and its fields. This is the value the boot ROM uses at the next reset. No password. |
| `AT+BOOTLOADER_PIN=0,DEADBEE` | Disables the backdoor. SYNC high at reset no longer starts the ROM bootloader; the application boots. |
| `AT+BOOTLOADER_PIN=1,DEADBEE` | Enables the backdoor again on SYNC (DIO_5), active high, as in the release images. |
| `AT+BOOTLOADER_PIN=<0\|1>,DEADBEE,DRYRUN` | Prints the current and new `BL_CONFIG` and the write method, and writes nothing. |

Every set form takes the password `DEADBEE` as its second argument, DRYRUN included. It isn't a
secret; it makes a CCFG write something nobody does by accident (a mistyped `AT+BOOTLOADER_PIN=0`,
a script sending the wrong line). A missing or wrong password answers `ERROR` and the firmware
doesn't even read the CCFG. Enabling needs it as well as disabling: enabling is the write that
erases the CCFG sector, and one rule for every write is easier to get right than two. The web
console's **Settings** tab has a **Bootloader Pin** card that asks you to type the password by hand
before it sends the command.

The change takes effect at the next reset and persists across power cycles and
`AT+SETTINGS=RESET`. It is not one of the saved settings.

> [!WARNING]
> With the backdoor disabled, the ADSBee 1421 Programmer, the web console's **Enter bootloader**
> and any other SYNC + reset tool can no longer enter the bootloader, so they can't reflash the
> module. The only ways back are `AT+BOOTLOADER_PIN=1,DEADBEE` sent to the running firmware, or a JTAG
> flash. If the firmware's console stops answering while the backdoor is off, only JTAG can
> recover the module.
>
> Behind the ADSBee 1421 Programmer, a module with the backdoor off can't be checked against the
> Programmer's baked image or reflashed. When bootloader entry fails but the application console
> answers, the Programmer prints a warning and enters pass-through without the image check, so
> `AT+BOOTLOADER_PIN=1,DEADBEE` can still be sent through it; tap BOOTSEL afterwards to rerun the check.
> Programmer images from 0.3.11-rc3 and earlier don't have this fallback: they keep retrying the
> bootloader entry and never bridge the console. With those, connect a USB-UART adapter to
> SURX/SUTX (module pins 20/21) in place of the Programmer and send `AT+BOOTLOADER_PIN=1,DEADBEE`, or use
> JTAG.

What the command writes, per the CC13x4 Technical Reference Manual and TI driverlib:

- The CCFG is a dedicated 2 KB flash sector at `0x50000000`, separate from the 1 MB main flash.
  It holds only the 0x7C-byte CCFG structure (the linker's `FLASH_CCFG` region contains only
  `.ccfg`); no application code, settings or device info share it. The CC13x4 CCFG has no CRC.
- `BL_CONFIG` (offset `0x28`) holds `BOOTLOADER_ENABLE` (bits 31:24, `0xC5` = ROM bootloader on),
  `BL_LEVEL` (bit 16), `BL_PIN_NUMBER` (bits 15:8) and `BL_ENABLE` (bits 7:0, `0xC5` = backdoor
  on). The release images set `0xC5FF05C5`: ROM bootloader on, backdoor on DIO_5, active high.
- **Disable** sets `BL_ENABLE` to `0x00` (`0xC5FF0500`). That only clears bits, so the firmware
  programs the one word in place; the sector is never erased. `BOOTLOADER_ENABLE` stays `0xC5`,
  so a module whose application image is invalid still starts the ROM bootloader.
- **Enable** has to set bits again, which takes an erase. The firmware copies the whole sector to
  RAM, changes only `BL_CONFIG`, checks the new image, then erases the sector, programs it from
  RAM and compares the whole sector with the RAM image. If any step fails it erases again and
  programs the original image back, up to three times. When the ROM erases the CCFG sector it
  immediately writes TI's default security fields (ROM bootloader on, backdoor off, debug ports
  on), and `IMAGE_VALID_CONF` stays erased, so even a write that fails completely leaves a module
  that resets into the ROM UART bootloader, where a reflash restores the CCFG.
- Every write is verified by reading the whole sector back. On success the command prints the
  read-back `BL_CONFIG` and the CRC32 of the CCFG structure, the same CRC the ROM bootloader's
  CRC32 command returns for `0x50000000` (`0x66E9858D` for the release images' CCFG).
- A write that fails prints a `CCFG WRITE FAILED` banner as console errors (also as part of the
  command's response when `AT+LOG_LEVEL` hides errors) and answers `ERROR`, never `OK`. The banner
  says what was attempted, what failed, the read-back `BL_CONFIG` and CRC, the resulting state and
  how to recover:
  - Disable failed: the sector wasn't erased and only `BL_CONFIG` can differ; the module resets
    normally. Check with `AT+BOOTLOADER_PIN?` and retry.
  - Enable failed, original restored: nothing changed; retry.
  - Enable failed and the restore failed too: the CCFG is in an unknown state. **Don't reset the
    module.** The firmware keeps the original CCFG in RAM, and the next
    `AT+BOOTLOADER_PIN=<0|1>,DEADBEE` writes it back before anything else; repeat until it answers
    `OK`. If the module resets first, it most likely starts in the ROM UART bootloader, where the
    ADSBee 1421 Programmer (or JTAG) can reflash a complete image.
- During the write, interrupts are masked and the flash cache is off, as TI requires. An erase
  takes milliseconds; console input that arrives meanwhile is lost.
- The command refuses to write if `ERASE_CONF_1` write-protects the CCFG sector, or if the CCFG
  reads as erased.

Flashing a complete image writes that image's CCFG: the ADSBee 1421 Programmer and the web
console both erase the CCFG sector and program the image's CCFG last, and a JTAG flash of the
`.hex` includes it as well. Flashing any release image therefore turns the backdoor back on.

### Troubleshooting

| Symptom | Likely cause |
|---|---|
| The application answers with AT text after the reset, and no bootloader ACK arrives | SYNC wasn't high when RESET_N was released: check the RTS → SYNC wiring and polarity. Otherwise the backdoor is disabled (`AT+BOOTLOADER_PIN?` answers `0`; re-enable it with `AT+BOOTLOADER_PIN=1,DEADBEE`) or the firmware predates it (check `AT+DEVICE_INFO?`). The ADSBee 1421 Programmer prints `App console responds at <baud> ... SBL entry fails` for this case. |
| Nothing at all answers | Reset or UART wiring (TX/RX swapped, no common ground), module unpowered, or (plain adapter) DTR still asserted so the module is held in reset. Behind the Programmer, check that its LED is green: it ignores RTS/DTR until pass-through. |
| Garbage bytes where `0x00 0xCC` should be | Baud rate above the ROM's ~1.2 M ceiling or beyond what the adapter can do. Retry at 115200. |
| The new firmware doesn't run after flashing; the module keeps showing up in the bootloader | SYNC is still high. Assert RTS (SYNC low) and reset again. |
| The module goes quiet after a script or terminal closes the port | Plain adapter, or a Programmer image from 0.3.11-rc2 or earlier: HUPCL deasserted RTS, so SYNC is high and the module is asleep. Assert RTS again, or clear HUPCL (see [SYNC and sleep](#sync-and-sleep)). |
| The module turns up in the bootloader after a long sleep | The watchdog fired while SYNC was high. Disable it for long sleeps (`AT+WATCHDOG=0`). |
| An image flashed through the Programmer was replaced by a different version | The Programmer reflashed its baked image on a recheck (see [The ADSBee 1421 Programmer does this for you](#the-adsbee-1421-programmer-does-this-for-you)). |

The Programmer's own diagnostics for bootloader entry are listed in its
[troubleshooting table](programmer/README.md#troubleshooting-yellow-blink--no-green).

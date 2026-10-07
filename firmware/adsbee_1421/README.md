# ADSBee 1421 / m1421 firmware

Firmware for the ADSBee m1421 module (TI CC1314R10 + Semtech LR2021) and the ADSBee 1421
Developer Kit.

- [`AGENTS.md`](AGENTS.md): build guide, JTAG flashing, and the SYNC sleep / LR2021 bus handoff.
- [`ti/`](ti/): the CC1314R10 application. [`ti/README.md`](ti/README.md) covers flashing over
  JTAG with a J-Link.
- [`programmer/`](programmer/README.md): the ADSBee 1421 Programmer, an RP2040-Zero board in the
  Developer Kit. It keeps the module's firmware up to date and is its USB serial adapter.
- [`software/adsbee_1421_console/`](../../software/adsbee_1421_console/README.md): the web
  console, with a UART firmware uploader.
- [m1421 datasheet](../../word/exports/datasheet_adsbee_m1421.pdf): pinout and AT command
  reference.

## Console autobaud

The console runs at its saved baud rate: 9600 to 3,000,000, default 1,000,000
(`AT+BAUD_RATE=CONSOLE,<baud>`, then `AT+SETTINGS=SAVE`). It sends `UU` at its current rate:

- after every rate change (`OK` at the old rate, `UU` at the new one);
- at every boot;
- when it receives a break on SURX (pin 20). Queued output is dropped first.

`UU` has an edge at every bit, so a host can measure the rate from it. The
[ADSBee 1421 Programmer](programmer/README.md#finding-the-console) uses this to follow the console.

- Wait for the `OK` of `AT+BAUD_RATE=CONSOLE,<n>` before sending the next command.
- The console ignores NUL bytes (each break arrives as one).
- Send a NUL before each break: the CC1314 misses a break that directly follows another.
- Firmware 0.3.11-rc3 and earlier doesn't send `UU`.

## Reflashing over UART: the SYNC bootloader backdoor

The m1421 can be reflashed over its console UART with no debugger or buttons. Every release build
enables this on purpose. To lock a module against it, see
[Turning the backdoor off](#turning-the-backdoor-off-atbootloader_pin).

### How it works

The CC1314's ROM has TI's serial bootloader. The firmware's boot configuration (CCFG, set in
[`ti/syscfg/adsbee_1421.syscfg`](ti/syscfg/adsbee_1421.syscfg)) enables its backdoor on SYNC
(DIO_5), active high.

- The ROM reads SYNC once, as the chip leaves reset. High: it runs the bootloader on the console
  UART. Low: the application boots.
- After boot, SYNC is the application's sleep request (see [SYNC and sleep](#sync-and-sleep)).
- The bootloader detects the baud rate from the host's `0x55 0x55`, up to about 1.2 Mbaud. Its
  protocol is in the CC13x4 Technical Reference Manual (SWCU194, "ROM Serial Bootloader").

### Pins and wiring

| Module pin | Signal | CC1314 pin | Role | USB-UART adapter | ADSBee 1421 Programmer |
|---|---|---|---|---|---|
| 20 | SURX | DIO_2 | UART RX | TX | GP28 |
| 21 | SUTX | DIO_3 | UART TX | RX | GP29 |
| 28 | SYNC | DIO_5 | Backdoor, **active high**, read at reset. Pulled down (external on PCBA Rev D and later, internal before) | RTS | GP27 |
| 17 | ~SRST | RESET_N | Reset, **active low**, pulled up | DTR | GP26 (open-drain) |
| — | GND | GND | | GND | GND |

Wire **RTS → SYNC, DTR → RESET_N**. Asserting a line on a typical adapter drives its pin low:

| Host line | Pin | Effect |
|---|---|---|
| RTS asserted | SYNC low | Application runs |
| RTS deasserted | SYNC high | Application sleeps; the next reset enters the bootloader |
| DTR asserted | RESET_N low | Held in reset (plain adapter); 50 ms reset pulse (Programmer) |
| DTR deasserted | RESET_N released | Module runs |

- Match the adapter's I/O voltage to the module's supply.
- On a plain adapter, a terminal that asserts DTR on open (most do) holds the module in reset while
  the port is open.

### Entry sequence

1. Drive SYNC high.
2. Pull RESET_N low for a few milliseconds (the Programmer uses 50 ms), or power-cycle.
3. Release RESET_N with SYNC still high. SYNC may stay high for the session.
4. Send `0x55 0x55` at up to about 1.2 Mbaud. The ROM answers `0x00 0xCC` and keeps that rate.
5. Erase, program and verify.
6. Drive SYNC **low**, then reset. With SYNC still high, the module lands back in the bootloader.

### The ADSBee 1421 Programmer does this for you

At power-up the [ADSBee 1421 Programmer](programmer/README.md) enters the bootloader, compares the
module with the image baked into it, flashes only the sectors that differ, and starts the
application.

In pass-through (green LED) it passes its USB port's RTS and DTR to the module like an adapter
wired as above:

- **RTS asserted → SYNC low; RTS deasserted → SYNC high while DTR is asserted.** With DTR
  deasserted, SYNC is low, so closing the port leaves the module awake.
- **DTR assert edge → 50 ms RESET_N pulse.** SYNC follows RTS at the edge and is held 250 ms after
  the pulse. Holding DTR asserted doesn't hold the module in reset.
- The host's baud rate is ignored; the Programmer talks to the bootloader at 1,000,000 baud.
- RTS/DTR are ignored until pass-through.

> [!NOTE]
> The baked image wins. Whenever the Programmer rechecks the module (power-up, BOOTSEL tap, failed
> console search) and the flash differs, it reflashes the baked image. To run another image long
> term, bake it in (`-DADSBEE_1421_HEX=<path>`, see [programmer/README.md](programmer/README.md#build)).

### Using it from a host

Works through the Programmer and through a plain adapter wired RTS → SYNC, DTR → RESET_N:

```python
import time
import serial  # pip install pyserial

s = serial.Serial()
s.port = "/dev/ttyACM0"  # the Programmer's USB serial port, or your adapter
s.baudrate = 1000000     # use 115200 if your adapter can't do 1 M
s.timeout = 1
s.rts = False            # SYNC high: next reset enters the bootloader
s.dtr = False
s.open()
time.sleep(0.1)


def pulse_reset():
    s.dtr = True   # Programmer: 50 ms reset pulse. Plain adapter: RESET_N low.
    time.sleep(0.05)
    s.dtr = False  # plain adapter: release RESET_N
    time.sleep(0.1)


pulse_reset()               # enters the ROM bootloader
s.reset_input_buffer()
s.write(b"\x55\x55")        # baud-rate sync
ack = s.read(2)
assert b"\xcc" in ack, f"no bootloader ACK: {ack!r}"
print("In the ROM bootloader")

# ... erase / program / verify with a CC13x4 ROM bootloader client ...

s.rts = True                # SYNC low
pulse_reset()               # boot the application
s.close()
```

Tools that drive the backdoor:

- [ADSBee 1421 Programmer](programmer/README.md): at power-up, on a BOOTSEL tap, and with host
  RTS/DTR in pass-through.
- [Web console](../../software/adsbee_1421_console/README.md#firmware-upload-wiring) **Upload
  Firmware**: **Enter bootloader** drives RTS/DTR, checks the ROM answers, and restarts the module
  after flashing. Without RTS/DTR wired, enter the bootloader by hand, then press the button.
- Your own script: the snippet above plus any CC13x4 ROM bootloader client.

> [!IMPORTANT]
> Opening the port resets the module into its application (the OS asserts DTR and RTS on open).
> Enter the bootloader after opening the port and keep it open for the whole session.

### SYNC and sleep

While the application runs, SYNC high requests sleep: the firmware enters STANDBY and hands the
LR2021 bus to an external MCU (see [SYNC low-power sleep](AGENTS.md#sync-low-power-sleep)).

- Return SYNC low after flashing, or the module sleeps and the next reset enters the bootloader.
- Any reset with SYNC high, including a watchdog reset, enters the bootloader. The firmware wakes
  briefly every half watchdog timeout (5 s by default) to feed it, so sleep alone doesn't reset it.
- On a plain adapter, closing the port can put the module to sleep: Linux and macOS deassert RTS
  and DTR on close (HUPCL). To prevent this, clear HUPCL (`stty -F /dev/ttyUSB0 -hupcl`) and leave
  RTS asserted.
- Behind the Programmer the module stays awake when the port closes (Programmer images
  0.3.11-rc2 and earlier act like a plain adapter).

### Prerequisites

- Firmware with the backdoor: every adsbee_1421 build from this repository, `adsbee_1421-0.3.7`
  onward (`AT+DEVICE_INFO?` shows the version).
- The backdoor still on: `AT+BOOTLOADER_PIN?` answers `BOOTLOADER_PIN=1`.
- Otherwise, or if SYNC isn't reachable, flash once over JTAG ([`ti/README.md`](ti/README.md)).

### Turning the backdoor off: AT+BOOTLOADER_PIN

Stops anyone reflashing the module by holding SYNC high through a reset.

| Command | Effect |
|---|---|
| `AT+BOOTLOADER_PIN?` | `BOOTLOADER_PIN=1` if the backdoor is on, `0` if off, plus the raw `BL_CONFIG` word. No password. `AT+SETTINGS?JSON` also reports it (`"BOOTLOADER_PIN":[1]`, read-only). |
| `AT+BOOTLOADER_PIN=0,DEADBEE` | Turns the backdoor off. |
| `AT+BOOTLOADER_PIN=1,DEADBEE` | Turns it back on (SYNC, active high). |
| `AT+BOOTLOADER_PIN=<0\|1>,DEADBEE,DRYRUN` | Shows the current and new value; writes nothing. |

- `DEADBEE` isn't a secret; it guards against accidental writes. Without it the command answers
  `ERROR`.
- The change takes effect at the next reset and survives power cycles and `AT+SETTINGS=RESET`.
- The web console's **Settings** tab has a **Bootloader Pin** card for this.
- Flashing any complete release image (Programmer, web console, or JTAG) turns the backdoor back on.

> [!WARNING]
> With the backdoor off, the Programmer, the web console and any SYNC + reset tool can't reflash
> the module. The only ways back are `AT+BOOTLOADER_PIN=1,DEADBEE` sent to the running firmware, or
> JTAG. If the console stops answering while the backdoor is off, only JTAG can recover the module.
>
> Behind the Programmer, a module with the backdoor off isn't checked or reflashed. The Programmer
> prints a warning and enters pass-through anyway, so you can send `AT+BOOTLOADER_PIN=1,DEADBEE`,
> then tap BOOTSEL to rerun the check. Programmer images 0.3.11-rc3 and earlier retry forever
> instead: connect a USB-UART adapter to SURX/SUTX (pins 20/21) and send the command, or use JTAG.

#### What it writes

The setting lives in `BL_CONFIG` in the CCFG, a separate 2 KB flash sector at `0x50000000`
(release value `0xC5FF05C5`).

- **Off** writes `0xC5FF0500` in place, with no erase. The ROM bootloader itself stays on.
- **On** erases and rewrites the sector from a RAM copy, restoring the original on failure. An
  erased CCFG resets into the ROM bootloader, so even a total failure can be fixed by reflashing.
- Every write is read back; success prints `BL_CONFIG` and the CCFG's CRC32.
- Console input is lost during the write (a few milliseconds).

#### If you see CCFG WRITE FAILED

The command answers `ERROR` and prints what failed, the read-back value, and what to do:

- **Disable failed**: nothing was erased; the module resets normally. Check with
  `AT+BOOTLOADER_PIN?` and retry.
- **Enable failed, original restored**: nothing changed; retry.
- **Enable and restore both failed**: the CCFG is in an unknown state. **Don't reset the module.**
  Repeat `AT+BOOTLOADER_PIN=<0|1>,DEADBEE` until it answers `OK`; the firmware writes back its RAM
  copy of the original first. If the module resets anyway, it most likely starts in the ROM
  bootloader, where the Programmer (or JTAG) can reflash a complete image.

### Troubleshooting

| Symptom | Likely cause |
|---|---|
| The application answers after the reset; no bootloader ACK | SYNC wasn't high at reset (check RTS → SYNC wiring), the backdoor is off (`AT+BOOTLOADER_PIN?` answers `0`), or the firmware predates it. The Programmer prints `App console responds at <baud> ... SBL entry fails`. |
| Nothing answers | Reset or UART wiring (TX/RX swapped, no ground), no power, or DTR still asserted on a plain adapter. Behind the Programmer, wait for the green LED. |
| Garbage instead of `0x00 0xCC` | Baud rate too high for the ROM (~1.2 M) or the adapter. Retry at 115200. |
| New firmware doesn't run; module stays in the bootloader | SYNC is still high. Assert RTS and reset. |
| Module goes quiet after the port closes | SYNC went high on close (plain adapter or old Programmer). Assert RTS, or clear HUPCL (see [SYNC and sleep](#sync-and-sleep)). |
| Module is in the bootloader after a long sleep | The watchdog fired with SYNC high. Turn it off for long sleeps (`AT+WATCHDOG=0`). |
| An image flashed through the Programmer was replaced | The Programmer restored its baked image (see [above](#the-adsbee-1421-programmer-does-this-for-you)). |

For the Programmer's own messages, see its
[troubleshooting table](programmer/README.md#troubleshooting-yellow-blink--no-green).

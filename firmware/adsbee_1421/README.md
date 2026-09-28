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

## Reflashing over UART: the SYNC bootloader backdoor

The m1421 can be reflashed over its console UART with no debugger and no button presses. Every
release build enables this bootloader backdoor on purpose, and it is a supported way to update the
firmware.

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

- **RTS asserted → SYNC low; RTS deasserted → SYNC high.** Level, applied immediately.
- **DTR assert edge → 50 ms RESET_N pulse.** Edge-triggered, so a terminal that holds DTR
  asserted doesn't hold the module in reset. Deasserting DTR does nothing.
- The host's baud rate is applied to the Programmer's UART, so a host tool can run the
  bootloader protocol through the Programmer at whatever rate it likes.

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
s.rts = False            # deasserted: SYNC high, backdoor armed
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
| [Web console](../../software/adsbee_1421_console/README.md#firmware-upload-wiring) **Upload Firmware** | Yes. **Enter bootloader** drives RTS/DTR with Web Serial `setSignals()` (SYNC high, DTR deasserted → asserted 50 ms → deasserted, which suits both the Programmer and a plain adapter), checks the ROM answers, and after flashing drives SYNC low and restarts the module. A manual **Check bootloader** path remains for adapters without RTS/DTR wired. |
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
- Closing the port can put the module to sleep. On Linux, closing a serial port with HUPCL set
  (the default) deasserts RTS and DTR. RTS deasserted is SYNC high, so the module sleeps until
  something drives SYNC low again. Behind the ADSBee 1421 Programmer, the next open asserts DTR
  and resets the module, which is harmless. To keep the lines as they are after closing, clear HUPCL
  (`stty -F /dev/ttyACM0 -hupcl`) and leave RTS asserted.

### Prerequisites

- Firmware with the backdoor enabled. Every adsbee_1421 build from this repository has it,
  starting with the first release here, `adsbee_1421-0.3.7`. `AT+DEVICE_INFO?` reports the
  running version. Older firmware from before the move into this repository may not have it.
- A fallback for when SYNC entry doesn't work (firmware without the backdoor, or SYNC not
  reachable): send `AT+BOOT_UART_BOOTLOADER=1DEADBEE` on the console. The firmware erases flash
  sector 0 (the 2 KB vector-table sector at `0x00000000`) and resets. With no valid reset vector
  the ROM runs the serial bootloader on DIO_2/DIO_3 regardless of SYNC, and stays there on every
  reset until the module is reflashed. Only sector 0 is erased: the Settings (`0x000FC000`) and
  Device Info / OTA keys (`0x000FE000`) sectors are untouched. The command needs a working
  console, so it doesn't help a module whose console doesn't answer. Use JTAG for that
  ([`ti/README.md`](ti/README.md)).

### Troubleshooting

| Symptom | Likely cause |
|---|---|
| The application answers with AT text after the reset, and no bootloader ACK arrives | SYNC wasn't high when RESET_N was released: check the RTS → SYNC wiring and polarity, or the firmware predates the backdoor (check `AT+DEVICE_INFO?`, or use the `AT+BOOT_UART_BOOTLOADER` fallback). The ADSBee 1421 Programmer prints `App console responds at <baud> ... SBL entry fails` for this case. |
| Nothing at all answers | Reset or UART wiring (TX/RX swapped, no common ground), module unpowered, or (plain adapter) DTR still asserted so the module is held in reset. Behind the Programmer, check that its LED is green: it ignores RTS/DTR until pass-through. |
| Garbage bytes where `0x00 0xCC` should be | Baud rate above the ROM's ~1.2 M ceiling or beyond what the adapter can do. Retry at 115200. |
| The new firmware doesn't run after flashing; the module keeps showing up in the bootloader | SYNC is still high. Assert RTS (SYNC low) and reset again. |
| The module goes quiet after a script or terminal closes the port | HUPCL deasserted RTS, so SYNC is high and the module is asleep. Assert RTS again, or clear HUPCL (see [SYNC and sleep](#sync-and-sleep)). |
| The module turns up in the bootloader after a long sleep | The watchdog fired while SYNC was high. Disable it for long sleeps (`AT+WATCHDOG=0`). |
| An image flashed through the Programmer was replaced by a different version | The Programmer reflashed its baked image on a recheck (see [The ADSBee 1421 Programmer does this for you](#the-adsbee-1421-programmer-does-this-for-you)). |

The Programmer's own diagnostics for bootloader entry are listed in its
[troubleshooting table](programmer/README.md#troubleshooting-yellow-blink--no-green).

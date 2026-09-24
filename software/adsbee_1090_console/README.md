# ADSBee 1090 Web Console

A single-file, self-contained web console for the ADSBee 1090 that vendors the
device's ESP32-served web interface
(`firmware/adsbee_1090/esp/main/server/web/`) and connects to the device either
over the **Web Serial API** (the RP2040's USB CDC console) or over the
**network** (the ESP32's `/console`, `/metrics`, and `/aircraft` WebSockets —
the same endpoints the device-hosted page uses). Open
`adsbee_1090_console.html` directly in a browser (works from `file://`, no
server needed), pick **USB Serial** or **Network** in the tab bar, and click
**Connect**.

## Connecting

- **USB Serial** (Chrome/Edge only): opens a serial port picker; the page
  remembers a previously-granted port for one-click reconnects and
  auto-reconnects when the port reappears (USB replug, or the reboot after a
  firmware update). The console is a USB CDC virtual COM port, so unlike the
  1421 console there is no baud negotiation — any rate works.
- **Network** (any browser): enter the device's hostname or IP
  (`adsbee.local`, `192.168.x.x`) and the page opens the same three WebSockets
  the device-hosted page uses. The sockets keep retrying on their own (amber
  status dot) until the device is reachable. The host is remembered for the
  next session.

Serial and network reach the *same* RP2040 console — its output is mirrored to
USB stdout and the network console simultaneously — so the terminal behaves
identically in both modes.

## Feature matrix

| Feature | Network | USB Serial |
|---|---|---|
| Console terminal (AT commands, history, ANSI colors) | ✔ | ✔ |
| Map + aircraft table + detail sidebar | ✔ (`/aircraft` socket) | ✔ (`AT+PROTOCOL_OUT=CONSOLE,AIRCRAFT_JSON`, see below) |
| Settings tab (schema-driven GUI, `AT+SETTINGS?JSON` bulk read) | ✔ | ✔ |
| Feed editor (add/edit/remove feeds via `AT+FEED`) | ✔ | ✔ |
| Settings download / restore (`AT+SETTINGS?DUMP` replay) | ✔ | ✔ |
| Firmware upload (`.ota` via `AT+OTA`) | ✔ | ✔ |
| Receiver message-rate metrics (sparkline cards) | ✔ | ✘ — network-only, note shown |
| Feed throughput (msg/s per feed) | ✔ | ✘ — config-only cards, note shown |
| CPU load / temperatures / ESP32 & Sub-GHz status | ✔ | ✘ — note shown |
| GNSS fix, uptime, device info/versions | ✔ | ✔ (polled via AT) |

The receiver metrics, feed rates, and CPU/temperature blocks exist only on the
`/metrics` WebSocket — the 1090 firmware has no AT command that reports them —
so in serial mode each of those sections is replaced with a note explaining it
needs a network connection. In their place, serial mode quietly polls what the
AT interface *does* provide: `AT+GNSS_FIX?` (5 s — this also keeps the map's
receiver marker live), `AT+UPTIME?` (10 s), `AT+FEED?` (15 s, configuration
only), and `AT+DEVICE_INFO?` once per connect. The polls run through a hidden
AT queue, so the terminal stays clean.

## Map tab over serial

Entering the Map tab saves the current `AT+LOG_LEVEL` / `AT+PROTOCOL_OUT`
settings, then sets `AT+LOG_LEVEL=SILENT` and
`AT+PROTOCOL_OUT=CONSOLE,AIRCRAFT_JSON` and renders the newline-delimited
aircraft JSON stream. Switching back restores the saved settings. These
changes are RAM-only (`AT+SETTINGS=SAVE` is never issued), so a device power
cycle always returns to the persisted configuration — including if the page is
closed while on the Map tab (a best-effort restore is attempted on close, but
cannot be guaranteed). In network mode the Map tab simply uses the `/aircraft`
WebSocket, exactly like the device-hosted page.

## Firmware upload

Flashes a `.ota` image using the same `AT+OTA` flow as the device-hosted page
(GET_PARTITION → ERASE → WRITE in 4 KB chunks with CRC32 → VERIFY → BOOT).
Over serial this works because `AT+OTA=WRITE,<offset>,<len>,<crc>` prints
`READY` and then reads exactly `<len>` raw bytes from the console byte stream
(`ATReadConsole` in `comms_at.cc`) — there is no framing dependency, so the
page just writes the chunk to the port. During a serial upload the whole
serial stream is diverted to the updater (status pollers pause). `AT+OTA=BOOT`
reboots the RP2040 and re-enumerates the USB port; the page reconnects
automatically when it reappears.

## Vendoring conventions

This page is assembled by hand from the sources it mirrors — edit there, then
re-copy the marked block:

- **Settings engine**: `SettingsEngine` and its shared helpers are copied
  verbatim from `firmware/adsbee_1090/esp/main/server/web/settings.js`
  (between its `BEGIN/END SHARED SETTINGS ENGINE` markers) into this page's
  `BEGIN/END VENDORED ADSBee settings engine` markers. The
  `SETTINGS_SCHEMA_1090` table and `Settings1090Transport` are copied from the
  same file below the shared block.
- **Page CSS**: `style.css` is embedded in full, followed by a marked block of
  standalone-console additions (connection controls, browser banner,
  `.metrics-unavailable` notes).
- **UI/network classes**: `adsbee.js` is embedded near-verbatim with a short
  list of deltas documented at the top of the block (AT-based FeedEditor,
  channel-based ADSBeeAT, mode-aware uploader/settings-manager).
- **Serial transport**: `SerialManager`, `LineSink`, `AtQueue`, and the line
  router are ported from `software/adsbee_1421_console/adsbee_1421_console.html`
  (minus its baud sweep, which USB CDC makes unnecessary).
- **Leaflet 1.9.4** is vendored inline (BSD-2-Clause, license header kept)
  between `BEGIN/END VENDORED` markers, copied from the 1421 console.

## Notes

- USB serial requires Chrome or Edge (Web Serial API); other browsers show a
  banner and can still use network mode.
- Network mode uses plain `ws://`, so the page must not be served over HTTPS
  (mixed-content blocking) — opening it from `file://` is fine.
- Map tiles load from openstreetmap.org and need internet; everything else
  works offline.
- Opening the serial port asserts DTR deliberately: the RP2040's USB CDC
  console gates its output on DTR ("terminal connected").
- Never open the port at 233495790 baud (0xDEADBEE) — that is the pico-sdk
  magic rate that reboots the RP2040 into its UF2 bootloader.

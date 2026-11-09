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
- **Scan** (network mode): reads the QR code on the device's serial label with
  the camera (rear camera on phones) and connects to the device's default
  hostname. A serial typed or pasted into the host box, or typed by a USB
  barcode scanner (ending in Enter), is converted the same way. The label
  `NNNNNNNNNR-YYYYMMDD-VVXXXX` maps to `adsbee1090-` + `YYYMMDD` (the date
  without its first digit) + `VVXXXX` + `.local`, e.g.
  `000000000A-20250101-000001` → `adsbee1090-0250101000001.local`, as the
  firmware sets it (`DeviceInfo::GetDefaultSSID` in
  `firmware/common/settings/settings.hh`). If the camera can't be used
  (permission denied, no camera, page not on https or `file://`), **Photo**
  decodes a picture of the label instead. The QR is decoded with the browser's
  `BarcodeDetector` where it has one (Chrome on Android, macOS and ChromeOS),
  otherwise with the vendored jsQR.
  - `.local` names need mDNS on the computer or phone: built into macOS, iOS
    and Windows 10+, Avahi on Linux; Android support varies by version and
    browser. The page and the device must be on the same network.
  - A hostname changed with `AT+HOSTNAME` no longer matches the label; type it
    instead.

Serial and network reach the *same* RP2040 console — its output is mirrored to
USB stdout and the network console simultaneously — so the terminal behaves
identically in both modes.

## Feature matrix

| Feature | Network | USB Serial |
|---|---|---|
| Console terminal (AT commands, history, ANSI colors) | ✔ | ✔ |
| Map + aircraft table + detail sidebar | ✔ (`/aircraft` socket) | ✔ (`AT+PROTOCOL_OUT=CONSOLE,AIRCRAFT_JSON`, see below) |
| Weather on the map (FIS-B) | ✔ (`/uplink` socket) | ✔ (`AIRCRAFT_JSON_UAT_UPLINK`, see below) |
| Settings tab (schema-driven GUI, `AT+SETTINGS?JSON` bulk read) | ✔ | ✔ |
| Feed editor (add/edit/remove feeds via `AT+FEED`, whereplane.xyz map link) | ✔ | ✔ |
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

## Live Map tab over serial

Entering the Live Map tab saves the current `AT+LOG_LEVEL` / `AT+PROTOCOL_OUT`
settings, then sets `AT+LOG_LEVEL=SILENT` and
`AT+PROTOCOL_OUT=CONSOLE,AIRCRAFT_JSON` and renders the newline-delimited
aircraft JSON stream. Switching back restores the saved settings. These
changes are RAM-only (`AT+SETTINGS=SAVE` is never issued), so a device power
cycle always returns to the persisted configuration, even if the page is closed
on the Live Map tab. In network mode the Live Map tab uses the `/aircraft`
WebSocket, like the device-hosted page.

## Weather (FIS-B)

**Weather** on the Live Map draws FIS-B from UAT ground stations (US): radar, lightning, cloud
tops, icing, turbulence, airports colored by flight category with METAR, TAF and NOTAMs, pilot
reports, TFR, SIGMET and AIRMET outlines, and the ground stations. The buttons under the zoom
control show or hide each kind; hover one for its time and color key. Icing and turbulence show
the altitude picked in the box below their buttons. Network mode reads the
`/uplink` WebSocket while the map is shown. Serial mode switches the map stream to
`AIRCRAFT_JSON_UAT_UPLINK`; firmware without it keeps `AIRCRAFT_JSON` and the toggle shows ⚠.
Uplink lines never print in the terminal.

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

## Firmware update check

The nav bar shows the installed firmware version with a status dot (gray: not checked, offline
or no keys; spinner: checking or installing; green: up to date; amber: update available; red:
failed). On connect the page reads `AT+DEVICE_INFO?` (unique ID, part code, OTA keys), trades it
with the adsbee.aero firmware service (API v1) for a short-lived token and lists releases; the
listing is cached for 6 h and refreshed at that interval. Clicking the version opens a dropdown:
the stable release the service's rollout assigns to this receiver with **⇣ Update** (or
`✓ up to date`), **RC** for release candidates (with a warning), release notes, and **↻** to
check now. Update downloads the `.ota`, checks its size and SHA-256 and installs it with the same
upload path as Upload Firmware, then checks again when the receiver is back. Receivers without
valid OTA keys get a link to the GitHub releases for a manual upload. In network mode the
terminal is muted while the page reads `AT+DEVICE_INFO?`.
`localStorage['adsbee-fw-api']` overrides the service URL for testing. The code is
`firmware_release.js` from the device web UI, vendored verbatim
(`test/firmware_release_drift.test.js`).

## Vendoring conventions

This page is assembled by hand from the sources it mirrors — edit there, then
re-copy the marked block:

- **Settings engine**: `SettingsEngine` and its shared helpers are copied
  verbatim from `firmware/adsbee_1090/esp/main/server/web/settings.js`
  (between its `BEGIN/END SHARED SETTINGS ENGINE` markers) into this page's
  `BEGIN/END VENDORED ADSBee settings engine` markers. The
  `SETTINGS_SCHEMA_1090` table and `Settings1090Transport` are copied from the
  same file. `firmware/adsbee_1090/esp/main/server/web/test/settings_drift.test.js`
  fails when a copy differs.
- **Page CSS**: `style.css` is embedded in full, followed by a marked block of
  standalone-console additions (connection controls, browser banner,
  `.metrics-unavailable` notes).
- **UI/network classes**: `adsbee.js` is embedded near-verbatim with a short
  list of deltas documented at the top of the block (AT-based FeedEditor,
  channel-based ADSBeeAT, mode-aware uploader/settings-manager).
- **Terminal paste guard**: `isOtherTextEntry` (in `adsbee.js`) is also copied into
  `software/adsbee_1421_console/adsbee_1421_console.html`;
  `firmware/adsbee_1090/esp/main/server/web/test/terminal_paste.test.js` checks both copies
  and that the terminal's paste handler leaves other text boxes alone.
- **Serial transport**: `SerialManager`, `LineSink`, `AtQueue`, and the line
  router are ported from `software/adsbee_1421_console/adsbee_1421_console.html`
  (minus its baud sweep, which USB CDC makes unnecessary).
- **Firmware update check**: `firmware/adsbee_1090/esp/main/server/web/firmware_release.js` is copied
  verbatim between the `BEGIN/END VENDORED firmware_release.js` markers (here and in the 1421
  console); `test/firmware_release_drift.test.js` fails when a copy differs.
- **Weather**: `fisb.js` and `fisb_stations.js` are copied verbatim between their
  `BEGIN/END VENDORED` markers, and `AircraftWebSocket`, `UplinkWebSocket`, `webSocketRefused()` and
  `MapWeather` are copied from `adsbee.js` (here and, `MapWeather` only, in the 1421 console);
  `test/fisb_drift.test.js` fails when a copy differs.
- **Serial label → hostname**: `labelToHostname` between the `BEGIN/END LABEL_HOSTNAME` markers;
  `firmware/adsbee_1090/esp/main/server/web/test/label_hostname.test.js` tests it and checks it
  against the firmware's default-hostname test vector.
- **jsQR 1.4.0** (Apache-2.0, https://github.com/cozmo/jsQR) is vendored inline between
  `BEGIN/END VENDORED jsQR 1.4.0` markers: `dist/jsQR.js` from the npm package, minified with
  terser 5.
- **Leaflet 1.9.4** is vendored inline (BSD-2-Clause, license header kept)
  between `BEGIN/END VENDORED` markers, copied from the 1421 console.

## Notes

- USB serial requires Chrome or Edge (Web Serial API); other browsers show a
  banner and can still use network mode.
- Network mode uses plain `ws://`, so the page must not be served over HTTPS
  (mixed-content blocking) — opening it from `file://` is fine.
- Map tiles (OpenStreetMap) load through the adsbee.aero/tiles proxy and need
  internet; everything else works offline.
- Opening the serial port asserts DTR deliberately: the RP2040's USB CDC
  console gates its output on DTR ("terminal connected").
- Never open the port at 233495790 baud (0xDEADBEE) — that is the pico-sdk
  magic rate that reboots the RP2040 into its UF2 bootloader.

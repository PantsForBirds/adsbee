# ADSBee m1421 Web Console

A single-file, self-contained web console for the ADSBee m1421 (TI CC1314R10) that
mimics the ADSBee 1090 ESP32 web interface, but talks to the device over the
**Web Serial API** instead of WebSockets. Open `adsbee_1421_console.html` directly in
Chrome or Edge (works from `file://`, no server needed) and click **Connect**
(the page probes the firmware's baud whitelist to find the device — see below).

## Features

- **Baud auto-detection** — the device boots at its saved console baud
  (`AT+BAUD_RATE` + `AT+SETTINGS=SAVE`; factory default 1,000,000), so it may be
  at any of {115200, 230400, 460800, 921600, 1000000}. On connect the page
  sweeps that list — trying last session's rate first — and locks onto whatever
  rate answers. Each probe is `AT+BAUD_RATE?`, repeated for ~2.7 s per rate
  before moving on (opening the port asserts DTR, which resets the device, so it
  is usually still booting when the first probe goes out), and the rate shown is
  the one the device itself reports. Behind the programmer jig
  (`firmware/adsbee_1421/programmer/`) the host baud is virtual and the jig
  retunes the device to it after each host-driven reset; the page waits that
  out and, if the device's rate ever differs from the host port's, says so
  rather than guessing. After `AT+REBOOT`,
  `AT+SETTINGS=RESET`, or a firmware flash the page re-sweeps automatically, and
  a hand-typed `AT+BAUD_RATE=CONSOLE,<n>` is followed to the new rate instead of
  desyncing the link. Disconnecting leaves the device at its current rate.
- **Console tab** — interactive AT command terminal (line editing, history, ANSI
  colors), a Receiver Statistics panel, a Device Status card, and firmware upload.
  - Statistics update whenever an `RX_STATS=` response appears — type
    `AT+RX_STATS?` yourself or click **Refresh**. There is no background polling,
    so the console stream stays clean.
  - An **uptime** card sits alongside the statistics, fed by `AT+UPTIME?`. It is read
    quietly on connect and after anything that reboots the device (`AT+REBOOT`,
    `AT+SETTINGS=RESET`, a firmware flash), and refreshed by the same **Refresh**
    button — or by typing `AT+UPTIME?` yourself.
  - Device info (`AT+DEVICE_INFO?`) is queried once per connect.
  - All protocol output (CSBee, raw aircraft JSON, etc.) prints in the terminal.
    Aircraft JSON is hidden from the terminal only while the Map tab itself is
    driving the stream; JSON lines always feed the map's aircraft store either way.
- **Map tab** — Leaflet map + sortable aircraft table + detail sidebar. Entering the
  tab saves the current `AT+LOG_LEVEL` / `AT+PROTOCOL_OUT` settings, then sets
  `AT+LOG_LEVEL=SILENT` and `AT+PROTOCOL_OUT=CONSOLE,AIRCRAFT_JSON` and renders the
  newline-delimited aircraft JSON stream. Switching back to the Console tab restores
  the saved settings.
  - Aircraft on the ground (`on_ground`) render gray on both the map and the table
    (`grnd` in the altitude column). Icon rotation and the Hdg column use the ground
    track when reported, falling back to true/magnetic heading (surface traffic).
    Because surface reception is bursty (poor line-of-sight), ground aircraft are
    retained at their last known state for 10 minutes without an update (airborne:
    1 minute). The sidebar always shows a Status row (Airborne / On ground) and a
    "Last seen" age.
  - Each moving aircraft gets a **velocity vector**: a screen-space line along its
    direction whose length scales with speed (about one icon width at 250 kt,
    clamped) — zooming the map never changes its on-screen size.
  - These changes are RAM-only (`AT+SETTINGS=SAVE` is never issued), so a device
    power cycle always returns to the persisted configuration — including if the
    page is closed while on the Map tab (a best-effort restore is attempted on
    close, but cannot be guaranteed).
- **Settings tab** — a schema-driven form for every read/write settings AT command
  (receivers, gain/preamble/boost, sub-GHz mode, output protocol, MAVLink IDs,
  receiver position, console baud, log level, watchdog). Edits are applied with a
  single **Save** button, which sends only the changed `AT+<CMD>=` commands and then
  `AT+SETTINGS=SAVE`; **Refresh** re-reads everything from the device and discards
  edits. Reads happen in one round trip via `AT+SETTINGS?JSON` (a single-line JSON
  dump keyed by AT command), falling back automatically to per-command queries on
  firmware that predates it. All settings traffic runs through the hidden AT queue, so the terminal
  stays clean. A console baud change is followed automatically (the port is
  reopened at the new rate before `AT+SETTINGS=SAVE` is sent, so the new rate
  persists). Entering the tab from the Map tab first restores the persisted
  `PROTOCOL_OUT`/`LOG_LEVEL` so the form shows saved values, not the map stream's
  overrides.
  - The form renderer, dirty tracking, and save/refresh logic (`SettingsEngine`) are
    **vendored verbatim** from
    `firmware/adsbee_1090/esp/main/server/web/settings.js` (between that file's
    `BEGIN/END SHARED SETTINGS ENGINE` markers) into this page's
    `BEGIN/END VENDORED ADSBee settings engine` markers — edit it there and
    re-copy. The 1421-specific parts (the `SETTINGS_SCHEMA_1421` table and the
    AtQueue transport adapter) live just below the tab controller. To expose a new
    AT command in the GUI, add one entry to the schema table.
- **Upload Firmware** — flashes a `.hex` image (from
  `firmware/adsbee_1421/ti/build/<Config>/adsbee_1421-<ver>.hex`) via the
  CC13x4 factory ROM serial bootloader, a direct port of
  `software/adsbee_1421_flasher/adsbee_1421_flasher.py`. Runs at the detected
  baud (the ROM auto-bauds up to ~1.2 M): ~15 s for a ~600 KB image at 1 M
  instead of ~2 min at 115200. After the post-flash reboot the page re-detects
  the link automatically.

## Firmware upload wiring

The device must be in the ROM bootloader **before** flashing, and putting it there is
a manual step. The page does not drive the modem control lines to do it: that only
worked on an adapter wired like the programmer jig, it spent ~40 s failing on any
other adapter, and a failed attempt left SYNC high, which puts a running module to
sleep. The dialog prints the procedure instead:

1. Wire the adapter to the module: TX → SURX (pin 20, DIO_2), RX → SUTX (pin 21,
   DIO_3), and ground to ground.
2. Hold SYNC (pin 28, DIO_5) high at 3.3 V.
3. With SYNC still high, reset the module: pull RESET_N (pin 17) low briefly, or
   power-cycle it.
4. Release reset. SYNC can stay high, since the boot ROM samples it only at boot.
5. Press **Check bootloader**.

The firmware's CCFG must enable the bootloader backdoor, which
`firmware/adsbee_1421/ti/syscfg/adsbee_1421.syscfg` does (DIO_5, active high). The
baud rate does not need to match anything: the ROM locks onto whatever rate the page
sends its sync bytes at. If the SYNC pin is unreachable,
`AT+BOOT_UART_BOOTLOADER=1DEADBEE` also enters the bootloader, at the cost of erasing
the vector table, so the device stays there until it is reflashed.

**Check bootloader** is the gate on flashing. It syncs, pings, and reads the chip ID,
and only a device that answers all three unlocks the **Flash firmware** button. A
failure says which failure it is: if the application firmware answers instead, it says
so and names the baud; if nothing answers at all, it points at the wiring and power.
The check is repeated as a single ping immediately before the first erase, so a device
that was reset or unplugged between the check and the flash is caught while the image
is still intact. Disconnecting the port, picking a different file, or closing the
dialog all revoke a passed check.

After a successful flash the device is restarted with the bootloader's own `RESET`
command rather than a reset pulse. Return SYNC low first, or the reset lands back in
the bootloader instead of running the new image.

Nothing is erased by the check; erase begins only after the bootloader ACKs.
Only the 2 KB flash sectors covered by the image are erased (`SECTOR_ERASE`, never a
bank erase), so the module's Settings (`0x000FC000`) and Device Info / OTA keys
(`0x000FE000`) survive a reflash; an image that reaches into those sectors is refused
before anything is erased.
If a flash fails partway, the device stays in the ROM bootloader — reopen the dialog
and **Retry**, which re-runs the check before resuming. The board cannot be bricked (an interrupted flash leaves the
factory-default CCFG, which keeps the bootloader enabled).

## Notes

- Requires Chrome or Edge (Web Serial). Firefox/Safari show an unsupported banner.
- Map tiles load from openstreetmap.org and need internet; everything else works
  offline. Leaflet 1.9.4 is vendored inline (BSD-2-Clause, license header kept).
- Connecting may briefly reset the device on an adapter whose DTR is wired to
  RESET_N (the programmer jig): Chromium asserts DTR when opening a port. This is
  the only remaining reason the page touches the control lines — it parks them in
  the normal-run
  state immediately after opening.
- The page is assembled by hand — Leaflet's JS/CSS are pasted between
  `BEGIN/END VENDORED` markers with the sourcemap comment and `url(images/...)`
  rules stripped. To upgrade Leaflet, replace those two blocks.

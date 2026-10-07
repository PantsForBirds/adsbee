# ADSBee m1421 Web Console

A single-file, self-contained web console for the ADSBee m1421 (TI CC1314R10) that
mimics the ADSBee 1090 ESP32 web interface, but talks to the device over the
**Web Serial API** instead of WebSockets. Open `adsbee_1421_console.html` directly in
Chrome or Edge (works from `file://`), connect the module through the ADSBee 1421
Programmer (`firmware/adsbee_1421/programmer/`) and click **Connect**.

## Features

- **Console baud rate** — the ADSBee 1421 Programmer follows the module's rate (9600
  to 3,000,000; default 1,000,000), so the page needs no setting. After
  `AT+BAUD_RATE=CONSOLE,<n>`, wait for `OK` before the next command.
- **Console tab** — interactive AT command terminal (line editing, history, ANSI
  colors), a Receiver Statistics panel, a Device Status card, and firmware upload.
  - Statistics update whenever an `RX_STATS=` response appears — type
    `AT+RX_STATS?` yourself or click **Refresh**. There is no background polling,
    so the console stream stays clean.
  - An **uptime** card (`AT+UPTIME?`) is read on connect, after reboots and flashes,
    and on **Refresh**.
  - Device info (`AT+DEVICE_INFO?`) is queried once per connect.
  - All protocol output (CSBee, raw aircraft JSON, etc.) prints in the terminal.
    Aircraft JSON is hidden only while the Live Map tab drives the stream; it always
    feeds the map.
- **Live Map tab** — map, sortable aircraft table and detail sidebar. Entering the
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
    power cycle always returns to the persisted configuration, even if the page is
    closed on the Live Map tab.
- **Settings tab** — a schema-driven form for every read/write settings AT command
  (receivers, gain/preamble/boost, sub-GHz mode, output protocol, MAVLink IDs,
  receiver position, console baud (9600 to 3,000,000), log level, watchdog). Edits
  are applied with a
  single **Save** button, which sends only the changed `AT+<CMD>=` commands and then
  `AT+SETTINGS=SAVE`; **Refresh** re-reads everything from the device and discards
  edits. Settings are read in one `AT+SETTINGS?JSON` dump (retried if incomplete;
  older firmware is queried per command), out of the terminal's view.
  The **Bootloader Pin** row reads the live CCFG backdoor state from the same dump
  (read-only key `BOOTLOADER_PIN`; firmware whose dump lacks the key shows it as
  unknown). Changing it sends `AT+BOOTLOADER_PIN=<0|1>,DEADBEE` after you type
  the password, then re-reads the dump.
  A console baud change takes effect immediately; **Save** persists it.
  Entering the tab from the Live Map tab first restores the persisted
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
  `firmware/adsbee_1421/ti/build/<Config>/adsbee_1421-<ver>.hex`) through the
  CC13x4 ROM serial bootloader, which the page enters itself (see below). About
  15 s at 1 M baud,
  or ~2 min at 115200. After the post-flash reboot the page re-detects
  the link automatically.

## Firmware upload wiring

The module must be in the ROM bootloader before flashing (see
[Reflashing over UART: the SYNC bootloader backdoor](../../firmware/adsbee_1421/README.md#reflashing-over-uart-the-sync-bootloader-backdoor)).
Press **Enter bootloader** in the dialog. It works two ways:

- Automatically, with the ADSBee 1421 Programmer or an adapter wired RTS → SYNC and
  DTR → RESET_N (asserting a line drives its pin low). The page deasserts RTS (SYNC
  high) and pulses DTR to reset, then checks that the ROM answers. If it doesn't, the
  page drives SYNC low again so the module isn't left asleep, and reports whether the
  application answered (RTS/DTR not wired, or firmware older than 0.3.7) or nothing did.
- By hand, for adapters without RTS/DTR wired to the module:
  1. Wire the adapter to the module: TX → SURX (pin 20, DIO_2), RX → SUTX (pin 21,
     DIO_3), and ground to ground.
  2. Hold SYNC (pin 28, DIO_5) high at 3.3 V.
  3. With SYNC still high, reset the module: pull RESET_N (pin 17) low briefly, or
     power-cycle it.
  4. Release reset and keep SYNC high.
  5. Press **Enter bootloader**.

The firmware's CCFG must enable the bootloader backdoor, which
every release since `adsbee_1421-0.3.7` does, unless `AT+BOOTLOADER_PIN=0,DEADBEE` turned
it off; `AT+BOOTLOADER_PIN?` shows it and `AT+BOOTLOADER_PIN=1,DEADBEE` turns it back on.
Any baud rate works: the ROM locks onto the page's rate.

**Connecting resets the module** into its application (the OS asserts DTR on open),
even if it was in the bootloader, which is why the page enters the bootloader after
connecting. On Linux and macOS, closing the port deasserts both lines: the ADSBee 1421
Programmer keeps the module awake, but on a plain adapter (or Programmer 0.3.11-rc2 or
earlier) the module sleeps until SYNC is driven low.

The check that ends **Enter bootloader** gates flashing. It syncs, pings, and reads the chip ID,
and only a device that answers all three unlocks the **Flash firmware** button. A
failure says which failure it is: if the application firmware answers instead, it says
so and names the baud; if nothing answers at all, it points at the wiring and power.
The check is repeated as a single ping immediately before the first erase, so a device
that was reset or unplugged between the check and the flash is caught while the image
is still intact. Disconnecting the port, picking a different file, or closing the
dialog all revoke a passed check.

After a flash, a settings erase, or closing the dialog after a passed check, the page
drives SYNC low and resets the device into its application. **If you are holding SYNC
high by hand, release it first**, or the reset lands back in the bootloader.

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
- Connecting resets the device; see [Firmware upload wiring](#firmware-upload-wiring).
- The page is assembled by hand — Leaflet's JS/CSS are pasted between
  `BEGIN/END VENDORED` markers with the sourcemap comment and `url(images/...)`
  rules stripped. To upgrade Leaflet, replace those two blocks.

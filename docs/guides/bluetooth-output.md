# Bluetooth Output

An ADSBee 1090 can stream traffic to one app over Bluetooth Low Energy, as GDL90 (the same heartbeat, ownship, traffic
and FIS-B it sends to WiFi access point clients on UDP port 4000) or as FLARM NMEA for gliding apps. The tablet stays on
its own network for internet. Off by default.

## Turn on

Send on the [CLI](../getting-started/quick-start.md#connect-to-cli):

```
AT+BLE_OUTPUT=GDL90
AT+SETTINGS=SAVE
```

Or use **Settings → Reporting → Bluetooth Output** on the device's webpage.

| Command | Does |
|---|---|
| `AT+BLE_OUTPUT=<mode>` | `OFF`, `GDL90` or `FLARM_NMEA` |
| `AT+BLE_OUTPUT?` | Shows the mode and its state: `OFF`, `ADVERTISING` (waiting for an app), `CONNECTED`, `STREAMING`, `BLOCKED_BY_WIFI`, `BLOCKED_BY_REMOTE_ID`, `BLOCKED_BY_RAM` or `NOT_IN_BUILD` |

The ADSBee advertises as `ADSBee 1090 XXXX` (the end of its Bluetooth MAC address). In the app, pick it from the
Bluetooth or external device list. Apps that pair first (AvareX) pair with no PIN.

## Apps

GDL90 over Bluetooth works with apps that read GDL90 from a generic Bluetooth LE serial device:

| App | Platform | Works |
|---|---|---|
| AvareX | Android | Yes. Pair the ADSBee in AvareX's IO screen, then connect. |
| XCSoar 7.45 and newer | Android | Yes, with the GDL90 driver on a Bluetooth LE port. |
| ForeFlight, Garmin Pilot, FlyQ, WingX, iFly, OzRunways, AvPlan, EasyVFR, SkyDemon | iOS, Android | No. Use WiFi (UDP port 4000). |

SkyDemon, SeeYou Navigator and Enroute read Bluetooth traffic as FLARM NMEA: use `FLARM_NMEA`.

## FLARM NMEA

`AT+BLE_OUTPUT=FLARM_NMEA` sends FLARM data port sentences (FLARM FTD-012, data port version 9) once per second:

| Sentence | Sent | Content |
|---|---|---|
| `GPRMC`, `GPGGA` | Ownship known and UTC time known (GNSS, or NTP) | Ownship position, speed, track; GNSS altitude (mean sea level) |
| `PGRMZ` | Ownship from its own ADS-B | Ownship pressure altitude |
| `PFLAU` | Always | Heartbeat: number of aircraft received, GPS state. `TX` is 0 and there are no alarms |
| `PFLAA` | Ownship known | One per aircraft, nearest first: position relative to ownship, track, speed, climb, aircraft type, source (ADS-B, ADS-R, TIS-B), signal strength |

FLARM positions are relative to ownship, so traffic needs ownship from `AT+RX_POSITION` source `GNSS` (a GNSS fix) or
`ICAO` (the tracked aircraft with your own ICAO address). A fixed position or the lowest aircraft is not ownship.
Without ownship only `PFLAU` goes out, with GPS 0; apps show the device as connected without GPS.

Vertical separation uses pressure altitude against pressure altitude when ownship has one (`ICAO`), else GNSS
altitude. ADS-B GNSS altitude is above the WGS84 ellipsoid and a GNSS fix is above mean sea level, so with a GNSS
ownship it is off by the local geoid separation (typically under 50 m). Aircraft with no altitude, and Remote ID
drones with an `ICAO` ownship, are counted but not sent.

ADSBee receives only: it doesn't transmit FLARM and computes no collision alarms. Every alarm level is 0 and `PFLAU`
reports `TX` 0, which some apps show as "No FLARM transmission".

| App | Setup | Checked |
|---|---|---|
| XCSoar (Android) | Device with the FLARM driver on a Bluetooth LE port | XCSoar's NMEA parser reads the stream; not tried in the app |
| Enroute (Android, iOS) | Bluetooth LE traffic receiver | Enroute's FLARM parser reads the stream; not tried in the app |
| SkyDemon, SeeYou Navigator, LK8000, XCTrack | | Not tested |

## Profiles

Both generic serial profiles are offered; the app subscribes to one.

| Service | Data characteristic |
|---|---|
| Nordic UART Service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | Notify `6E400003-…`; writes to `6E400002-…` are ignored |
| HM-10 / TI CC254x `FFE0` | `FFE1`: notify; writes are ignored |

Data is the GDL90 byte stream, framed as on UDP (0x7E flags, byte stuffing, CRC), or NMEA sentences ending in CR LF. A
notification holds whole messages or sentences unless one is longer than the notification.

## Limits

* One app at a time.
* ADSBees without PSRAM (`AT+DEVICE_INFO?` shows `ESP32 PSRAM: NONE`) need the WiFi access point and station off
  (`BLOCKED_BY_WIFI`). Ethernet can stay on.
* Without PSRAM, Remote ID over Bluetooth takes priority: the output waits (`BLOCKED_BY_REMOTE_ID`) while Remote ID
  receive or transmit uses Bluetooth. With PSRAM both run.
* Throughput: a 247-Byte link (244-Byte notifications) carried 200 aircraft at 1 Hz (about 6 kB/s). With more data
  than the link takes, the next second's report replaces what is still queued, so the heartbeat always goes out first.
  FIS-B uplinks are dropped before traffic. A link at the default 23 Bytes (20-Byte notifications) carries less.

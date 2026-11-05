# Bluetooth Output

An ADSBee 1090 can stream GDL90 to one app over Bluetooth Low Energy: the same heartbeat, ownship, traffic and FIS-B
it sends to WiFi access point clients on UDP port 4000. The tablet stays on its own network for internet. Off by
default.

## Turn on

Send on the [CLI](../getting-started/quick-start.md#connect-to-cli):

```
AT+BLE_OUTPUT=GDL90
AT+SETTINGS=SAVE
```

Or use **Settings → Reporting → Bluetooth Output** on the device's webpage.

| Command | Does |
|---|---|
| `AT+BLE_OUTPUT=<mode>` | `OFF` or `GDL90` |
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

SkyDemon, SeeYou Navigator and Enroute read Bluetooth traffic as FLARM NMEA, not GDL90.

## Profiles

Both generic serial profiles are offered; the app subscribes to one.

| Service | Data characteristic |
|---|---|
| Nordic UART Service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | Notify `6E400003-…`; writes to `6E400002-…` are ignored |
| HM-10 / TI CC254x `FFE0` | `FFE1`: notify; writes are ignored |

Data is the GDL90 byte stream, framed as on UDP (0x7E flags, byte stuffing, CRC). A notification holds whole messages
unless one message is longer than the notification.

## Limits

* One app at a time.
* ADSBees without PSRAM (`AT+DEVICE_INFO?` shows `ESP32 PSRAM: NONE`) need the WiFi access point and station off
  (`BLOCKED_BY_WIFI`). Ethernet can stay on.
* Without PSRAM, Remote ID over Bluetooth takes priority: the output waits (`BLOCKED_BY_REMOTE_ID`) while Remote ID
  receive or transmit uses Bluetooth. With PSRAM both run.
* Throughput: notifications carry 244 Bytes. With PSRAM, about 180 of the 200 aircraft the ESP32 tracks go out
  every second (about 6 kB/s). Without PSRAM, about 95 aircraft go out per second; with more, the next report starts where the
  last one stopped, so each aircraft still goes out every few seconds. The heartbeat always goes out first, and FIS-B
  uplinks are dropped before traffic.

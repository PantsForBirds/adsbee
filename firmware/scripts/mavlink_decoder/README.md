# MAVLink Decoder Script

A terminal utility that connects to an ADSBee device over serial and displays a live table of ADS-B aircraft tracked via the MAVLink `ADSB_VEHICLE` message.

## Requirements

- Python 3.10+
- [Poetry](https://python-poetry.org/)

## Setup

```sh
poetry install
```

## Usage

```sh
poetry run python mavlink_decoder.py <serial_port> <baud_rate>
```

**Example:**

```sh
poetry run python mavlink_decoder.py /dev/ttyUSB0 57600
```

The screen refreshes each time a new `ADSB_VEHICLE` MAVLink packet is received and prints one row per tracked aircraft:

```
ICAO Addr |Latitude  |Longitude |Alt Type  |Alt (m)   |Hdg (deg) |Hvel (m/s)|Vvel (m/s)|Callsign  |Type      |TSLC (sec)|Flags     |Squawk
----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------
    acf5aa|  +37.0650| -121.5878|         1|    205.00|  336.0300|      35.0|      -1.0|N934MT    |         1|        40| 110111111|    177777
    a0f4b5|  +37.5646| -121.5084|         1|   3605.00|  262.0100|     140.0|      -2.0|DAL679    |         5|         4| 110111111|    177777
```

## Column Reference

| Column | Description |
|--------|-------------|
| ICAO Addr | 24-bit ICAO aircraft address (hex) |
| Latitude | Degrees, WGS84 |
| Longitude | Degrees, WGS84 |
| Alt Type | `0` = pressure altitude, `1` = geometric (GNSS) altitude |
| Alt (m) | Altitude in meters |
| Hdg (deg) | True heading in degrees |
| Hvel (m/s) | Horizontal velocity in m/s |
| Vvel (m/s) | Vertical velocity in m/s (positive = climbing) |
| Callsign | Flight callsign or `?` if unknown |
| Type | ADS-B emitter category (MAVLink `ADSB_EMITTER_TYPE` enum) |
| TSLC (sec) | Time since last communication, in seconds |
| Flags | MAVLink `ADSB_FLAGS` bitmask (binary) |
| Squawk | Transponder squawk code (e.g. 7700) |

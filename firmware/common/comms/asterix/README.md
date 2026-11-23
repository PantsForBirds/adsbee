# ASTERIX output

ADSBee 1090 and ADSBee 1421 can report traffic as EUROCONTROL ASTERIX, the surveillance data format used by air
navigation service providers:

- **CAT021** (ADS-B target reports), [Edition 2.6](https://www.eurocontrol.int/publication/cat021-eurocontrol-specification-surveillance-data-exchange-asterix-part-12-category-21).
  The records are also valid Edition 2.7: 2.7 only adds I021/090 extensions that ADSBee doesn't send.
- **CAT023** (ground station and service status), [Edition 1.3](https://www.eurocontrol.int/publication/cat023-eurocontrol-specification-surveillance-data-exchange-part-16-category-23).

## Turning it on

- Serial: `AT+PROTOCOL_OUT=<interface>,ASTERIX` (ADSBee 1090 `CONSOLE` or `COMMS_UART`, ADSBee 1421 `CONSOLE`).
- Network feed (ADSBee 1090): `AT+FEED=<index>,<uri>,<port>,1,ASTERIX`, or pick ASTERIX in the web UI feed editor.
- Data source: `AT+ASTERIX_SAC_SIC=<sac>,<sic>` sets the System Area Code and System Identification Code (0-255
  each, default 0,0) carried by every record. Use the codes your ASTERIX consumer assigned to this receiver; SACs
  are allocated per country by EUROCONTROL. Each ADSBee feeding the same consumer needs its own SAC/SIC pair. Save
  with `AT+SETTINGS=SAVE`.
- Time server (ADSBee 1090): `AT+NTP_SERVER=<host>` (default `pool.ntp.org`), see [Time](#time).

## What is sent

Once per second:

1. One CAT023 data block: a ground station status report, then a service status report for each service, normal
   or disabled with its receiver:
   - Service 1: 1090 MHz ADS-B (type 2, 1090 ES).
   - Service 2: UAT ADS-B (type 3, UAT).
   - Service 3: 1090 MHz TIS-B and ADS-R rebroadcasts, DF18 CF=2, 5 and 6 (type 5, TIS-B 1090 ES).
   - Service 4: UAT TIS-B and ADS-R rebroadcasts, address qualifiers 2, 3 and 6 (type 6, TIS-B UAT).
2. CAT021 data blocks with one record per aircraft. Blocks are at most 1024 bytes, so each fits in one UDP
   datagram.

The stream is data blocks back to back with no extra framing. Each block starts with its category and length.

CAT021 records carry, when known: I021/010 data source, 040 target report descriptor, 161 track number, 015
service ID, 131 position, 080 address, 073 and 075 times of reception of position and velocity, 140 geometric
height, 090 quality indicators (NACv, NIC, NICbaro, SIL, NACp, SDA, GVA, PIC), 210 ADS-B version and link
technology, 070 Mode 3/A, 145 flight level, 152 magnetic heading, 200 emergency and alert status, 155 and 157
vertical rates, 160 ground speed and track, 150 and 151 airspeed, 077 report time, 170 callsign, 020 emitter
category, 008 TCAS RA and single antenna, 132 signal strength.

How each target is heard:

- I021/015 service identification is the service of the target's position: 1 or 2 for ADS-B, 3 or 4 for TIS-B and
  ADS-R. CAT021 has no TIS-B or ADS-R flag, so use I021/015 to tell rebroadcasts apart.
- I021/040 ATP (address type) is 0 (ICAO address) for DF17, DF18 CF=0 and ICAO-addressed TIS-B and ADS-R, and 3
  (anonymous) for DF18 CF=1, TIS-B track files and other non-ICAO addresses. UAT surface vehicles are 2, and fixed
  beacons set RAB (report from a field monitor).
- I021/210 (ADS-B version) is left out for 1090 MHz TIS-B targets, whose messages carry none.
- An aircraft heard both directly and through a rebroadcast is reported once, from the direct source while its data
  is fresh.

Not reported:

- Aircraft without a position.
- Mode S aircraft that don't send ADS-B, and Remote ID drones. CAT021 is ADS-B only. Monoradar categories
  (CAT048, CAT010) need a measured range and bearing and multilateration (CAT020) needs several receivers.
- Ground speed and heading of aircraft on the ground (CAT021 puts these in its Reserved Expansion Field).

Track numbers stay with an aircraft while it is reported and are reused 60 s after it was last reported. The
first ground station status report after boot sets RN (track numbering restarted).

## Time

ASTERIX times are seconds since midnight UTC. ADSBee takes UTC from, in order:

1. A GNSS receiver with a valid time (`AT+GNSS`), with the accuracy of its NMEA time messages.
2. On the ADSBee 1090, SNTP (Simple Network Time Protocol) on the ESP32, polled once an hour from the
   `AT+NTP_SERVER` host. It runs only while a serial interface (console, comms UART, GNSS UART) or an active feed uses ASTERIX, and needs
   a WiFi station or Ethernet connection. The time counts for 3 h after the last answer from the server.
   `"sntp"` in the device status of the metrics stream (`/metrics`) shows whether it is synced.

Both count as a valid time source, so CAT023 clears TSV and NOGO. CAT021 and CAT023 have no item that tells
GNSS time from network time.

Without either:

- Times count from boot, modulo 24 h.
- CAT023 reports TSV (time source invalid) and NOGO (data must not be used operationally).
- Every CAT021 record sets the I021/040 NOGO bit.

The ADSBee 1421 has no GNSS input or network, so its output always carries NOGO.

## Decoding

- Wireshark and tshark decode ASTERIX on UDP and TCP port 8600. For another port use
  `tshark -d tcp.port==<port>,asterix`.
- Python: [asterix_decoder](https://pypi.org/project/asterix_decoder/) or
  [libasterix](https://pypi.org/project/libasterix/).

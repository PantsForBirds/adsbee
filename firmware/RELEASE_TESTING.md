# Release testing

Run this before tagging any release candidate or stable release of `adsbee_1090` or `adsbee_1421`
(see [Releases](README.md#releases)). Every item compares the candidate with the **previous release on
the same unit, same conditions**. Report each item as pass or fail with numbers and how they were
measured; the release notes' Testing section comes from this report.

- **Images:** the CI artifacts (`firmware`) of the commit that will be tagged. Record their SHA-256s.
  If `main` moves during the test, keep testing that commit.
- **Regression** means worse than the previous release. Alternate old/new (or old/new/old) so traffic
  or temperature changes don't fool you. Don't call a regression from empty sky.
- **Real bugs:** file them with a repro and keep testing the rest.

## Bench rules

- **Locks:** `flock /tmp/adsbee-hil-locks/<serial>.lock` (or `adsbee-hil lock`) for every unit and Bee
  Waggler you use, held only while you use it. Other people share the benches.
- **Backup first:** 1090: full 16 MB flash read over PICOBOOT; 1421: `AT+SETTINGS?DUMP` and the
  installed release. **Restore at the end:** bench units byte-exact, live units to the release and
  settings they had (`AT+CONFIG_HASH?` as found). Never `AT+SETTINGS=SAVE` on a live unit except to
  restore it.
- **Feeds off** on any unit that receives Bee Wiggler, Pluto Wiggler or Bee Waggler RF, for the
  whole test: `AT+FEED_ENABLE=0` + `AT+SETTINGS=SAVE`, checked after every flash (defaults turn feeds
  back on). Restore them afterwards. Never change feed settings on live units.
- **Bench ICAO addresses only: `0xBEE000`-`0xBEE0FF`**, a block ICAO Annex 10 allocates to no state.
  Check a public aggregator for the block before and after RF (command in
  [`ci/hil/README.md`](../ci/hil/README.md)). Max -10 dBm into a 1090, -20 dBm into a 1421. Never
  transmit from a 1421.
- **Coprocessor reflash:** the RP2040 reflashes the ESP32 and CC1312 only when the version number
  changes. When two images share a version, send `AT+ESP32_FLASH` after the UF2.
- **Remote ID:** bench UAS IDs only (`ADSBEEBENCHA`/`B`), never with a position
  (`AT+RX_POSITION` source NONE, LOWEST or ICAO; status shows `TX_NO_POSITION`).

## ADSBee 1090

`adsbee-hil` is the [bench toolkit](../ci/hil/README.md). Bench = 1090U cabled to a Bee Waggler
(FPGA-timed frames: `AT+SCHED=ADD,…` lines played once with `AT+TX=1,FILE,sched`, or a packet file uploaded
with `AT+FILE` and a header `period` when looped; levels in dBm; a Bee Waggler still on firmware 1.x uses
`AT+PW_SCHED`). A looped `sched`, or a file without a period, wraps back to back and loses a frame per loop.
Live = a 1090 on a roof antenna with its usual feeds. Mode S (1090 MHz) on port 1, UAT (978 MHz) on port 2;
items that need both at once overlapping opt in with `AT+SIMULTANEOUS=MODE_S,UAT,ACCEPT_COSTS` (max level
in `AT+SIMULTANEOUS?`, about -18 dBm) and `AT+SIMULTANEOUS=OFF` after. In that plan a 1090U fed above about
-45 dBm misses frames: raise its `AT+TL_OFFSET` or attenuate more.

| # | Item | How | Pass |
|---|---|---|---|
| 1 | UF2 install over USB, old to new | `adsbee-hil flash -d <unit> adsbee_1090-<ver>.uf2` | All 3 chips report the new version (`AT+DEVICE_INFO?`); `AT+CONFIG_HASH?` matches `<ver>.hashes.txt`; `AT+OTA?` verifies partition 0; no CC1312 or ESP32 boot errors |
| 2 | OTA install in the web UI, old to new | Upload Firmware with the `.ota` (like a user), or `adsbee-hil flash ... .ota --host H` | Same as 1 for the OTA partition; page back online |
| 3 | Settings migration | `AT+SETTINGS?DUMP` before and after 1 and 2, diff | Every old line kept; new keys at their documented defaults; `FEED_ENABLE` unchanged |
| 4 | Warm reboot, OTA boot | `AT+REBOOT`, console log | No new boot errors |
| 5 | Downgrade to the previous release | UF2 of the previous release | Works; settings reset to defaults if the settings version changed (say so in Upgrade notes) |
| 6 | DF17 decode vs level | Bee Waggler, 4 bench ICAOs, 1 ms apart, -94 to -10 dBm, 512 frames per level | 50 % point within 0.5 dB of the previous release; ≥ 99.5 % from -80 to -10 dBm |
| 7 | RSSI vs level | Same run | Same curve (±1 dB) |
| 8 | Mixed DF | DF0/4/5/11/16/17/20/21, 1024 frames each at -76, -60, -40, -20 dBm | ≥ 99.8 % each, no worse than previous |
| 9 | Max rate | 256-frame DF17 bursts over 64 bench ICAOs at 7,500, 7,750 and 8,000/s; DF4 at 12,000 and 13,000/s; count from `AIRCRAFT_JSON` `messages` growth (RAW over USB caps near 3,600 lines/s) | 7,500/s ≥ 99.9 %; **7,750/s within 0.5 % of previous** (caught #340: 96.9 vs 99.6 %) |
| 10 | Frame pairs | Two frames 80-90 µs (56-bit follower) and **127-130 µs** (112-bit follower) apart, 256 pairs per spacing, -60 dBm | Second frame decoded as often as on the previous release (±0.5 %) |
| 11 | MLAT timestamp jitter | Robust RMS of timestamp minus scheduled time, levels ≥ -80 dBm | Within 2 ns of previous (about 15 ns) |
| 12 | UAT | ADS-B long/short and uplinks vs level; 100 ADS-B at 100-1000/s; 20 uplinks 30-6 ms apart | 50 % points within 0.5 dB; all bursts decoded |
| 13 | New features | Each PR's own test notes; datasheet rows for new AT commands and outputs | Behavior matches the datasheet |
| 14 | **Remote ID** | [Remote ID](#remote-id-adsbee-1090) | Every case passes |
| 15 | Live sky, bracketed | Live unit old / new / old, ≥ 1 h of new at a busy hour: Mode S frames per aircraft-second, aircraft per minute vs a public reference, RSSI percentiles, UAT, resets, ESP32 heap, console errors, feeds sending as found | Per-aircraft numbers within the old brackets; no resets |
| 16 | Web UI | Aircraft table, Live Map (tiles, 3 viewers, no "stalled for N ms" drops), Settings, console, version chip | Works, no page errors |
| 17 | Soak | New image ≥ 2 h on one unit (overnight is fine) | No reboot of any chip; heap flat |
| 18 | Full aircraft dictionary | Bee Waggler, -60 dBm: DF17 from all 256 bench ICAOs plus UAT long ADS-B from 144 of them (separate aircraft), each refreshed within 15 s; then 256 + 256; DF17 bursts at 2,000-7,000/s over 64 of them; count `AIRCRAFT_JSON` entries by hex and type | RP2040 holds 400, then drops the oldest; bursts decoded as on the previous release; at most one ESP32 "dictionary full" line per minute; no resets or queue overflows; `RP2040 RAM:` line of `AT+DEVICE_INFO?` recorded, free ≥ 8 kB |

### Remote ID (ADSBee 1090)

Required for every 1090 RC and stable release. Two 1090Us within Bluetooth range of each other, **both
on the candidate**; no host Bluetooth adapter is needed. One command per case
(`adsbee-hil remote-id`, [details](../ci/hil/README.md#remote-id-release-test)), **15 min per case**.
Pass = **no empty minute** on the receiving side and **no ESP32 reboot** on either unit. Put a unit with
a reachable web server (`host` in the bench file) on the receiving side: it counts frames from
`/metrics`; a unit reached only over USB is judged by its RSSI updates.

| # | Case | Command |
|---|---|---|
| R1 | RX + TX on both units | `adsbee-hil remote-id rxtx -d A -d B` |
| R2 | TX-only BT4, heard by the other unit | `adsbee-hil remote-id tx-bt4 -d A -d B` |
| R3 | TX-only BT5 Long Range | `adsbee-hil remote-id tx-bt5lr -d A -d B` |
| R4 | TX-only BT4 + BT5 Long Range (wedged the controller in rc9 testing) | `adsbee-hil remote-id tx-both -d A -d B` |
| R5 | RX-only unit hearing a TX unit | `adsbee-hil remote-id rx-only -d A -d B` |
| R6 | Ethernet / no-PSRAM guards, on every unit | `adsbee-hil remote-id guards -d A` |
| R7 | `AT+REMOTE_ID=` applies without SAVE | R1-R5 never save; each prints the seconds until the status words appeared |
| R8 | Heap with Remote ID on vs previous release | ESP32 heap min/median from R1 and R5 on the candidate vs the same cases on the previous release (or its RX-only numbers) |
| R9 | Watchdog resets | Scan and advertising watchdog resets (#377, #381) and host resets per minute: printed by every case; report them |
| R10 | 1090/978 decode not hurt | `adsbee-hil remote-id off -d A -d B`, then `rxtx`, then `off` on the live unit: Mode S and UAT frames/s per aircraft within the off brackets; bench items 6 and 9 with RX + TX on |

## ADSBee 1421

Bench = m1421 on its ADSBee 1421 Programmer cabled to a Bee Waggler; live = an m1421 on a roof antenna.

| # | Item | How | Pass |
|---|---|---|---|
| 1 | Install old to new | `adsbee-hil flash -d <programmer> adsbee_1421_programmer-fw<ver>.uf2` (magic baud) | Module reflashed, CRC32 verified; a second copy leaves it alone ("Firmware up to date") |
| 2 | Settings migration | `AT+SETTINGS?DUMP` before and after, diff | Every old value kept (including non-default ones); new keys at defaults |
| 3 | Self test, hashes | `AT+TEST`, `AT+CONFIG_HASH?`, `AT+RX_STATS?` | All tests pass; hash matches `<ver>.hashes.txt` |
| 4 | Downgrade | Previous release's Programmer `.uf2` | Works; settings reset if the settings version changed (Upgrade notes) |
| 5 | Mode S decode vs level | DF17 and mixed DF, -90 to -20 dBm | ≥ 99 % from -86 to -30 dBm, no worse than previous |
| 6 | Rate | 100 to 4,000 frames/s, BEAST | 100 % decoded; MLAT stamped share no lower than previous |
| 7 | MLAT | Pairing and jitter at 4,000/s | 0 wrong pairings; jitter RMS within 5 ns of previous |
| 8 | UAT | ADS-B and uplinks vs level | 50 % points within 0.5 dB of previous |
| 9 | SPI under load | Interleaved AT queries during max rate | Every query answered, counters monotonic |
| 10 | Sleep current | PPK2, if free | Within 10 % of previous (about 0.11 mA) |
| 11 | New features | Each PR's test notes; datasheet rows | Behavior matches the datasheet |
| 12 | Live sky, bracketed | Old / new / old, ≥ 1 h of new at a busy hour: DF17 per aircraft-second, aircraft count, MLAT stamped share and timestamp sanity, uptime, console errors; web console Settings tab | Within the old brackets; 0 backward timestamps; no resets |
| 13 | Soak | New image ≥ 2 h under load or live | No reboot or stall |

# ADSBee swarm API v1

Two halves. A bee posts a report every period with its account's **enroll token**. A person or script
reads and manages the account with a session or an **API token**. `adsbee_swarm/openapi.yaml` is the
full spec. Errors are `{"error": code, "detail"?: text}`.

## Device report

Set on the bee: `AT+SWARM=<enabled 0|1>,<server base URL>,<enroll token>` (`AT+SWARM?` masks the token).
The default server is `https://adsbee.aero`.

`POST {server}/api/swarm/v1/report`, `Authorization: Bearer <enroll token>`:

```json
{
  "v": 1,
  "product": "adsbee_1090",
  "unique_id": "E6614C311B5B6F2A",
  "mac_address": "DC:54:75:D2:6C:24",
  "ota_keys": ["..", ".."],
  "versions": {"rp2040": "0.9.1", "esp32": "0.9.1"},
  "auto_update": true,
  "uptime_s": 12345,
  "seq": 17,
  "period": {"start_uptime_s": 8745, "len_s": 3600},
  "rx_position": {"lat": 37.1, "lon": -122.1, "alt_m": 30, "source": "fixed"},
  "stats": {"uat_adsb": 0, "uat_uplink": 0, "aircraft_max": 0, "aircraft_unique": 0},
  "listen_s": {"mode_s": 3600, "uat": 3600, "remote_id": 3600},
  "frames": {"mode_s": 0, "remote_id": 0},
  "ranges": {
    "mode_s": {"sectors": 36, "bands": 12, "unit_m": 2000, "changed": "<base64, 54 bytes>", "max": "<base64>"},
    "uat": {"sectors": 36, "bands": 12, "unit_m": 2000, "changed": "..", "max": ".."},
    "remote_id": {"sectors": 36, "bands": 12, "unit_m": 50, "changed": "..", "max": ".."}
  }
}
```

| Field | Rule |
|---|---|
| `ota_keys` | Sent only to the default `adsbee.aero` host. Checked by the server's device verifier, if it has one. |
| `unique_id` | RP2040 flash UID, 16 hex digits. |
| `seq` | +1 per report, reset at boot. |
| `rx_position` | Required for `ranges`. Without it, send `null` and no `ranges`; the server keeps the stats and drops any range. |
| `ranges` | One map per protocol. A protocol with no bins in the period is omitted; `{}` is valid. Send only protocols the server listed in its last reply (below). |
| `listen_s` | Optional. Seconds each protocol's receiver was on in the period (integers 0..604800; 0 or missing: not listening). More than `len_s` is stored as `len_s`. A protocol listened to with no map is stored as "listened, heard nothing". Without the key, listen time is unknown (older firmware). |
| `frames` | Optional. Valid frames per protocol that has no `stats` keys (integers 0..2^40): `mode_s` (every valid Mode S frame, 56- and 112-bit) and `remote_id`; ignored for `uat`. |
| `stats` | Missing keys count as 0. Newer firmware sends a protocol's keys only when it listened to it: `uat` → `uat_adsb`, `uat_uplink`; `mode_s` and `remote_id` have none (they go in `frames`). |
| `modes_valid`, `es_valid` | Older firmware's Mode S counts in `stats` (56-bit, 112-bit), still accepted and stored as sent. Mode S is one count: `frames.mode_s` or, from older firmware, `modes_valid + es_valid`. A report with `frames.mode_s` has these taken as 0. |
| `range` | Older firmware: one map for every aircraft, stored as protocol `aircraft`. Never with `ranges`. |
| `changed` | Bitmap of present bins, standard base64 with padding. Bin `i` = byte `i/8`, bit `i%8` (LSB first). |
| `max` | One byte per set bit, in bin order: `ceil(distance_m / unit_m)`, clamped 1..255. |
| Bin | `band*36 + sector`. Sector = `floor(true bearing receiver→target / 10°)`, 0 = north. |

| Protocol | Positions | `unit_m` | Bands (ft) |
|---|---|---|---|
| `mode_s` | 1090 ES (DF17/18) | 2000 | altitude (baro, else geometric): [0,1k) [1k,2k) [2k,4k) [4k,6k) [6k,10k) [10k,15k) [15k,20k) [20k,25k) [25k,30k) [30k,35k) [35k,40k) [40k,∞) |
| `uat` | 978 UAT ADS-B | 2000 | as `mode_s` |
| `remote_id` | 2.4 GHz Remote ID drones (max 12.75 km) | 50 | height above the receiver (geodetic alt − rx `alt_m`, floored at 0): [0,100) [100,200) [200,300) [300,400) [400,500) [500,700) [700,1k) [1k,1.5k) [1.5k,2k) [2k,3k) [3k,5k) [5k,∞) |
| `range` (old) | Mode S and UAT | any | as `mode_s` |

Another `unit_m`, or both `range` and `ranges`, is a 400. A protocol name this server doesn't know (in
`ranges`, `listen_s` or `frames`) is ignored and counted (`protocols_ignored`; the user API shows the bee
with `check_protocols`), so newer firmware keeps reporting to an older server. Protocol names are 1-16 of
`a-z`, `0-9`, `_`; at most 16 per object.

The protocols, units and bands come from `adsbee_swarm/protocols.json` (the registry; a test checks
this table against it).

**Position guards.** `rx_position` and `ranges` go only with a FIXED position or a GNSS fix; 0,0
counts as none. A position beyond the horizon for its band is dropped:
`limit_km = 1.15 * 4.12 * (sqrt(h_rx_m) + sqrt(h_band_top_m)) + 20` (h_rx floored at 0; the top
band uses 60k ft, or 10k ft for `remote_id`). When drops reach 10% of counted positions and 5
distinct aircraft or drones (counted over every protocol), the period's maps are withheld. Optional fields:

| Field | Meaning |
|---|---|
| `range_withheld` | `"no_position"` or `"implausible"`: only when `ranges` is omitted for that reason |
| `range_dropped` | positions dropped as beyond the horizon |

The server applies the same limit to every bin of every protocol (one `unit_m` of rounding
allowed) and ignores the rest, counting them as `bins_ignored`. A device whose last report withheld its range, dropped
positions or had bins ignored has `check_position: true` in the user API.

A position counts after 2 consistent decodes of the aircraft or drone. Report once per period (default 3600 s,
±60 s jitter); the first report goes 2–5 min after boot. On failure, max-merge new data into the
pending map and retry with backoff, holding at most one map.

**Retries.** Keep `seq` and `period.start_uptime_s` until a report is acknowledged.

| Compared with the last stored report | Server |
|---|---|
| Same `seq`, `start_uptime_s` and `len_s` | Duplicate: 200, not stored |
| Same `seq` and `start_uptime_s`, longer `len_s` | Merged retry, a superset: replaces that report's stats, `listen_s` and `frames`; its maps are max-merged per protocol |
| Same `seq` and `start_uptime_s`, shorter `len_s` | Stale copy: 200, not stored |
| Anything else | New report |

Stats, `listen_s` and `frames` in a merged retry cover the whole widened period.

**200**: `{"next_report_s": 3600, "firmware": {"tag": "0.9.2", "channel": "stable"} | null, "protocols": ["mode_s", "uat", "remote_id"]}`.
`firmware` is the account's target for this bee (see Rollouts). `protocols` is what this server takes in
`ranges`, `listen_s` and `frames`. A bee keeps the last list it got; until it has one (after boot, or a
new server or token), or when a reply has none (older servers), it sends maps for `mode_s`, `uat` and
`remote_id` only. The bee's own updater installs it only
when `AT+AUTO_UPDATE=1`, downloading from the adsbee.aero firmware API.

| Status | `error` | Bee does |
|---|---|---|
| 400 | `bad_request` | Fix the report; `detail` says what |
| 401 | `bad_token` | Unknown or cut-off token, an old token from a bee that wasn't on the account when it was replaced, disabled account |
| 403 | `bad_key` | Key check failed (adsbee.aero only) |
| 410 | `removed` | Stop until the owner restores it or gives it a newer token |
| 429 | `rate_limited` | Wait `Retry-After` seconds |
| 503 | `unavailable` | The server cannot check keys right now: back off `Retry-After` seconds (default 3600), retry |

An account has one current enroll token. A new token admits any bee; an old one keeps working for
the bees the account had when it was replaced, until the owner cuts off all bees. A removed bee is
accepted again once its owner restores it, or it reports with a token made after its removal.

Without a `DEVICE_VERIFIER` (self-hosted servers), an old token lets whoever holds it report as a
bee already on the account until "cut off all bees".

## User API

Base `/api/swarm/v1`. `Authorization: Bearer <API token>`, or a signed-in session (send `X-CSRFToken`
on POST, PATCH and DELETE). The account needs the `swarm` privilege. Another account's bee or
rollout is a 404.

| Method | Path | Does |
|---|---|---|
| GET | `/account` | Username, privileges, enroll token prefix, `enroll_token_revealable` |
| POST | `/account/enroll-token` | New enroll token with the `AT+SWARM` line; bees on the account keep theirs. `{"cut_off": true}`: every old token stops working |
| POST | `/account/enroll-token/reveal` | The current token and `AT+SWARM` line again. Signed-in session only (API token: 403 `session_only`); 404 `unavailable` when it can't be read (made before tokens were kept, or the key changed): make a new one |
| GET, POST | `/account/api-tokens` | List (prefixes) / create (shown once) |
| DELETE | `/account/api-tokens/{id}` | Delete |
| GET | `/devices` | Bees (`?removed=1` adds removed ones); `old_token`: its last report used a replaced token; `check_protocols`, `ignored_protocols`: its last report carried protocols this server doesn't know |
| GET, PATCH, DELETE | `/devices/{id}` | Read; set `label`, `note`, `location` (`null` = as reported); remove |
| POST | `/devices/{id}/restore` | Accept a removed bee again |
| GET | `/range?device=&since=&until=&protocol=` | Max per bin over the window: per bee `max_m` (12 bands × 36 sectors, meters or null) and `sector_max_m`; `swarm` = max over bees |
| GET | `/stats?device=&since=&until=&step=hour\|day` | Counts summed; `aircraft_*` maxed per bee, then summed; `bees` = bees that reported in the step; `mode_s` = every valid Mode S frame (`frames.mode_s`, plus `modes_valid + es_valid` from older firmware; both columns stay in the row); `frames` per protocol when reported. Steps without a report are left out |
| GET, POST | `/rollouts` | List / start |
| GET, POST | `/rollouts/{id}` | Read; `{"action": "promote" \| "pause" \| "resume" \| "cancel"}` |

### Range by protocol

`protocol` is `aircraft` (default), `mode_s`, `uat` or `remote_id`. `aircraft` is the max over
Mode S, UAT and older firmware's mixed maps; older firmware's maps appear only there. Remote ID is
never mixed in. `protocols` lists the ones with data for the chosen bees in the window (hide the
rest); `band_basis` says what the bands measure.

**Listening.** `listening` gives, per protocol, how many of the chosen bees `listened` and how many
`heard` something (a bee on older firmware, which sends no listen time, counts as listened when it
heard). `devices` leaves out bees that report listen time and didn't listen to the protocol; each bee
has `listen_s` (null: unknown), `listened` (true, false, or null: unknown), `heard` and `frames` (the
protocol's own frame count: Mode S as in `/stats`; null for `aircraft` and when not reported).

```
GET /api/swarm/v1/range?protocol=remote_id&device=4
{"since": "2026-10-16T20:00:00+00:00", "until": "2026-10-23T20:00:00+00:00",
 "protocol": "remote_id", "protocols": ["mode_s", "uat", "remote_id", "aircraft"],
 "sectors": 36, "bands": 12, "sector_deg": 10, "band_basis": "height_above_rx",
 "band_edges_ft": [[0, 100], [100, 200], ..., [5000, null]],
 "listening": {"mode_s": {"listened": 2, "heard": 2}, "uat": {"listened": 2, "heard": 1}, ...},
 "devices": [{"id": 4, "label": "Attic", "unique_id": "E6614C311B5B6F2A", "position": {...},
              "check_position": false, "check_protocols": false, "max_m": [[1200, null, ...], ...],
              "sector_max_m": [...], "bins": 5, "max_range_m": 3650,
              "listen_s": 86400, "listened": true, "heard": true, "frames": 5300}],
 "swarm": {"max_m": [[...]], "sector_max_m": [...]}}
```

Windows default to the last 7 days and span at most 400 days (`/stats?step=day`: any span). Every report lands in its bee's
hourly row and its daily row, with one range map per protocol; windows over 3 days, and `step=day`,
read daily rows. Hourly rows are kept 90 days, daily rows indefinitely (`manage.py swarm_retention`
rebuilds each day per protocol before its hours go).

### Rollouts

A rollout targets one product's bees: all of them, those with a label, or chosen ones. With
`test_devices` it starts on those bees only (`stage: test`); `promote` widens it to the scope. The
newest rollout that includes a bee decides its target. A paused rollout offers nothing. A cancelled
one hands the bee back to the rollout before it.

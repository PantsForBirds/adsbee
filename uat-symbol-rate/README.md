HIL data for the UAT symbol rate fix (#257), adsbee-ci-pi-0, 2026-10-06 UTC.

- `hil_counts.csv`: 10-minute windows, alternating main and PR firmware on the same ADSBee 1090 and ADSBee 1421 (via an ADSBee 1421 Programmer). Counts are RAW console frames (`#MDS`, `#UAT*-` ADS-B, `#UAT*+` uplink).
- `uplinks_1090_rc6.txt`: the 106 uplink frames from window `after1090_a`. All decode to one ground station at 37.323, -121.755.

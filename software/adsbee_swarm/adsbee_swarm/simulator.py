"""
Simulated bees posting realistic reports, for demos and tests. No Django needed to post:

    python -m adsbee_swarm.simulator --server http://localhost:8000 --token <enroll token> --bees 4

Range shapes: the radio horizon for each altitude band, 4.12 km * (sqrt(h_rx) + sqrt(h_aircraft)),
trimmed per bee by a few terrain shadows and the traffic that happens to be up there. With
`protocols`, a report carries Mode S, UAT (fewer, lower, shorter) and Remote ID (a few drones,
up to a few km) maps in `ranges`; without, one mixed `range` as older firmware sends. With
`listen`, as newer firmware: `listen_s` for those protocols, maps and stats only for them, and
nothing heard on the ones in `quiet`.
`manage.py swarm_simulate` stores weeks of history at once (no rate limits, past timestamps).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field

try:
    from . import ranges
except ImportError:  # run as a plain script
    import ranges  # type: ignore

FT = 0.3048
#: Where the demo bees sit: (label, lat, lon, antenna m above ground).
SITES = [('Attic', 37.4419, -122.1430, 9), ('Hangar', 37.4613, -122.1150, 6),
         ('Ridge', 37.3270, -122.2060, 25), ('Harbor', 37.8060, -122.4100, 12),
         ('Valley', 37.3382, -121.8863, 8), ('Coast', 36.9741, -122.0308, 15)]


@dataclass
class Bee:
    unique_id: str
    lat: float
    lon: float
    alt_m: float
    label: str = ''
    product: str = 'adsbee_1090'
    version: str = '0.9.1'
    auto_update: bool = True
    seq: int = 0
    uptime_s: int = 300
    #: Antenna, cable and noise: the share of the radio horizon this bee reaches.
    quality: float = 0.7
    shadows: list = field(default_factory=list)

    @classmethod
    def make(cls, index: int, seed: int = 1) -> 'Bee':
        label, lat, lon, alt = SITES[index % len(SITES)]
        rng = random.Random(seed * 1000 + index)
        uid = hashlib.sha256(f'bee{seed}-{index}'.encode()).hexdigest()[:16].upper()
        shadows = [(rng.uniform(0, 360), rng.uniform(20, 70), rng.uniform(0.35, 0.8)) for _ in range(rng.randint(1, 3))]
        return cls(uid, lat + rng.uniform(-.01, .01), lon + rng.uniform(-.01, .01), alt,
                   label=f'{label}' if index < len(SITES) else f'{label} {index}', shadows=shadows,
                   auto_update=index % 3 != 2, quality=rng.uniform(0.45, 0.85))

    def reach_m(self, band: int, sector: int, rng: random.Random) -> int | None:
        floor_ft = ranges.BAND_FLOORS_FT[band]
        top_ft = ranges.BAND_FLOORS_FT[band + 1] if band + 1 < ranges.BANDS else 45000
        aircraft_m = (floor_ft + top_ft) / 2 * FT
        horizon = 4120 * (math.sqrt(self.alt_m) + math.sqrt(aircraft_m))
        bearing = sector * ranges.SECTOR_DEG + 5
        factor = 1.0
        for center, width, depth in self.shadows:
            off = abs((bearing - center + 180) % 360 - 180)
            if off < width:
                # Terrain hides low aircraft most; high ones clear it sooner.
                factor *= 1 - min(.95, depth * (1 - off / width) * (1.2 if floor_ft < 6000 else .7))
        # Low bands are often empty; high bands nearly always carry traffic.
        if rng.random() > (0.35 + 0.05 * band):
            return None
        meters = horizon * self.quality * factor * rng.uniform(.6, 1.0)
        return int(min(meters, 400_000)) if meters > 2000 else None

    def drones_m(self, rng: random.Random) -> dict:
        """Remote ID: a few drones, mostly below 400 ft above the bee, a few km out at most."""
        heard = {}
        for _ in range(rng.randint(0, 6)):
            band = min(int(rng.expovariate(0.5)), ranges.BANDS - 1)
            heard[ranges.bin_index(band, rng.randrange(ranges.SECTORS))] = int(rng.uniform(150, 4000) * self.quality)
        return heard

    def report(self, period_s: int = 3600, rng: random.Random | None = None, unit_m: int = ranges.DEFAULT_UNIT_M,
               protocols: bool = False, listen: tuple[str, ...] | None = None, quiet: tuple[str, ...] = ()) -> dict:
        rng = rng or random.Random()
        start = self.uptime_s
        self.uptime_s += period_s
        self.seq += 1
        heard = {}
        for band in range(ranges.BANDS):
            for sector in range(ranges.SECTORS):
                meters = self.reach_m(band, sector, rng)
                if meters:
                    heard[ranges.bin_index(band, sector)] = meters
        if protocols:
            # UAT: general aviation, below 18k ft, fewer of them.
            uat = {i: int(m * rng.uniform(.5, .9)) for i, m in heard.items()
                   if i // ranges.SECTORS < 7 and rng.random() < .4}
            maps = {'mode_s': (heard, ranges.DEFAULT_UNIT_M), 'uat': (uat, ranges.DEFAULT_UNIT_M),
                    'remote_id': (self.drones_m(rng), ranges.PROTOCOLS['remote_id'].unit_m)}
            if listen is not None:
                maps = {name: (({}, m[1]) if name in quiet else m) for name, m in maps.items() if name in listen}
            range_fields = {'ranges': {name: _wire(values, unit) for name, (values, unit) in maps.items() if values}}
            if listen is not None:
                range_fields['listen_s'] = {name: period_s for name in listen}
        else:
            range_fields = {'range': _wire(heard, unit_m)}
        busy = rng.uniform(.3, 1.2)
        aircraft = int(rng.uniform(20, 90) * busy)
        body = {
            'v': 1, 'product': self.product, 'unique_id': self.unique_id, 'mac_address': '',
            'versions': {'rp2040': self.version, 'esp32': self.version},
            'auto_update': self.auto_update, 'uptime_s': self.uptime_s, 'seq': self.seq,
            'period': {'start_uptime_s': start, 'len_s': period_s},
            'rx_position': {'lat': self.lat, 'lon': self.lon, 'alt_m': self.alt_m, 'source': 'fixed'},
            'stats': {'modes_valid': int(aircraft * 900 * busy), 'es_valid': int(aircraft * 400 * busy),
                      'uat_adsb': int(aircraft * 3), 'uat_uplink': rng.randint(0, 600),
                      'aircraft_max': aircraft, 'aircraft_unique': int(aircraft * rng.uniform(2.5, 4))},
            **range_fields,
        }
        if listen is not None:
            for name, keys in (('mode_s', ('modes_valid', 'es_valid')), ('uat', ('uat_adsb', 'uat_uplink'))):
                for key in keys:
                    if name not in listen:
                        body['stats'].pop(key)
                    elif name in quiet:
                        body['stats'][key] = 0
        return body


def _wire(values: dict, unit_m: int) -> dict:
    changed, maxes = ranges.encode(values, unit_m)
    return {'sectors': ranges.SECTORS, 'bands': ranges.BANDS, 'unit_m': unit_m, 'changed': changed, 'max': maxes}


def post(server: str, token: str, body: dict) -> tuple[int, dict]:
    request = urllib.request.Request(f"{server.rstrip('/')}/api/swarm/v1/report", data=json.dumps(body).encode(),
                                     headers={'Authorization': f'Bearer {token}', 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return response.status, json.loads(response.read() or b'{}')
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b'{}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--server', default='http://localhost:8000')
    parser.add_argument('--token', required=True)
    parser.add_argument('--bees', type=int, default=4)
    parser.add_argument('--rounds', type=int, default=1, help='reports per bee')
    parser.add_argument('--every', type=float, default=0, help='seconds between rounds')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--legacy', action='store_true', help='one mixed range, as older firmware sends')
    args = parser.parse_args(argv)
    bees = [Bee.make(i, args.seed) for i in range(args.bees)]
    rng = random.Random(args.seed)
    for round_ in range(args.rounds):
        for bee in bees:
            status, body = post(args.server, args.token, bee.report(rng=rng, protocols=not args.legacy))
            print(f'{bee.unique_id} seq={bee.seq} -> {status} {json.dumps(body)}')
        if args.every and round_ + 1 < args.rounds:
            time.sleep(args.every)


if __name__ == '__main__':
    main()

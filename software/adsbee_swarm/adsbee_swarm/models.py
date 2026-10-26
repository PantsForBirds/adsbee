"""
    User ── Account ──< ApiToken             one login per account; privileges are flags
               └──< EnrollToken          the current one and the ones it replaced
               └──< Device ──< Sample        hourly and daily stats
                                 └──< SampleRange   one range map per protocol
               └──< Rollout >──< Device      a firmware target for all, a label, or chosen bees

Enroll tokens (what a bee is given with AT+SWARM) and API tokens are looked up by SHA-256 hash.
API tokens are shown once; the current enroll token is also kept encrypted (`sealed`) so its owner
can see it again.
"""
from __future__ import annotations

import hashlib

from django.conf import settings
from django.db import models
from django.utils import timezone

from . import ranges


def token_hash(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


class Account(models.Model):
    user = models.OneToOneField(settings.AUTH_USER_MODEL, on_delete=models.CASCADE, related_name='swarm_account')
    #: Privilege flags (conf PRIVILEGES): ["swarm", "wiggler", ...].
    privileges = models.JSONField(default=list, blank=True)
    disabled = models.BooleanField(default=False)
    created_at = models.DateTimeField(default=timezone.now)

    def __str__(self):
        return self.user.get_username()

    @property
    def enroll_token(self) -> EnrollToken | None:
        """The current enroll token (what a new bee is given)."""
        return self.enroll_tokens.filter(retired_at__isnull=True).order_by('-created_at', '-pk').first()

    @property
    def enroll_token_prefix(self) -> str:
        current = self.enroll_token
        return current.prefix if current else ''

    @property
    def enroll_token_rotated_at(self):
        current = self.enroll_token
        return current.created_at if current else None

    def has(self, privilege: str) -> bool:
        return not self.disabled and privilege in (self.privileges or [])


class ApiToken(models.Model):
    account = models.ForeignKey(Account, on_delete=models.CASCADE, related_name='api_tokens')
    name = models.CharField(max_length=64, blank=True)
    token_hash = models.CharField(max_length=64, unique=True)
    prefix = models.CharField(max_length=8)
    created_at = models.DateTimeField(default=timezone.now)
    last_used_at = models.DateTimeField(null=True, blank=True)

    def __str__(self):
        return f'{self.prefix}… {self.name}'


class EnrollToken(models.Model):
    """
    One of an account's enroll tokens. The current one (not retired) admits any bee. A retired
    one still admits the bees the account had when it was retired, until it is cut off.
    """
    account = models.ForeignKey(Account, on_delete=models.CASCADE, related_name='enroll_tokens')
    token_hash = models.CharField(max_length=64, unique=True)
    #: The first characters, so a person can tell which token a bee holds.
    prefix = models.CharField(max_length=8)
    #: The current token, encrypted (`sealed.seal`); emptied when it is retired. Empty for tokens
    #: made before tokens were kept.
    sealed = models.TextField(blank=True)
    created_at = models.DateTimeField(default=timezone.now)
    retired_at = models.DateTimeField(null=True, blank=True)
    #: Stops working for every bee.
    cut_off = models.BooleanField(default=False)

    class Meta:
        ordering = ['-created_at', '-pk']

    def __str__(self):
        return f'{self.prefix}…'

    @property
    def current(self) -> bool:
        return self.retired_at is None


class Device(models.Model):
    account = models.ForeignKey(Account, on_delete=models.CASCADE, related_name='devices')
    product = models.CharField(max_length=32)
    #: RP2040 flash UID (1090), 16 uppercase hex digits.
    unique_id = models.CharField(max_length=16)
    mac_address = models.CharField(max_length=17, blank=True)
    label = models.CharField(max_length=120, blank=True)
    note = models.CharField(max_length=500, blank=True)
    #: Set by the owner: where the map shows the bee. Empty: `rx_*` as reported.
    lat = models.FloatField(null=True, blank=True)
    lon = models.FloatField(null=True, blank=True)
    alt_m = models.FloatField(null=True, blank=True)
    rx_lat = models.FloatField(null=True, blank=True)
    rx_lon = models.FloatField(null=True, blank=True)
    rx_alt_m = models.FloatField(null=True, blank=True)
    rx_source = models.CharField(max_length=16, blank=True)
    versions = models.JSONField(default=dict, blank=True)
    auto_update = models.BooleanField(default=False)
    uptime_s = models.BigIntegerField(null=True, blank=True)
    first_seen = models.DateTimeField(default=timezone.now)
    last_report_at = models.DateTimeField(null=True, blank=True)
    #: Removed by the owner: reports answer 410 until it is restored or reports with an enroll
    #: token made after its removal.
    removed_at = models.DateTimeField(null=True, blank=True)
    #: The enroll token of its last accepted report.
    enroll_token = models.ForeignKey(EnrollToken, null=True, blank=True, on_delete=models.SET_NULL,
                                     related_name='+')
    #: The last report stored: [seq, start_uptime_s, len_s], its Sample's start and its stats, so a
    #: retry is recognized and a merged retry replaces what it widens.
    last_report = models.JSONField(default=dict, blank=True)
    #: The last report's position guards (docs/swarm_api.md): why it sent no range ('' none,
    #: 'no_position', 'implausible'), positions it dropped, bins the server ignored.
    range_withheld = models.CharField(max_length=16, blank=True)
    range_dropped = models.PositiveIntegerField(default=0)
    bins_ignored = models.PositiveIntegerField(default=0)

    class Meta:
        ordering = ['label', 'unique_id']
        constraints = [models.UniqueConstraint(fields=['account', 'product', 'unique_id'], name='swarm_device_unique')]

    def __str__(self):
        return self.label or f'{self.product}:{self.unique_id}'

    @property
    def removed(self) -> bool:
        return self.removed_at is not None

    @property
    def position(self) -> dict | None:
        """Where to draw the bee: the owner's location, else the reported one."""
        if self.lat is not None and self.lon is not None:
            return {'lat': self.lat, 'lon': self.lon, 'alt_m': self.alt_m, 'source': 'owner'}
        if self.rx_lat is not None and self.rx_lon is not None:
            return {'lat': self.rx_lat, 'lon': self.rx_lon, 'alt_m': self.rx_alt_m, 'source': self.rx_source}
        return None

    @property
    def old_token(self) -> bool:
        """Its last report used a token that has since been replaced: give it the new line."""
        return self.enroll_token_id is not None and self.enroll_token.retired_at is not None

    @property
    def check_position(self) -> bool:
        """The last report says its position is missing or doubtful: worth a look."""
        return bool(self.range_withheld or self.range_dropped or self.bins_ignored)

    @property
    def version(self) -> str:
        from . import conf

        key = conf.products().get(self.product, {}).get('version_key', '')
        return str((self.versions or {}).get(key, ''))


#: What a report's `stats` holds.
REPORTED_STATS = ('modes_valid', 'es_valid', 'uat_adsb', 'uat_uplink', 'aircraft_max', 'aircraft_unique')
#: Every count a sample keeps: the reported ones, positions the bee dropped as implausible
#: (`range_dropped`) and bins past the horizon limit the server ignored (`bins_ignored`).
STATS = (*REPORTED_STATS, 'range_dropped', 'bins_ignored')
#: Why a report sent no range, least to most severe; a sample keeps the most severe.
WITHHELD = ('', 'no_position', 'implausible')
#: Stats that are a peak, not a count: merged by max rather than summed.
PEAK_STATS = ('aircraft_max', 'aircraft_unique')


class Sample(models.Model):
    """One device's hour (span 3600) or day (86400): summed counts; range maps in `SampleRange`."""
    HOUR, DAY = 3600, 86400
    device = models.ForeignKey(Device, on_delete=models.CASCADE, related_name='samples')
    start = models.DateTimeField()
    span_s = models.PositiveIntegerField(default=HOUR)
    reports = models.PositiveIntegerField(default=0)
    modes_valid = models.BigIntegerField(default=0)
    es_valid = models.BigIntegerField(default=0)
    uat_adsb = models.BigIntegerField(default=0)
    uat_uplink = models.BigIntegerField(default=0)
    aircraft_max = models.IntegerField(default=0)
    aircraft_unique = models.IntegerField(default=0)
    range_dropped = models.BigIntegerField(default=0)
    bins_ignored = models.BigIntegerField(default=0)
    range_withheld = models.CharField(max_length=16, blank=True)

    class Meta:
        ordering = ['start']
        constraints = [models.UniqueConstraint(fields=['device', 'start', 'span_s'], name='swarm_sample_unique')]
        indexes = [models.Index(fields=['device', 'span_s', 'start'])]

    def range_maps(self) -> dict[str, ranges.Map]:
        """{stored protocol: map}."""
        return {row.protocol: row.ranges() for row in self.range_rows.all()}

    def ranges(self, protocol: str = ranges.LEGACY) -> ranges.Map:
        """The map a user sees for `protocol`: `aircraft` is the max over every aircraft map."""
        merged: ranges.Map = {}
        for name, values in self.range_maps().items():
            if name in ranges.SOURCES[protocol]:
                ranges.merge(merged, values)
        return merged


class SampleRange(models.Model):
    """A sample's range map for one protocol (`ranges.PROTOCOLS`): max distance per bin."""
    sample = models.ForeignKey(Sample, on_delete=models.CASCADE, related_name='range_rows')
    protocol = models.CharField(max_length=16)
    unit_m = models.PositiveIntegerField(default=ranges.DEFAULT_UNIT_M)
    #: `ranges.pack`: 54-byte bitmap, then one byte per present bin.
    range_map = models.BinaryField()

    class Meta:
        constraints = [models.UniqueConstraint(fields=['sample', 'protocol'], name='swarm_sample_range_unique')]

    def ranges(self) -> ranges.Map:
        return ranges.unpack(self.range_map, self.unit_m)

    def set_ranges(self, values: ranges.Map) -> None:
        """Store `values` in `unit_m` steps (rounded up)."""
        self.range_map = ranges.pack(values, self.unit_m)


class Rollout(models.Model):
    """
    A firmware target for an account's bees of one product. Scope picks the bees; a rollout in
    stage `test` targets only its test bees, `all` every bee in scope. Newest active rollout wins.
    """
    SCOPE_ALL, SCOPE_LABEL, SCOPE_DEVICES = 'all', 'label', 'devices'
    SCOPES = [(SCOPE_ALL, 'All bees'), (SCOPE_LABEL, 'Bees with a label'), (SCOPE_DEVICES, 'Chosen bees')]
    STAGE_TEST, STAGE_ALL = 'test', 'all'
    STAGES = [(STAGE_TEST, 'Test bees'), (STAGE_ALL, 'Every bee in scope')]
    account = models.ForeignKey(Account, on_delete=models.CASCADE, related_name='rollouts')
    product = models.CharField(max_length=32)
    tag = models.CharField(max_length=64)
    channel = models.CharField(max_length=16, default='stable')
    scope = models.CharField(max_length=8, choices=SCOPES, default=SCOPE_ALL)
    label = models.CharField(max_length=120, blank=True)
    devices = models.ManyToManyField(Device, blank=True, related_name='rollouts')
    test_devices = models.ManyToManyField(Device, blank=True, related_name='test_rollouts')
    stage = models.CharField(max_length=8, choices=STAGES, default=STAGE_ALL)
    paused = models.BooleanField(default=False)
    cancelled_at = models.DateTimeField(null=True, blank=True)
    note = models.CharField(max_length=200, blank=True)
    created_at = models.DateTimeField(default=timezone.now)
    promoted_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ['-created_at', '-id']

    def __str__(self):
        return f'{self.product} {self.tag} #{self.pk}'

    @property
    def state(self) -> str:
        if self.cancelled_at:
            return 'cancelled'
        if self.paused:
            return 'paused'
        return self.stage

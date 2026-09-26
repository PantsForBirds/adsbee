"""RF loopback test: a transmitter plays a known pattern, a receiver must report it.

With no transmitter configured or attached the test is skipped, not failed, so it can sit in CI
before the bench has one.
"""

import time
from dataclasses import dataclass, field
from typing import Dict, Optional

from .receivers import Receiver
from .transmitters import TestPattern, Transmitter

_KINDS = {"1090": {"mode_s"}, "978": {"uat_adsb"}, "dual": {"mode_s", "uat_adsb"}}


@dataclass
class LoopbackResult:
    status: str  # "pass", "fail" or "skip"
    receiver: str
    transmitter: Optional[str] = None
    sent: int = 0
    received: int = 0  # frames of the pattern's kind(s) reported during the window
    matched: int = 0  # of those, frames equal to one of the pattern's explicit messages
    distinct_icao: int = 0
    counters: Dict[str, int] = field(default_factory=dict)  # receiver counter deltas, if any
    detail: str = ""

    def summary(self) -> str:
        return (f"{self.status.upper():4s} {self.receiver} <- {self.transmitter or '-'}: sent {self.sent}, "
                f"received {self.received}, matched {self.matched}, distinct ICAO {self.distinct_icao}"
                + (f" ({self.detail})" if self.detail else ""))


def rf_loopback(receiver: Receiver, transmitter: Optional[Transmitter], pattern: TestPattern,
                min_fraction: float = 0.5, settle_s: float = 1.0) -> LoopbackResult:
    """Transmits ``pattern`` and checks the receiver reported at least min_fraction of it.

    With explicit messages, only frames identical to a sent message count (background traffic
    can't pass the test). With an instrument's built-in table the content is unknown here, so all
    frames of the band's kind count; use a shielded/cabled setup or explicit messages for strict
    results. Per band, "sent" is pattern.count (dual-band patterns send count on each band).
    """
    res = LoopbackResult("skip", receiver.id, transmitter.id if transmitter else None)
    if transmitter is None:
        res.detail = "no transmitter configured for this receiver"
        return res
    if not transmitter.available():
        res.detail = f"transmitter {transmitter.id} not attached"
        return res
    if pattern.count < 0:
        raise ValueError("loopback needs a finite pattern")
    kinds = _KINDS[pattern.band]
    res.sent = pattern.count * len(kinds)

    try:
        before = receiver.rx_counters()
    except Exception:
        before = {}
    with receiver.capture() as cap:
        time.sleep(settle_s)
        transmitter.transmit(pattern)
        time.sleep(settle_s)
    try:
        after = receiver.rx_counters() if before else {}
    except Exception:
        after = {}
    res.counters = {k: after[k] - before.get(k, 0) for k in after if isinstance(after[k], int)}

    frames = [f for f in cap.frames if f.kind in kinds]
    res.received = len(frames)
    res.distinct_icao = len({f.icao for f in frames if f.icao is not None})
    wanted = {m.upper() for m in pattern.messages}
    if wanted:
        res.matched = sum(1 for f in frames if f.hex in wanted)
        got = res.matched
    else:
        got = res.received
    need = int(res.sent * min_fraction + 0.999)
    res.status = "pass" if got >= need else "fail"
    res.detail = f"needed {need}"
    return res

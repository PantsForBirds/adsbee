"""Model/type registries (with entry-point plugins) and device selection."""

from importlib import metadata
from typing import Dict, List, Optional, Type

from . import usb
from .config import Bench, ConfigError, ReceiverConfig, TransmitterConfig
from .receivers import BUILTIN_MODELS, Receiver
from .transmitters import BUILTIN_TYPES, Transmitter

RECEIVER_GROUP = "adsbee_hil.receivers"
TRANSMITTER_GROUP = "adsbee_hil.transmitters"


def _plugins(group: str) -> Dict[str, type]:
    found = {}
    try:
        eps = metadata.entry_points(group=group)
    except TypeError:  # pragma: no cover  (Python < 3.10)
        eps = metadata.entry_points().get(group, [])
    for ep in eps:
        found[ep.name] = ep.load()
    return found


def receiver_models() -> Dict[str, Type[Receiver]]:
    return {**BUILTIN_MODELS, **_plugins(RECEIVER_GROUP)}


def transmitter_types() -> Dict[str, Type[Transmitter]]:
    return {**BUILTIN_TYPES, **_plugins(TRANSMITTER_GROUP)}


def make_receiver(cfg: ReceiverConfig, bench: Bench, lock_timeout: float = 600.0) -> Receiver:
    models = receiver_models()
    if cfg.model not in models:
        raise ConfigError(f"receiver '{cfg.id}': unknown model '{cfg.model}' (known: {sorted(models)})")
    return models[cfg.model](cfg, bench, lock_timeout)


def make_transmitter(cfg: TransmitterConfig, bench: Bench, lock_timeout: float = 600.0) -> Transmitter:
    types = transmitter_types()
    if cfg.type not in types:
        raise ConfigError(f"transmitter '{cfg.id}': unknown type '{cfg.type}' (known: {sorted(types)})")
    return types[cfg.type](cfg, bench, lock_timeout)


def select_receivers(bench: Bench, targets: List[str] = (), model: Optional[str] = None,
                     tag: Optional[str] = None, all_: bool = False, lock_timeout: float = 600.0) -> List[Receiver]:
    """Receivers named by id or USB serial, or all of them (optionally one model / tag).

    A USB serial that isn't in the bench file works too, given ``model`` (or a USB product
    string that identifies the model), so a single board can be driven without a bench file.
    """
    chosen: List[ReceiverConfig] = []
    for t in targets:
        cfg = bench.receiver(t)
        if cfg is None:
            d = usb.find_by_serial(t)
            m = model or (d.model_hint if d else None)
            if not m:
                raise ConfigError(f"'{t}' is not a receiver id or serial in the bench file; "
                                  "pass --model to use it ad hoc")
            cfg = ReceiverConfig(id=t, model=m, usb_serial=t)
        chosen.append(cfg)
    if all_ or (not targets and (model or tag)):
        for cfg in bench.receivers:
            if cfg not in chosen:
                chosen.append(cfg)
    if model:
        chosen = [c for c in chosen if c.model == model]
    if tag:
        chosen = [c for c in chosen if tag in c.tags]
    return [make_receiver(c, bench, lock_timeout) for c in chosen]


def transmitters_for(bench: Bench, receiver_id: str, lock_timeout: float = 600.0) -> List[Transmitter]:
    return [make_transmitter(t, bench, lock_timeout) for t in bench.transmitters
            if not t.receivers or receiver_id in t.receivers]

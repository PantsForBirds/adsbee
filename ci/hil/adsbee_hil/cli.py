"""adsbee-hil: command-line front end for the bench toolkit. See ci/hil/README.md."""

import argparse
import json
import os
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, List

from . import config as config_mod
from . import lock as lock_mod
from . import usb
from .console import redact
from .loopback import rf_loopback
from .receivers import Receiver
from .registry import (make_receiver, make_transmitter, receiver_models, select_receivers, transmitter_types,
                       transmitters_for)
from .transmitters import PATTERNS, TestPattern

_print_lock = threading.Lock()


def out(prefix: str, text: str) -> None:
    with _print_lock:
        for line in text.rstrip("\n").splitlines() or [""]:
            print(f"[{prefix}] {line}" if prefix else line, flush=True)


def run_parallel(receivers: List[Receiver], fn: Callable[[Receiver], None], jobs: int) -> int:
    """Runs fn on every receiver concurrently; returns 1 if any failed (each is reported), else 0."""
    if not receivers:
        print("no receivers selected (use -d ID/SERIAL, --model, --tag or --all)", file=sys.stderr)
        return 1

    def one(r: Receiver) -> bool:
        try:
            fn(r)
            return True
        except Exception as e:  # noqa: BLE001 - reported per device, others keep going
            out(r.id, f"ERROR: {e}")
            return False

    with ThreadPoolExecutor(max_workers=max(1, min(jobs, len(receivers)))) as ex:
        results = list(ex.map(one, receivers))
    return 1 if False in results else 0


# -- commands -------------------------------------------------------------------------------------


def cmd_discover(args, bench) -> int:
    devices = usb.adsbee_candidates()
    rows = []
    for d in devices:
        cfg = bench.receiver(d.serial) or bench.transmitter(d.serial)
        row = {
            "usb_port": d.port_path, "vidpid": d.vidpid, "mode": d.mode, "serial": d.serial,
            "product": d.product, "manufacturer": d.manufacturer, "console": d.console,
            "model_hint": d.model_hint, "bench_id": cfg.id if cfg else None,
            "bench_model": getattr(cfg, "model", None) or getattr(cfg, "type", None) if cfg else None,
        }
        if args.probe and d.mode == "app" and not isinstance(cfg, config_mod.TransmitterConfig):
            model = cfg.model if cfg else (d.model_hint or "adsbee_1090u")
            try:
                rx = make_receiver(config_mod.ReceiverConfig(d.serial, model, d.serial), bench, args.lock_timeout)
                info = rx.device_info()
                row["device_info"] = info
            except Exception as e:  # noqa: BLE001
                row["probe_error"] = str(e)
        rows.append(row)
    missing = [r for r in bench.receivers if not any(d.serial == r.usb_serial for d in devices)]
    if args.json:
        print(json.dumps({"devices": rows, "missing": [m.id for m in missing]}, indent=2))
        return 0
    if not rows:
        print("no RP2040-family USB devices found")
    for r in rows:
        name = r["bench_id"] or "-"
        model = r["bench_model"] or (f"{r['model_hint']}?" if r["model_hint"] else "?")
        print(f"{r['usb_port']:10s} {r['vidpid']} {r['mode']:7s} serial={r['serial'] or '-':18s} "
              f"id={name:12s} model={model:14s} '{r['manufacturer']} {r['product']}'")
        if r["console"]:
            print(f"{'':10s} console {r['console']}")
        for k, v in r.get("device_info", {}).items():
            print(f"{'':10s} {k}: {v}")
        if "probe_error" in r:
            print(f"{'':10s} probe failed: {r['probe_error']}")
    for m in missing:
        print(f"{'-':10s} {'':9s} absent  serial={m.usb_serial:18s} id={m.id:12s} model={m.model}")
    if any(r["mode"] == "bootsel" for r in rows):
        print("note: BOOTSEL devices report the bootrom's serial, not the app's; they are matched to a "
              "receiver by USB port.")
    return 0


def cmd_list(args, bench) -> int:
    print(f"bench: {bench.name} ({bench.path or 'no bench file'})")
    for r in bench.receivers:
        try:
            state = make_receiver(r, bench).state()
        except Exception as e:  # noqa: BLE001
            state = f"error: {e}"
        print(f"  rx {r.id:14s} {r.model:14s} serial={r.usb_serial:18s} port={r.usb_port or '-':8s} "
              f"tags={','.join(r.tags) or '-':10s} {state}")
    for t in bench.transmitters:
        try:
            avail = "attached" if make_transmitter(t, bench).available() else "not attached"
        except Exception as e:  # noqa: BLE001
            avail = f"error: {e}"
        print(f"  tx {t.id:14s} {t.type:14s} -> {','.join(t.receivers) or 'all receivers':24s} {avail}")
    print(f"models: {', '.join(sorted(receiver_models()))}; transmitter types: "
          f"{', '.join(sorted(transmitter_types()))}")
    return 0


def _select(args, bench) -> List[Receiver]:
    return select_receivers(bench, args.device or [], args.model, args.tag, args.all, args.lock_timeout)


def cmd_port(args, bench) -> int:
    return run_parallel(_select(args, bench), lambda r: out("", r.console_path()), args.jobs)


def cmd_at(args, bench) -> int:
    def fn(r):
        text = r.at(args.command, timeout=args.timeout, allow_tx=args.allow_tx)
        out(r.id, text if args.show_secrets else redact(text))
    return run_parallel(_select(args, bench), fn, args.jobs)


def cmd_info(args, bench) -> int:
    results = {}

    def fn(r):
        info = r.device_info()
        results[r.id] = {"model": r.model, "state": r.state(), **info}
        if not args.json:
            for k, v in info.items():
                out(r.id, f"{k}: {v}")
    rc = run_parallel(_select(args, bench), fn, args.jobs)
    if args.json:
        print(json.dumps(results, indent=2))
    return rc


def cmd_rx_stats(args, bench) -> int:
    def fn(r):
        if args.reset and hasattr(r, "reset_rx_counters"):
            r.reset_rx_counters()
        c = r.rx_counters()
        out(r.id, ", ".join(f"{k}={v}" for k, v in c.items()) or f"{r.model} has no receive counters")
    return run_parallel(_select(args, bench), fn, args.jobs)


def cmd_capture(args, bench) -> int:
    import time

    def fn(r):
        with r.capture() as cap:
            time.sleep(args.seconds)
        frames = cap.frames
        kinds = {}
        for f in frames:
            kinds[f.kind] = kinds.get(f.kind, 0) + 1
        out(r.id, f"{len(frames)} frames in {args.seconds:.0f} s {kinds or ''}".rstrip())
        if args.print:
            for f in frames:
                out(r.id, f"{f.kind} {f.hex} {f.meta}")
    return run_parallel(_select(args, bench), fn, args.jobs)


def cmd_reboot(args, bench) -> int:
    return run_parallel(_select(args, bench), lambda r: (r.reboot(), out(r.id, "rebooted")), args.jobs)


def cmd_flash(args, bench) -> int:
    def fn(r):
        info = r.flash(args.image, force=args.force, host=args.host, verbose=args.verbose)
        for k, v in info.items():
            out(r.id, f"{k}: {v}")
    return run_parallel(_select(args, bench), fn, args.jobs)


def _pattern(args) -> TestPattern:
    base = PATTERNS[args.pattern] if args.pattern else TestPattern()
    p = TestPattern(**{**base.__dict__})
    for k in ("band", "count", "rate", "power_dbm", "atten_db"):
        v = getattr(args, k, None)
        if v is not None:
            setattr(p, k, v)
    if getattr(args, "message", None):
        p.messages = args.message
    return p


def _transmitter(args, bench):
    cfg = bench.transmitter(args.transmitter)
    if not cfg:
        raise config_mod.ConfigError(f"no transmitter '{args.transmitter}' in the bench file")
    return make_transmitter(cfg, bench, args.lock_timeout)


def cmd_tx(args, bench) -> int:
    import time

    tx = _transmitter(args, bench)
    if args.tx_cmd == "info":
        print(f"{tx.id}: {tx.type}, {'attached' if tx.available() else 'not attached'}")
        for k, v in tx.info().items():
            print(f"  {k}: {v}")
        return 0
    if args.tx_cmd == "stop":
        # Emergency stop for a transmission left running by another process.
        if hasattr(tx, "at"):
            print(tx.at("x", 3.0) or "(no reply)")
        return 0
    p = _pattern(args)
    if args.tx_cmd == "play":
        print(f"{tx.id}: playing {p}", flush=True)
        tx.transmit(p)
    else:  # run
        p.count = -1
        print(f"{tx.id}: transmitting {p} for {args.seconds:.0f} s", flush=True)
        with tx.running(p):
            time.sleep(args.seconds)
    print("done")
    return 0


def cmd_loopback(args, bench) -> int:
    p = _pattern(args)
    results = []

    def fn(r):
        txs = transmitters_for(bench, r.id, args.lock_timeout)
        if args.transmitter:
            txs = [t for t in txs if t.id == args.transmitter]
        res = rf_loopback(r, txs[0] if txs else None, p, args.min_fraction)
        results.append(res)
        out(r.id, res.summary())
        if res.status == "fail":
            raise RuntimeError("loopback failed")
    # Receivers sharing a transmitter would run one after another anyway (transmitter lock).
    if run_parallel(_select(args, bench), fn, args.jobs):
        return 1
    if args.fail_on_skip and any(r.status == "skip" for r in results):
        return 1
    return 0


def cmd_locks(args, bench) -> int:
    d = lock_mod.lock_dir(bench.lock_dir)
    print(f"lock dir: {d}")
    try:
        names = sorted(n for n in os.listdir(d) if n.endswith(".lock"))
    except FileNotFoundError:
        names = []
    for n in names:
        p = os.path.join(d, n)
        held = lock_mod.is_locked(p)
        print(f"  {n[:-5]:20s} {'LOCKED by ' + (lock_mod.read_holder(p) or '?') if held else 'free'}")
    return 0


def cmd_lock(args, bench) -> int:
    """Runs a command while holding the device's lock (for scripts that don't use this package)."""
    cfg = bench.receiver(args.target) or bench.transmitter(args.target)
    key = cfg.usb_serial if cfg and cfg.usb_serial else args.target
    cmd = args.cmd[1:] if args.cmd and args.cmd[0] == "--" else args.cmd
    if not cmd:
        print("usage: adsbee-hil lock DEVICE -- COMMAND ...", file=sys.stderr)
        return 2
    with lock_mod.DeviceLock(key, bench.lock_dir, args.lock_timeout, " ".join(cmd)[:80]):
        return subprocess.call(cmd)


# -- argument parsing -----------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="adsbee-hil", description=__doc__)
    ap.add_argument("-c", "--config", help="bench file (default: $ADSBEE_HIL_CONFIG, "
                                           "~/.config/adsbee-hil/bench.toml)")
    ap.add_argument("--lock-timeout", type=float, default=600.0, metavar="S",
                    help="max seconds to wait for a busy device (default 600)")
    ap.add_argument("-j", "--jobs", type=int, default=8, help="devices handled concurrently (default 8)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sel = argparse.ArgumentParser(add_help=False)
    g = sel.add_argument_group("device selection")
    g.add_argument("-d", "--device", action="append", metavar="ID|SERIAL",
                   help="receiver id or USB serial (repeatable)")
    g.add_argument("-m", "--model", help="only this model (with no -d: every receiver of it); "
                                         "also the model of an ad-hoc serial")
    g.add_argument("-t", "--tag", help="only receivers with this tag")
    g.add_argument("-a", "--all", action="store_true", help="every receiver in the bench file")

    pat = argparse.ArgumentParser(add_help=False)
    g = pat.add_argument_group("test pattern")
    g.add_argument("--pattern", choices=sorted(PATTERNS), help="named base pattern")
    g.add_argument("--band", choices=["1090", "978", "dual"])
    g.add_argument("--count", type=int)
    g.add_argument("--rate", type=float, help="packets per second")
    g.add_argument("--power-dbm", type=float, dest="power_dbm")
    g.add_argument("--atten-db", type=float, dest="atten_db")
    g.add_argument("--message", action="append", help="explicit hex message (repeatable)")

    p = sub.add_parser("discover", help="list attached ADSBee-family USB devices (no bench file needed)")
    p.add_argument("--json", action="store_true")
    p.add_argument("--probe", action="store_true", help="also ask each app-mode device for AT+DEVICE_INFO? "
                                                        "(takes each device's lock)")
    p.set_defaults(fn=cmd_discover)

    p = sub.add_parser("list", help="bench file inventory and state")
    p.set_defaults(fn=cmd_list)

    p = sub.add_parser("port", parents=[sel], help="print the console path of devices")
    p.set_defaults(fn=cmd_port)

    p = sub.add_parser("at", parents=[sel], help="send one AT command")
    p.add_argument("command")
    p.add_argument("-T", "--timeout", type=float, default=3.0)
    p.add_argument("--allow-tx", action="store_true", help="allow commands that make the receiver transmit")
    p.add_argument("--show-secrets", action="store_true", help="don't redact OTA keys")
    p.set_defaults(fn=cmd_at)

    p = sub.add_parser("info", parents=[sel], help="device info and firmware versions")
    p.add_argument("--json", action="store_true")
    p.set_defaults(fn=cmd_info)

    p = sub.add_parser("rx-stats", parents=[sel], help="receive counters (models that have them)")
    p.add_argument("--reset", action="store_true")
    p.set_defaults(fn=cmd_rx_stats)

    p = sub.add_parser("capture", parents=[sel], help="count packets reported on the console (RAW)")
    p.add_argument("-s", "--seconds", type=float, default=10.0)
    p.add_argument("--print", action="store_true", help="print every frame")
    p.set_defaults(fn=cmd_capture)

    p = sub.add_parser("reboot", parents=[sel], help="reboot devices")
    p.set_defaults(fn=cmd_reboot)

    p = sub.add_parser("flash", parents=[sel], help="flash firmware (.uf2, .hex or .ota by model)")
    p.add_argument("image")
    p.add_argument("--force", action="store_true", help="flash even if a busy_processes match is running")
    p.add_argument("--host", help="1090U OTA: device host name")
    p.add_argument("-v", "--verbose", action="store_true")
    p.set_defaults(fn=cmd_flash)

    p = sub.add_parser("tx", help="drive a transmitter")
    tsub = p.add_subparsers(dest="tx_cmd", required=True)
    for name, helptext in (("info", "identity and availability"), ("play", "play a finite pattern"),
                           ("run", "transmit continuously for --seconds"), ("stop", "stop a stray transmission")):
        tp = tsub.add_parser(name, parents=[pat] if name in ("play", "run") else [], help=helptext)
        tp.add_argument("transmitter", help="transmitter id from the bench file")
        if name == "run":
            tp.add_argument("-s", "--seconds", type=float, default=10.0)
    p.set_defaults(fn=cmd_tx)

    p = sub.add_parser("loopback", parents=[sel, pat], help="RF loopback: transmit a pattern, count receptions")
    p.add_argument("--transmitter", help="transmitter id (default: first one cabled to each receiver)")
    p.add_argument("--min-fraction", type=float, default=0.5)
    p.add_argument("--fail-on-skip", action="store_true", help="exit 1 when no transmitter is available")
    p.set_defaults(fn=cmd_loopback)

    p = sub.add_parser("locks", help="show device locks and their holders")
    p.set_defaults(fn=cmd_locks)

    p = sub.add_parser("lock", help="run a command while holding a device's lock")
    p.add_argument("target", help="receiver/transmitter id or USB serial")
    p.add_argument("cmd", nargs=argparse.REMAINDER)
    p.set_defaults(fn=cmd_lock)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        bench = config_mod.load(args.config)
        return args.fn(args, bench) or 0
    except (config_mod.ConfigError, lock_mod.LockTimeout) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    except BrokenPipeError:  # e.g. piped into head
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        return 0


if __name__ == "__main__":
    sys.exit(main())

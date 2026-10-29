"""``python -m awr1843_auto [gui|run] ...``"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from awr1843.config import load_cfg_file, parse_cfg

CONFIGS = Path(__file__).resolve().parent / "configs"
MODE_CFG = {"drive": "drive_long_range.cfg", "park": "park_short_range.cfg",
            "cabin": "cabin_occupancy.cfg"}


def cmd_gui(args) -> int:
    from .gui.app import run_gui

    return run_gui(sim=args.sim, mode=args.mode, cli_port=args.cli_port,
                   data_port=args.data_port, replay=args.replay)


def cmd_run(args) -> int:
    """Terminal mode: print one line per second."""
    from awr1843.device import RadarDevice

    from .cabin import CabinGuard
    from .drive import DriveAssist
    from .park import ParkAssist
    from .simulator import simulated_auto_device

    lines = load_cfg_file(CONFIGS / MODE_CFG[args.mode])
    params = parse_cfg(lines)
    dev = simulated_auto_device() if args.sim else RadarDevice.connect(args.cli_port, args.data_port)
    proc = {"drive": DriveAssist, "park": ParkAssist, "cabin": CabinGuard}[args.mode](params)
    with dev:
        dev.configure(lines)
        print(f"{args.mode} mode running - Ctrl+C to stop")
        t_end = time.time() + args.duration if args.duration else None
        last = 0.0
        try:
            for f in dev.frames(timeout=3):
                if args.mode == "cabin":
                    proc.add_frame(f)
                    r = None
                else:
                    r = proc.process(f)
                if time.time() - last >= 1.0:
                    last = time.time()
                    if args.mode == "drive":
                        lead = f"lead {r.lead_distance:5.1f} m" if r.lead_distance else "no lead   "
                        gap = f"gap {r.time_gap_s:.1f}s" if r.time_gap_s else ""
                        print(f"{r.ego.speed * 3.6:5.1f} km/h  {lead} {gap:9s} tracks {len(r.tracks):2d}  "
                              + " | ".join(a.text for a in r.alerts), flush=True)
                    elif args.mode == "park":
                        d = r.nearest
                        print(f"nearest {d:.2f} m ({r.nearest_sector})  zone {r.zone}" if math.isfinite(d)
                              else "nothing within range", flush=True)
                    else:
                        c = proc.update()
                        seats = " ".join(f"{s.seat.key}:{'X' if s.occupied else '.'}" for s in c.seats)
                        print(f"{c.status:20s} {seats}  {c.cpd_text}", flush=True)
                if t_end and time.time() > t_end:
                    break
        except KeyboardInterrupt:
            pass
        dev.stop()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="awr1843_auto",
                                 description="Drive Assist, Park Assist and Cabin Guard with the "
                                             "AWR1843BOOST radar.")
    sub = ap.add_subparsers(dest="command")
    for name, fn in (("gui", cmd_gui), ("run", cmd_run)):
        p = sub.add_parser(name)
        p.add_argument("--mode", choices=list(MODE_CFG), default="drive")
        p.add_argument("--sim", action="store_true", help="use the simulator (no hardware)")
        p.add_argument("--cli-port")
        p.add_argument("--data-port")
        if name == "gui":
            p.add_argument("--replay")
        else:
            p.add_argument("--duration", type=float)
        p.set_defaults(func=fn)
    args = ap.parse_args(argv)
    if not args.command:
        args = ap.parse_args(["gui"] + (argv if argv is not None else sys.argv[1:]))
    try:
        return args.func(args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

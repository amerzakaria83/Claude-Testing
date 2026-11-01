"""``python -m awr1843_vitals [gui|run|analyze] ...``"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from awr1843.config import load_cfg_file, parse_cfg

DEFAULT_CFG = Path(__file__).resolve().parent / "configs" / "vitals_15fps.cfg"


def _fmt(v, nd=0):
    return "  --" if v is None else f"{v:4.{nd}f}"


def _print(r, extra=""):
    print(f"t={r.time_s:7.1f}s  BR {_fmt(r.breath_bpm)} /min  HR {_fmt(r.heart_bpm)} bpm  "
          f"dist {_fmt(r.distance_m, 2)} m  {r.status}{extra}", flush=True)


def cmd_run(args) -> int:
    from awr1843.device import RadarDevice
    from awr1843.recorder import Recorder, new_recording_dir

    from .processing import VitalSignsProcessor
    from .recorder import VitalsLog
    from .simulator import simulated_vitals_device

    lines = load_cfg_file(args.cfg)
    radar = simulated_vitals_device() if args.sim else RadarDevice.connect(args.cli_port,
                                                                          args.data_port)
    with radar:
        proc = VitalSignsProcessor(parse_cfg(lines))
        rec = log = None
        if args.record is not None:
            out = Path(args.record) if args.record else new_recording_dir()
            rec = Recorder(out, lines, save_heatmaps=False)
            rec.attach(radar)
            log = VitalsLog(out / "vitals.csv")
            print(f"Recording to {out}")
        radar.configure(lines)
        print(f"Measuring at {proc.fs:.0f} fps - sit still in front of the radar. Ctrl+C to stop.")
        t_end = time.time() + args.duration if args.duration else None
        last = 0.0
        try:
            for frame in radar.frames(timeout=3):
                proc.add_frame(frame)
                if time.time() - last >= 1.0:
                    last = time.time()
                    r = proc.update()
                    _print(r)
                    if log:
                        log.write(r)
                if t_end and time.time() > t_end:
                    break
        except KeyboardInterrupt:
            pass
        radar.stop()
        if rec:
            rec.detach(radar)
            rec.close()
            log.close()
    return 0


def cmd_analyze(args) -> int:
    """Run the vital-sign processing over a recorded raw_data.bin."""
    from awr1843.recorder import _params_for, read_raw_frames

    from .processing import VitalSignsProcessor
    from .recorder import VitalsLog

    raw = Path(args.raw)
    params, _ = _params_for(raw, args.cfg)
    if params is None:
        print("config.cfg not found next to the capture; pass --cfg", file=sys.stderr)
        return 1
    proc = VitalSignsProcessor(params)
    out = Path(args.out) if args.out else raw.with_name("vitals_analyzed.csv")
    log = VitalsLog(out)
    step = max(1, int(round(params.frame_rate_hz)))
    for i, frame in enumerate(read_raw_frames(raw, params.num_range_bins)):
        proc.add_frame(frame)
        if i % step == 0:
            r = proc.update()
            log.write(r)
            if not args.quiet:
                _print(r)
    log.close()
    print(f"Wrote {log.rows} rows to {out}")
    return 0


def cmd_gui(args) -> int:
    from .gui.app import run_gui

    return run_gui(sim=args.sim, cfg=args.cfg, cli_port=args.cli_port,
                   data_port=args.data_port, replay=args.replay)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="awr1843_vitals",
                                 description="Contactless heart and breathing rate with the "
                                             "AWR1843BOOST radar.")
    sub = ap.add_subparsers(dest="command")

    def conn(p):
        p.add_argument("--cli-port")
        p.add_argument("--data-port")
        p.add_argument("--sim", action="store_true", help="simulated person, no hardware")
        p.add_argument("--cfg", default=None)

    p = sub.add_parser("gui", help="open the dashboard (default)")
    conn(p)
    p.add_argument("--replay", help="raw_data.bin to replay")
    p.set_defaults(func=cmd_gui)

    p = sub.add_parser("run", help="measure in the terminal")
    conn(p)
    p.add_argument("--duration", type=float)
    p.add_argument("--record", nargs="?", const="", default=None)
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("analyze", help="process a recorded raw_data.bin")
    p.add_argument("raw")
    p.add_argument("--cfg")
    p.add_argument("--out")
    p.add_argument("--quiet", action="store_true")
    p.set_defaults(func=cmd_analyze)

    args = ap.parse_args(argv)
    if not args.command:
        args = ap.parse_args(["gui"] + (argv if argv is not None else sys.argv[1:]))
    if getattr(args, "cfg", None) is None and args.command in ("gui", "run"):
        args.cfg = None if args.command == "gui" else str(DEFAULT_CFG)
    try:
        return args.func(args)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

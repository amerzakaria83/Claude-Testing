import math
import time
from pathlib import Path

import numpy as np
import pytest

from awr1843.config import load_cfg_file, parse_cfg
from awr1843.tlv import parse_frame
from awr1843_auto.cabin import CabinGuard, default_seats
from awr1843_auto.drive import DriveAssist, DriveSettings, EgoMotionEstimator, wrap
from awr1843_auto.park import ParkAssist
from awr1843_auto.simulator import AutoSimulatedRadar, scene_for, simulated_auto_device

CFG = Path(__file__).resolve().parents[1] / "awr1843_auto" / "configs"


def params(name):
    return parse_cfg(load_cfg_file(CFG / name))


@pytest.mark.parametrize("name", ["drive_long_range.cfg", "park_short_range.cfg",
                                  "cabin_occupancy.cfg"])
def test_configs_respect_front_end_limits(name):
    p = params(name)
    assert not p.warnings, p.warnings
    assert p.sample_rate_ksps >= 2000
    assert p.adc_start_time_us + p.num_adc_samples / (p.sample_rate_ksps / 1000) <= p.ramp_end_time_us
    assert p.start_freq_ghz + p.freq_slope_mhz_us * p.ramp_end_time_us / 1000 <= 81
    assert p.data_port_load() < 0.75


def test_drive_profile():
    p = params("drive_long_range.cfg")
    assert p.max_range_m > 80 and p.num_tx == 1
    assert p.max_velocity_ms == pytest.approx(15.66, abs=0.05)
    assert scene_for(p) == "drive"
    assert scene_for(params("park_short_range.cfg")) == "park"
    assert scene_for(params("cabin_occupancy.cfg")) == "cabin"


@pytest.mark.parametrize("speed", [0.0, 8.0, 15.0, 22.0, 31.0])
def test_ego_speed_with_doppler_aliasing(speed):
    """Stationary points seen from a car moving faster than the radar's
    unambiguous velocity must still give the right speed."""
    rng = np.random.default_rng(1)
    vmax = 15.66
    est = EgoMotionEstimator(vmax, DriveSettings())
    v = speed
    for _ in range(30):
        n = 40
        y = rng.uniform(3, 80, n)
        x = rng.choice([-9.0, 5.6], n) + rng.normal(0, 0.3, n)
        r = np.hypot(x, y)
        vm = wrap(-v * y / r + rng.normal(0, 0.08, n), vmax)
        st, static = est.update(x, y, vm, 0.05)
        v += 0.5 * 0.05     # gently accelerating
    assert st.valid and st.speed == pytest.approx(v, abs=0.3)
    assert static.mean() > 0.9


def _run_drive(seed, seconds=130):
    p = params("drive_long_range.cfg")
    sim = AutoSimulatedRadar(seed=seed, realtime=False)
    da = DriveAssist(p)
    dt = p.frame_period_ms / 1000
    speed_err, alerts = [], {}
    for _ in range(int(seconds / dt)):
        f = parse_frame(sim.make_frame(p), num_range_bins=p.num_range_bins)
        sim._sim_time += dt
        r = da.process(f)
        t = sim.truth["time_s"]
        if t > 2:
            speed_err.append(abs(r.ego.speed - sim.truth["ego_speed"]))
        for a in r.alerts:
            alerts.setdefault(a.code.rstrip("0123456789"), []).append(t)
    return np.array(speed_err), alerts, sim


@pytest.mark.parametrize("seed", [0, 4])
def test_drive_scenario_alerts_match_events(seed):
    err, alerts, sim = _run_drive(seed)
    assert err.mean() < 0.2 and np.percentile(err, 95) < 0.5     # m/s
    assert sim.drive.min_gap > 1.0                               # no crash in the script
    expected = {                     # code -> window (s) of the scripted event
        "cutin": (35, 43), "lead_braking": (69.5, 73), "fcw": [(70.5, 76), (103, 110)],
        "wrongway": (86, 92), "cross": (103, 110),
    }
    for code, win in expected.items():
        wins = win if isinstance(win, list) else [win]
        times = alerts.get(code, [])
        for lo, hi in wins:
            assert any(lo <= t <= hi for t in times), f"{code} missing in {lo}-{hi}"
        stray = [t for t in times if not any(lo - 1 <= t <= hi + 1 for lo, hi in wins)]
        assert not stray, f"{code} false alerts at {stray[:5]}"


def test_park_distances_and_beeper():
    p = params("park_short_range.cfg")
    sim = AutoSimulatedRadar(seed=2, realtime=False)
    pa = ParkAssist(p)
    dt = p.frame_period_ms / 1000
    seen = []
    for _ in range(int(20 / dt)):
        f = parse_frame(sim.make_frame(p), num_range_bins=p.num_range_bins)
        sim._sim_time += dt
        r = pa.process(f)
        seen.append((sim.truth["wall_distance"], r.distances["centre"], r.zone, r.beep_interval_s))
    for wall, centre, zone, beep in seen[5:]:
        assert centre == pytest.approx(wall, abs=0.15)
    zones = [z for *_, z, _ in seen]
    assert "green" in zones and "amber" in zones and "red" in zones and "stop" in zones
    assert seen[-1][3] == 0.0                     # continuous tone when stopped at 25 cm
    assert all(b is None for w, c, z, b in seen[:10])  # silent while far away


def test_cabin_occupancy_and_child_alarm():
    p = params("cabin_occupancy.cfg")
    sim = AutoSimulatedRadar(seed=3, realtime=False)
    cg = CabinGuard(p, default_seats(True))
    cg.auto_arm = False
    dt = p.frame_period_ms / 1000
    step = int(round(0.5 / dt))
    snapshots = {}
    for i in range(int(110 / dt)):
        f = parse_frame(sim.make_frame(p), num_range_bins=p.num_range_bins)
        sim._sim_time += dt
        cg.add_frame(f)
        t = sim.truth["time_s"]
        if 75 <= t < 75 + dt * 1.5:
            cg.locked = True                      # driver locks the car with the child inside
        if i % step == 0:
            r = cg.update()
            snapshots[round(t)] = r
    occ = lambda r: {s.seat.key for s in r.seats if s.occupied}  # noqa: E731
    assert occ(snapshots[30]) == {"driver", "front_pass", "rear_left"}
    child = next(s for s in snapshots[30].seats if s.seat.key == "rear_left")
    assert child.breath_bpm == pytest.approx(26, abs=2.5)
    drv = snapshots[30].driver
    assert drv.breath_bpm == pytest.approx(14, abs=2)
    assert occ(snapshots[74]) == {"rear_left"}     # adults have left
    assert snapshots[74].cpd_level == 0            # not armed yet
    assert snapshots[90].cpd_level == 2            # 10+ s after locking
    assert snapshots[108].cpd_level == 3           # 30+ s: alarm
    assert "behind driver" in snapshots[108].cpd_text


def test_cabin_auto_arm_when_driver_leaves():
    p = params("cabin_occupancy.cfg")
    sim = AutoSimulatedRadar(seed=5, realtime=False)
    cg = CabinGuard(p, default_seats(True), auto_arm_s=10)
    dt = p.frame_period_ms / 1000
    armed_at = None
    for i in range(int(95 / dt)):
        f = parse_frame(sim.make_frame(p), num_range_bins=p.num_range_bins)
        sim._sim_time += dt
        cg.add_frame(f)
        if i % 8 == 0:
            r = cg.update()
            if r.armed and armed_at is None:
                armed_at = sim.truth["time_s"]
    assert armed_at is not None and 70 < armed_at < 90   # driver out at 64 s


def test_device_scene_follows_configuration():
    dev = simulated_auto_device(realtime=False, seed=7)
    got = []
    dev.add_frame_listener(got.append)
    try:
        assert all(r.ok for r in dev.configure(load_cfg_file(CFG / "drive_long_range.cfg")))
        deadline = time.time() + 10
        while len(got) < 100 and time.time() < deadline:
            time.sleep(0.02)
        dev.stop()
        assert dev.simulator.scene == "drive" and len(got) >= 100
        assert any(len(f.points) for f in got)
        got.clear()
        assert all(r.ok for r in dev.configure(load_cfg_file(CFG / "cabin_occupancy.cfg")))
        deadline = time.time() + 10
        while len(got) < 20 and time.time() < deadline:
            time.sleep(0.02)
        assert dev.simulator.scene == "cabin"
        assert got[-1].azimuth_static is not None
    finally:
        dev.close()

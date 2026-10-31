from pathlib import Path

import numpy as np
import pytest

from awr1843.config import load_cfg_file, parse_cfg
from awr1843.recorder import Recorder, read_raw_frames
from awr1843.tlv import parse_frame
from awr1843_vitals.processing import (VitalSignsProcessor, VitalsSettings, fit_circle,
                                       spectral_peak)
from awr1843_vitals.simulator import VitalsSimulatedRadar, simulated_vitals_device

CFG_DIR = Path(__file__).resolve().parents[1] / "awr1843_vitals" / "configs"


def params(name="vitals_15fps.cfg"):
    return parse_cfg(load_cfg_file(CFG_DIR / name))


def run_sim(p, seconds, update_every_s=1.0, **sim_kwargs):
    """Feed simulated frames through the processor; yield (truth, result)."""
    sim = VitalsSimulatedRadar(realtime=False, **sim_kwargs)
    proc = VitalSignsProcessor(p)
    dt = p.frame_period_ms / 1000
    step = max(1, int(round(update_every_s / dt)))
    for i in range(int(seconds / dt)):
        proc.add_frame(parse_frame(sim.make_frame(p), num_range_bins=p.num_range_bins))
        truth = dict(sim.truth)
        sim._sim_time += dt
        if i % step == 0:
            yield truth, proc.update()


@pytest.mark.parametrize("name", ["vitals_15fps.cfg", "vitals_safe_8fps.cfg"])
def test_configs(name):
    p = params(name)
    assert not p.warnings, p.warnings
    assert p.gui.range_azimuth_heatmap and not p.clutter_removal
    assert p.max_range_m > 2.0


def test_15fps_profile_numbers():
    p = params()
    assert p.frame_rate_hz == pytest.approx(15, abs=0.01) and p.num_range_bins == 128
    assert p.num_virtual_antennas_azimuth == 8
    assert p.range_resolution_m == pytest.approx(0.0508, abs=0.002)
    # mmWave front-end limits: ADC rate >= 2000 ksps, sampling fits in the ramp,
    # sweep stays inside 77-81 GHz
    assert p.sample_rate_ksps >= 2000
    assert p.adc_start_time_us + p.num_adc_samples / (p.sample_rate_ksps / 1000) <= p.ramp_end_time_us
    assert p.start_freq_ghz + p.freq_slope_mhz_us * p.ramp_end_time_us / 1000 <= 81
    assert p.data_port_load() < 0.75


def test_fit_circle_recovers_centre():
    rng = np.random.default_rng(0)
    ang = np.linspace(0, 1.5, 200)  # an arc, not a full circle
    z = (3 + 2j) + 5 * np.exp(1j * ang) + rng.normal(0, 0.01, 200)
    assert abs(fit_circle(z) - (3 + 2j)) < 0.1


def test_spectral_peak_and_harmonic_suppression():
    fs = 20
    t = np.arange(0, 20, 1 / fs)
    breath = 0.3
    x = 1.5 * np.sin(2 * np.pi * 3 * breath * t) + np.sin(2 * np.pi * 1.3 * t)  # harmonic > heart
    f, *_ = spectral_peak(x, fs, (0.8, 2.5))
    assert f == pytest.approx(0.9, abs=0.03)          # without suppression: the harmonic wins
    f, *_ = spectral_peak(x, fs, (0.8, 2.5), suppress_f0=breath)
    assert f == pytest.approx(1.3, abs=0.03)


# 8 fps can only follow calm breathing (faster/deeper breathing aliases the phase)
@pytest.mark.parametrize("name,seed,breath", [("vitals_15fps.cfg", 1, (12, 18)),
                                              ("vitals_safe_8fps.cfg", 2, (10, 14))])
def test_rates_match_ground_truth(name, seed, breath):
    eb, eh = [], []
    for truth, r in run_sim(params(name), 120, events=False, seed=seed, breath_bpm=breath):
        if truth["time_s"] < 20:
            continue
        assert r.present and r.distance_m == pytest.approx(0.8, abs=0.1)
        assert r.angle_deg == pytest.approx(10, abs=8)
        assert r.breath_bpm is not None and r.heart_bpm is not None
        eb.append(abs(r.breath_bpm - truth["breath_bpm"]))
        eh.append(abs(r.heart_bpm - truth["heart_bpm"]))
    assert np.mean(eb) < 1.5
    assert np.mean(eh) < 4 and np.mean(np.array(eh) < 6) > 0.85


def test_other_position():
    results = [r for t, r in run_sim(params(), 40, events=False, seed=3,
                                     distance_m=1.2, angle_deg=-25)]
    r = results[-1]
    assert r.distance_m == pytest.approx(1.2, abs=0.1)
    assert r.angle_deg == pytest.approx(-25, abs=8)
    assert r.breath_bpm and r.heart_bpm


def test_motion_and_apnea_detected():
    states = {}
    for truth, r in run_sim(params(), 130, events=True, seed=4):
        tc = truth["time_s"]
        if 56 <= tc <= 59:
            states.setdefault("motion", []).append(r.motion)
        if 40 <= tc <= 54:
            states.setdefault("before", []).append(r.motion)
        if 120 <= tc <= 124:   # breath held from 110 s
            states.setdefault("paused", []).append(r.apnea_s >= 6 and r.breath_bpm is None)
        if 118 <= tc <= 127:
            states.setdefault("apnea", []).append(r.apnea)
    assert all(states["motion"])
    assert not any(states["before"])
    assert all(states["paused"])
    assert any(states["apnea"])


def test_empty_room():
    results = [r for _, r in run_sim(params(), 30, events=False, person=False, seed=5)]
    assert not any(r.present for r in results[5:])
    assert all(r.heart_bpm is None for r in results)


def test_missing_heatmap_is_reported():
    p = params()
    proc = VitalSignsProcessor(p)
    proc.add_frame(parse_frame(VitalsSimulatedRadar(realtime=False).make_frame(
        parse_cfg(load_cfg_file(CFG_DIR.parent.parent / "awr1843" / "configs"
                                / "awr1843_default.cfg")))))
    assert "heatmap" in proc.update().status


def test_heart_tracker_rejects_octave_jumps():
    proc = VitalSignsProcessor(params(), VitalsSettings())
    for _ in range(5):
        proc._track_heart(70)
    assert proc._track_heart(140) == pytest.approx(70, abs=1)   # octave error corrected
    assert proc._track_heart(100) == pytest.approx(70, abs=1)   # single jump ignored
    for _ in range(4):
        out = proc._track_heart(100)
    assert out == pytest.approx(100, abs=1)                     # persistent change accepted


def test_device_record_and_reanalyse(tmp_path):
    """Full path: CLI config -> serial frames -> recording -> offline analysis."""
    dev = simulated_vitals_device(realtime=False, events=False, seed=6)
    lines = load_cfg_file(CFG_DIR / "vitals_15fps.cfg")
    rec = Recorder(tmp_path, lines, save_heatmaps=False)
    rec.attach(dev)
    got = []
    dev.add_frame_listener(got.append)
    try:
        assert all(r.ok for r in dev.configure(lines))
        import time
        deadline = time.time() + 20
        while len(got) < 400 and time.time() < deadline:
            time.sleep(0.05)
    finally:
        dev.stop()
        rec.detach(dev)
        rec.close()
        dev.close()
    assert len(got) >= 400
    proc = VitalSignsProcessor(parse_cfg(lines))
    for f in read_raw_frames(tmp_path / "raw_data.bin", 128):
        proc.add_frame(f)
    r = proc.update()
    assert r.present and r.breath_bpm and r.heart_bpm

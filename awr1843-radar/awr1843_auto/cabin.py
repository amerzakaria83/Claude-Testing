"""Cabin Guard: seat occupancy, child-left-behind alarm and driver vitals.

The radar sits at the rear-view mirror / roof console looking into the cabin
and sends the range-azimuth heatmap (same data as the Vital Signs Monitor).
Every seat is a (range, angle) zone with its own vital-signs processor
(sharing one frame buffer).  A seat is *occupied* when its zone shows a
breathing or moving person.  Sidelobe leakage of a neighbour into an empty
seat is rejected: a weak zone whose chest signal is a copy (correlation) of a
much stronger zone's signal is not counted.

Child presence detection (as required by Euro NCAP from 2025): when the car
is locked - or automatically once the driver's seat has been empty for a
while - any occupied seat escalates from a notice to a warning to an alarm.
"""

from __future__ import annotations

import copy
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from awr1843.config import RadarParams
from awr1843.tlv import Frame
from awr1843_vitals.processing import VitalSignsProcessor, VitalsResult, VitalsSettings


@dataclass
class Seat:
    key: str
    label: str
    range_min: float
    range_max: float
    angle_min: float
    angle_max: float
    driver: bool = False


def default_seats(driver_on_left: bool = True) -> list[Seat]:
    """Sedan, radar at the mirror looking rearwards.  Facing the rear, the
    car's left side appears on the radar's right (+x / positive angles)."""
    sgn = 1 if driver_on_left else -1

    def zone(a0, a1):
        lo, hi = sorted((sgn * a0, sgn * a1))
        return lo, hi

    seats = [
        Seat("driver", "Driver", 0.30, 1.05, *zone(14, 65), driver=True),
        Seat("front_pass", "Front passenger", 0.30, 1.05, *zone(-65, -14)),
        Seat("rear_left" if driver_on_left else "rear_right",
             "Rear, behind driver", 1.10, 2.10, *zone(12, 45)),
        Seat("rear_mid", "Rear middle", 1.10, 2.10, -11, 11),
        Seat("rear_right" if driver_on_left else "rear_left",
             "Rear, behind passenger", 1.10, 2.10, *zone(-45, -12)),
    ]
    return seats


@dataclass
class SeatState:
    seat: Seat
    occupied: bool = False
    present_ratio: float = 0.0
    breath_bpm: float | None = None
    heart_bpm: float | None = None
    motion: bool = False
    leakage_of: str | None = None
    occupied_for_s: float = 0.0
    empty_for_s: float = 0.0
    result: VitalsResult | None = None


@dataclass
class CabinResult:
    time_s: float = 0.0
    seats: list[SeatState] = field(default_factory=list)
    occupants: int = 0
    armed: bool = False
    armed_reason: str = ""
    cpd_level: int = 0                    # 0 none, 1 notice, 2 warning, 3 alarm
    cpd_text: str = ""
    driver: SeatState | None = None
    driver_alertness: str = ""
    status: str = ""


class CabinGuard:
    def __init__(self, params: RadarParams, seats: list[Seat] | None = None,
                 auto_arm_s: float = 15.0):
        self.params = params
        self.fs = params.frame_rate_hz
        self.auto_arm_s = auto_arm_s
        self.locked = False                 # set by the user / the car
        self.auto_arm = True
        self.set_seats(seats or default_seats())

    def set_seats(self, seats: list[Seat]) -> None:
        self.seats = seats
        self.procs: dict[str, VitalSignsProcessor] = {}
        for seat in seats:
            vs = VitalsSettings(range_min_m=seat.range_min, range_max_m=seat.range_max,
                                angle_min_deg=seat.angle_min, angle_max_deg=seat.angle_max,
                                min_breath_s=8.0, min_heart_s=8.0, presence_ratio=8.0,
                                history_s=24.0)
            self.procs[seat.key] = VitalSignsProcessor(self.params, vs)
        self.states = {s.key: SeatState(s) for s in seats}
        self._votes = {s.key: deque(maxlen=5) for s in seats}
        self._driver_seen = False
        self._driver_left_for = 0.0
        self._armed_for = 0.0
        self._last_t: float | None = None
        self._baseline_br: deque[float] = deque(maxlen=240)

    def reset(self) -> None:
        self.set_seats(self.seats)

    def add_frame(self, frame: Frame) -> None:
        for proc in self.procs.values():
            proc.add_frame(frame)

    def update(self) -> CabinResult:
        res = CabinResult()
        procs = list(self.procs.values())
        first = procs[0]
        if first.missing_heatmap and not first._frames:
            res.status = "No range-azimuth heatmap - select cabin_occupancy.cfg"
            return res
        if len(first._frames) < int(3 * self.fs):
            res.status = "Collecting data..."
            return res
        cube = np.stack(first._frames)
        t_now = first._fnums[-1] / self.fs
        dt = 0.0 if self._last_t is None else max(0.0, t_now - self._last_t)
        self._last_t = t_now
        res.time_s = t_now

        results = {k: p.update(cube=cube) for k, p in self.procs.items()}
        # ---- leakage rejection between zones
        strength = {}
        for k, r in results.items():
            st = self.states[k]
            st.result = r
            st.present_ratio = r.signal_quality
            strength[k] = r.signal_quality if r.present else 0.0
        leak = {}
        for k, r in results.items():
            if not strength[k] or len(r.displacement_mm) < 30:
                continue
            for k2, r2 in results.items():
                if k2 == k or strength[k2] < 6 * strength[k] or len(r2.displacement_mm) != len(r.displacement_mm):
                    continue
                a = r.displacement_mm[-int(8 * self.fs):]
                b = r2.displacement_mm[-int(8 * self.fs):]
                if np.std(a) > 0 and np.std(b) > 0 and abs(np.corrcoef(a, b)[0, 1]) > 0.8:
                    leak[k] = k2
                    break

        # ---- occupancy with hysteresis
        for k, r in results.items():
            st = self.states[k]
            st.leakage_of = leak.get(k)
            breathing = r.breath_bpm is not None and r.breath_confidence >= 0.3
            vote = r.present and st.leakage_of is None and (breathing or r.motion
                                                             or r.warmup_left_s > 0)
            self._votes[k].append(bool(vote))
            votes = sum(self._votes[k])
            if not st.occupied and votes >= 3:
                st.occupied = True
                st.occupied_for_s = 0.0
            elif st.occupied and not any(list(self._votes[k])[-3:]):
                st.occupied = False
                st.empty_for_s = 0.0
            if st.occupied:
                st.occupied_for_s += dt
            else:
                st.empty_for_s += dt
            st.breath_bpm = r.breath_bpm if st.occupied else None
            st.heart_bpm = r.heart_bpm if st.occupied else None
            st.motion = r.motion and st.occupied
        # copies: a result must not change when later updates modify the state
        res.seats = [copy.copy(self.states[s.key]) for s in self.seats]
        res.occupants = sum(st.occupied for st in res.seats)

        # ---- driver / arming
        drv = next((st for st in res.seats if st.seat.driver), None)
        res.driver = drv
        if drv is not None:
            if drv.occupied:
                self._driver_seen = True
                self._driver_left_for = 0.0
                if drv.breath_bpm:
                    self._baseline_br.append(drv.breath_bpm)
            elif self._driver_seen:
                self._driver_left_for += dt
        auto = self.auto_arm and self._driver_seen and self._driver_left_for >= self.auto_arm_s
        res.armed = self.locked or auto
        res.armed_reason = "car locked" if self.locked else ("driver left" if auto else "")
        self._armed_for = self._armed_for + dt if res.armed else 0.0

        # ---- child presence detection
        others = [st for st in res.seats if st.occupied and not (st.seat.driver and not self.locked)]
        if res.armed and others:
            longest = max(st.occupied_for_s for st in others)
            t_alarm = min(longest, self._armed_for)
            names = ", ".join(st.seat.label.lower() for st in others)
            if t_alarm >= 30:
                res.cpd_level = 3
                res.cpd_text = f"ALARM: occupant left in the car ({names})!"
            elif t_alarm >= 10:
                res.cpd_level = 2
                where = "the locked car" if self.locked else "the car after the driver left"
                res.cpd_text = f"Occupant detected in {where}: {names}"
            else:
                res.cpd_level = 1
                res.cpd_text = f"Checking cabin... occupant in {names}"

        # ---- driver alertness (experimental): breathing much slower than usual
        if drv is not None and drv.occupied and drv.breath_bpm and len(self._baseline_br) > 40:
            base = float(np.median(list(self._baseline_br)[:120]))
            if drv.breath_bpm < 0.75 * base:
                res.driver_alertness = f"Breathing slowed to {drv.breath_bpm:.0f}/min " \
                                       f"(usual {base:.0f}) - take a break?"
            else:
                res.driver_alertness = "Normal"
        res.status = f"{res.occupants} occupant{'s' if res.occupants != 1 else ''}"
        return res

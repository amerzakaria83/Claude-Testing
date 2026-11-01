"""Simulated board for the Car Radar Suite.

The scene is chosen from the configuration the app sends:

* heatmap enabled (cabin_occupancy.cfg)  -> **cabin**: driver, front passenger
  and a child; the adults get out, the child stays (180 s cycle)
* long range (drive_long_range.cfg)      -> **drive**: scripted 130 s drive
  with set-off, cut-in, hard braking ahead while distracted, an overtaker in
  our lane and a pedestrian crossing
* short range (park_short_range.cfg)     -> **park**: reversing towards a
  wall with a pole and a parked car (35 s cycle)

Detections are generated with range-dependent SNR, measurement noise and the
radar's velocity wrapping (so the app's Doppler unfolding is exercised).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from awr1843.config import DATA_PORT_BYTES_PER_SEC, SPEED_OF_LIGHT, RadarParams
from awr1843.device import RadarDevice
from awr1843.simulator import SimulatedRadar
from awr1843.tlv import encode_frame


def scene_for(p: RadarParams) -> str:
    if p.gui.range_azimuth_heatmap:
        return "cabin"
    return "drive" if p.max_range_m > 30 else "park"


# ===================================================================== drive
@dataclass
class Actor:
    id: int
    kind: str          # car / truck / ped
    x: float           # lateral position (m, right +)
    s: float           # longitudinal position of the centre (m)
    v: float           # longitudinal ground speed (m/s)
    vx: float = 0.0
    length: float = 4.5
    width: float = 1.8
    rcs: float = 12.0
    role: str = ""


class DriveWorld:
    LANE = 3.5
    CYCLE = 130.0

    def __init__(self, rng: np.random.Generator):
        self.rng = rng
        self.reset()

    def reset(self) -> None:
        self.t = 0.0
        self.ego_s, self.ego_v, self.ego_a = 0.0, 0.0, 0.0
        self.lead = Actor(1, "car", 0.0, 12.0, 0.0, role="lead")
        self.actors: list[Actor] = [self.lead]
        self._next_id = 10
        self._next_oncoming = 2.0
        self.event = ""
        self.cut_state = "none"
        self.min_gap = math.inf

    # ---- scripted behaviour -------------------------------------------
    def _lead_accel(self, a: Actor) -> float:
        t = self.t
        if t < 4:
            return 0.0
        if t < 46:
            return float(np.clip((15.0 - a.v) * 0.8, -2, 1.5))
        if t < 70:                                   # slower: we catch up again
            return float(np.clip((11.0 - a.v) * 0.8, -2, 1.5))
        if t < 73:
            return -6.0 if a.v > 0 else 0.0          # emergency stop
        if t < 80:
            return 0.0
        return float(np.clip((13.0 - a.v) * 0.8, -2, 1.5))

    def _obstacle_ahead(self):
        """Nearest thing in the ego lane for the ego's car-following model."""
        best = None
        for a in self.actors:
            if abs(a.x) < 2.3 and a.s - a.length / 2 > self.ego_s:
                gap = a.s - a.length / 2 - self.ego_s
                if best is None or gap < best[0]:
                    best = (gap, a.v)
        return best

    def _ego_accel(self) -> float:
        v0, T, s0, amax, b = 17.0, 1.5, 3.0, 1.5, 2.0
        v = self.ego_v
        acc = amax * (1 - (v / v0) ** 4)
        ob = self._obstacle_ahead()
        if ob is not None:
            gap, vl = ob
            dv = v - vl
            s_star = s0 + max(0.0, v * T + v * dv / (2 * math.sqrt(amax * b)))
            acc -= amax * (s_star / max(gap, 0.3)) ** 2
        return float(np.clip(acc, -9.0, amax))

    def step(self, dt: float) -> None:
        self.t += dt
        t = self.t
        if t >= self.CYCLE:
            self.reset()
            return
        self.event = ""
        # ego: car-following model, "distracted" while the lead starts braking
        if 70.0 <= t < 71.4:
            self.event = "driver distracted"
        elif t < 4.0:
            self.ego_a = 0.0                     # waiting behind the stopped lead
        else:
            self.ego_a = self._ego_accel()
        self.ego_v = max(0.0, self.ego_v + self.ego_a * dt)
        self.ego_s += self.ego_v * dt

        lead = self.lead
        if lead in self.actors:
            lead.v = max(0.0, lead.v + self._lead_accel(lead) * dt)
            if 70 <= t < 73:
                self.event = self.event or "lead brakes hard"
            if 95 <= t < 98:                       # lead turns off to the right
                lead.vx = 1.6
            elif t >= 98 and lead.x >= 3.5:
                lead.vx, lead.v = 0.0, lead.v + 1.0 * dt

        # cut-in from the right lane
        if 30.0 <= t < 30.0 + dt * 1.5 and self.cut_state == "none":
            self.actors.append(Actor(2, "car", self.LANE, self.ego_s - 20, self.ego_v + 5,
                                     role="cutin"))
            self.cut_state = "approach"
        for a in self.actors:
            if a.role == "cutin":
                rear_gap = a.s - a.length / 2 - self.ego_s
                if self.cut_state == "approach":
                    a.v = self.ego_v + 4.0
                    if rear_gap > 9:
                        self.cut_state = "cutting"
                if self.cut_state == "cutting":
                    self.event = self.event or "vehicle cutting in"
                    a.v = self.ego_v + 1.0
                    a.vx = -1.3
                    if a.x <= 0:
                        a.x, a.vx, self.cut_state = 0.0, 0.0, "in_lane"
                        self._cut_t = t
                if self.cut_state == "in_lane":
                    a.v = self.ego_v + 1.0
                    if t - self._cut_t > 6:
                        self.cut_state = "leaving"
                if self.cut_state == "leaving":
                    a.vx = 1.3 if a.x < self.LANE else 0.0
                    a.v = min(a.v + 1.5 * dt, 22)

        # oncoming traffic, and an overtaker swinging into our lane
        if t >= self._next_oncoming:
            self._next_oncoming = t + self.rng.uniform(5, 9)
            self.actors.append(Actor(self._next_id, "car", -self.LANE, self.ego_s + 105,
                                     -float(self.rng.uniform(11, 15)), role="oncoming"))
            self._next_id += 1
        if 86.0 <= t < 86.0 + dt * 1.5:
            self.actors.append(Actor(3, "car", -self.LANE, self.ego_s + 100, -14.0,
                                     role="overtaker"))
        for a in self.actors:
            if a.role == "overtaker":
                d = a.s - self.ego_s
                if d > 45:
                    a.vx = 3.0 if a.x < -0.3 else 0.0
                else:
                    a.vx = -2.5 if a.x > -self.LANE else 0.0
                if abs(a.x) < 1.75 and d > 0:
                    self.event = self.event or "overtaker in our lane"

        # pedestrian crossing
        if 103.0 <= t < 103.0 + dt * 1.5:
            self.actors.append(Actor(4, "ped", 5.0, self.ego_s + 60, 0.0, vx=-1.4,
                                     length=0.5, width=0.6, rcs=-3.0, role="ped"))
        for a in self.actors:
            if a.role == "ped" and abs(a.x) < 2.5:
                self.event = self.event or "pedestrian crossing"

        for a in self.actors:
            a.s += a.v * dt
            a.x += a.vx * dt
        self.actors = [a for a in self.actors
                       if -25 < a.s - self.ego_s < 160 and a.x > -12 and
                       not (a.role == "ped" and a.x < -9)]
        ob = self._obstacle_ahead()
        if ob is not None:
            self.min_gap = min(self.min_gap, ob[0])

    # ---- radar view -----------------------------------------------------
    def scatterers(self):
        """(x, y, vx_rel, vy_rel, rcs) of everything in front of the radar."""
        out = []
        es = self.ego_s
        for k in range(int((es - 5) // 4), int((es + 100) // 4) + 1):       # right posts
            out.append((5.6, k * 4 - es, 0.0, -self.ego_v, 2.0))
        for k in range(int((es - 5) // 8), int((es + 100) // 8) + 1):       # left posts
            out.append((-9.0, k * 8 - es, 0.0, -self.ego_v, 2.0))
        for k in range(int((es - 5) // 15), int((es + 100) // 15) + 1):     # trees / signs
            h = (k * 2654435761) % 1000 / 1000
            out.append((8.0 + 3.0 * h, k * 15 + 6 * h - es, 0.0, -self.ego_v, 6.0))
        for a in self.actors:
            near = a.s - a.length / 2 - es
            n = 1 if a.kind == "ped" else int(self.rng.integers(2, 5))
            for i in range(n):
                off = 0 if n == 1 else (i / (n - 1) - 0.5) * a.width * 0.85
                out.append((a.x + off, near + self.rng.uniform(0, 0.4),
                            a.vx, a.v - self.ego_v, a.rcs))
        return out

    def truth(self) -> dict:
        ob = self._obstacle_ahead()
        return {"time_s": self.t, "ego_speed": self.ego_v, "event": self.event,
                "lead_distance": ob[0] if ob else None,
                "actors": [(a.role, a.x, a.s - self.ego_s) for a in self.actors]}


# ====================================================================== park
class ParkWorld:
    CYCLE = 35.0

    def __init__(self, rng):
        self.rng = rng
        self.t = 0.0

    def distance(self, t: float) -> tuple[float, float]:
        """Distance to the wall and closing speed at time t (cycle)."""
        c = t % self.CYCLE
        if c < 3:
            return 3.5, 0.0
        if c < 15:
            return 3.5 - 0.25 * (c - 3), 0.25
        if c < 17.5:
            return 0.5 - 0.1 * (c - 15), 0.1
        if c < 22:
            return 0.25, 0.0
        if c < 33:
            return 0.25 + 0.3 * (c - 22), -0.3
        return 3.55, 0.0

    def scatterers(self):
        d, u = self.distance(self.t)
        out = []
        for xw in np.arange(-3.0, 3.01, 0.3):                  # wall
            out.append((xw, d, -u, 6.0))
        if d - 0.9 > 0:
            out.append((-1.25, d - 0.9, -u, 4.0))               # pole in front of the wall
        for yc in np.arange(d - 1.2, d + 0.01, 0.3):            # parked car's corner
            if yc > 0.05:
                out.append((1.45, yc, -u, 8.0))
        return out, d, u

    def step(self, dt):
        self.t += dt


# ===================================================================== cabin
@dataclass
class Occupant:
    key: str
    r: float
    angle: float
    breath_bpm: float
    depth_mm: float
    heart_bpm: float
    leave_at: float | None   # seconds into the cycle when they get out (None = stays)
    amp: float
    phase_b: float = 0.0
    phase_h: float = 0.0


class CabinWorld:
    CYCLE = 180.0

    def __init__(self, rng, driver_on_left: bool = True):
        self.rng = rng
        sgn = 1 if driver_on_left else -1
        self.occupants = [
            Occupant("driver", 0.62, 35 * sgn, 14, 5.0, 68, 60.0, 2600),
            Occupant("front_pass", 0.62, -35 * sgn, 16, 4.5, 76, 40.0, 2400),
            Occupant("rear_left" if driver_on_left else "rear_right", 1.45, 25 * sgn,
                     26, 2.0, 112, None, 1500),
        ]
        self.static = [(0.65, 35, 900), (0.65, -35, 900), (1.6, 25, 1100), (1.6, 0, 1100),
                       (1.6, -25, 1100), (2.3, 0, 2500), (1.1, 70, 1400), (1.1, -70, 1400)]
        self.t = 0.0

    def present(self, o: Occupant, c: float) -> str:
        if o.leave_at is None or c < o.leave_at:
            return "in"
        if c < o.leave_at + 4:
            return "leaving"
        return "out"

    def step(self, dt):
        self.t += dt
        for o in self.occupants:
            o.phase_b += 2 * math.pi * o.breath_bpm / 60 * dt
            o.phase_h += 2 * math.pi * o.heart_bpm / 60 * dt * (1 + 0.03 * self.rng.standard_normal())

    def truth(self) -> dict:
        c = self.t % self.CYCLE
        return {"time_s": self.t, "cycle_s": c,
                "occupants": {o.key: self.present(o, c) for o in self.occupants}}


# ================================================================ the board
class AutoSimulatedRadar(SimulatedRadar):
    def __init__(self, seed: int | None = None, realtime: bool = True):
        super().__init__(seed=seed, realtime=realtime)
        self.drive = DriveWorld(self._rng)
        self.park = ParkWorld(self._rng)
        self.cabin = CabinWorld(self._rng)
        self.scene = ""
        self.truth: dict = {}

    def make_frame(self, p: RadarParams) -> bytes:
        scene = scene_for(p)
        if scene != self.scene:
            self.scene = scene
        dt = p.frame_period_ms / 1000
        if scene == "drive":
            self.drive.step(dt)
            args = self._drive_points(p)
            self.truth = {"scene": scene, **self.drive.truth()}
        elif scene == "park":
            self.park.step(dt)
            args = self._park_points(p)
        else:
            self.cabin.step(dt)
            args = self._cabin_heatmap(p)
            self.truth = {"scene": scene, **self.cabin.truth()}
        return self._encode(p, args)

    # ---- helpers ---------------------------------------------------------
    def _encode(self, p, args) -> bytes:
        g = p.gui
        n_bytes = len(encode_frame(self._frame_number, **args))
        if g.stats:
            tx_us = n_bytes / DATA_PORT_BYTES_PER_SEC * 1e6
            args["stats"] = (1200, int(tx_us), int(max(0, p.frame_period_ms * 1000 - tx_us - 5000)),
                             20, int(25 + self._rng.normal(0, 2)), int(15 + self._rng.normal(0, 2)))
            args["temperature"] = (0, int(self._sim_time * 1000), *([45] * 10))
        self._frame_number += 1
        return encode_frame(self._frame_number, **args)

    def _points_args(self, p, pts):
        if not pts:
            return {"xyzv": None}
        arr = np.array(pts, dtype=np.float32)
        args = {"xyzv": arr[:, :4]}
        if p.gui.detected_objects == 1:
            args["snr_noise_db"] = arr[:, 4:6].astype(float)
        return args

    def _detect(self, p, x, y, vx, vy, rcs, k_snr=82.0, z_sigma=0.0, ang_sigma=1.2,
                fov=60.0):
        rng = self._rng
        r = math.hypot(x, y)
        if r < 0.08 or r > p.max_range_m or y <= 0:
            return None
        az = math.degrees(math.atan2(x, y))
        if abs(az) > fov:
            return None
        snr = rcs + k_snr - 40 * math.log10(max(r, 0.5)) + rng.normal(0, 2.5)
        if snr < 13:
            return None
        vr = (x * vx + y * vy) / r + rng.normal(0, 0.08)
        vmax = p.max_velocity_ms
        vr = (vr + vmax) % (2 * vmax) - vmax
        rm = r + rng.normal(0, max(0.02, p.range_resolution_m / 4))
        azm = math.radians(az + rng.normal(0, ang_sigma))
        return (rm * math.sin(azm), rm * math.cos(azm), rng.normal(0, z_sigma) if z_sigma else 0.0,
                vr, min(snr, 45.0), 52.0)

    def _drive_points(self, p):
        pts = []
        for (x, y, vx, vy, rcs) in self.drive.scatterers():
            d = self._detect(p, x, y, vx, vy, rcs)
            if d:
                pts.append(d)
        pts.sort(key=lambda q: q[1])
        return self._points_args(p, pts[:220])

    def _park_points(self, p):
        objs, d, u = self.park.scatterers()
        self.truth = {"scene": "park", "time_s": self.park.t, "wall_distance": d, "closing": u}
        pts = []
        for (x, y, vy, rcs) in objs:
            det = self._detect(p, x, y, 0.0, vy, rcs, k_snr=40.0, z_sigma=0.15, ang_sigma=1.0,
                               fov=70)
            if det:
                pts.append(det)
        return self._points_args(p, pts)

    def _cabin_heatmap(self, p):
        rng = self._rng
        lam = SPEED_OF_LIGHT / (p.start_freq_ghz * 1e9)
        n_rb, n_ant = p.num_range_bins, p.num_virtual_antennas_azimuth
        res = p.range_resolution_m
        ant = np.arange(n_ant)
        az = rng.normal(0, 25, (n_rb, n_ant)) + 1j * rng.normal(0, 25, (n_rb, n_ant))

        def add(r, ang, amp, phase):
            steer = np.exp(1j * math.pi * ant * math.sin(math.radians(ang)))
            pos = r / res
            for k in range(int(pos) - 2, int(pos) + 3):
                if 0 <= k < n_rb:
                    az[k] += amp * np.sinc(k - pos) * np.exp(1j * phase) * steer

        for (r, ang, amp) in self.static:
            add(r, ang, amp, r * 7.0)
        c = self.cabin.t % self.cabin.CYCLE
        for o in self.cabin.occupants:
            state = self.cabin.present(o, c)
            if state == "out":
                continue
            b = o.phase_b
            disp = o.depth_mm / 2 * 1e-3 * math.sin(b + 0.3 * math.sin(b))
            disp += 0.12e-3 * (math.sin(o.phase_h) + 0.4 * math.sin(2 * o.phase_h + 0.6))
            r, ang = o.r, o.angle
            if state == "leaving":
                k = c - o.leave_at
                r += 0.15 * k / 4 + 0.03 * math.sin(2 * math.pi * 1.2 * k)
                ang += 20 * k / 4 * (1 if ang > 0 else -1)
            add(r + disp, ang, o.amp / r, 4 * math.pi * (r + disp) / lam)
        return {"azimuth_static": np.clip(az.real, -30000, 30000) + 1j * np.clip(az.imag, -30000, 30000)}

    @property
    def static(self):
        return self.cabin.static


def simulated_auto_device(**kwargs) -> RadarDevice:
    sim = AutoSimulatedRadar(**kwargs)
    dev = RadarDevice(sim.cli_port, sim.data_port, "SIM-CLI", "SIM-DATA")
    dev.simulator = sim
    return dev

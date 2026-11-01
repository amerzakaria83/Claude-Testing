"""Drive Assist: ego motion, object tracking and driver warnings from a single
forward-looking AWR1843 running the out-of-box demo (point cloud only).

Coordinates are the radar's: +y forward, +x to the right, origin at the radar.

Pipeline (``DriveAssist.process(frame)``):

1. **Ego motion ("radar odometry").**  Stationary things (posts, guard rails,
   signs, parked cars) all appear to approach with radial velocity
   ``-v_ego * cos(azimuth)``.  A consensus search over candidate speeds finds
   the speed that explains the most detections - *including* the wrapping of
   velocities beyond the radar's unambiguous range (+-15.7 m/s with the
   bundled config), because the wrap depends on each point's angle.  Gives a
   speedometer without GPS or CAN.
2. **Static / moving split** of every detection against that ego model.
3. **Clustering** of moving detections (position + Doppler) into objects.
4. **Tracking** with a constant-velocity Kalman filter per object (relative
   position/velocity), Hungarian association, Doppler unfolding against the
   track's predicted range rate, confirm/delete logic.
5. **Situation assessment**: lane assignment, lead vehicle, time gap, time to
   collision, required deceleration -> forward collision warning levels, lead
   braking, cut-ins, crossing pedestrians, oncoming traffic in our lane, plus
   stationary obstacles in our lane.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

from awr1843.config import RadarParams
from awr1843.tlv import Frame


def wrap(v, vmax):
    """Wrap velocities into the radar's unambiguous interval [-vmax, vmax)."""
    return (np.asarray(v) + vmax) % (2 * vmax) - vmax


# ------------------------------------------------------------------ settings
@dataclass
class DriveSettings:
    lane_width_m: float = 3.5
    target_gap_s: float = 2.0         # "2 second rule"
    ttc_caution_s: float = 4.0
    ttc_warning_s: float = 2.6
    ttc_critical_s: float = 1.6
    decel_warning: float = 3.5        # m/s^2 needed to avoid a collision
    decel_critical: float = 6.0
    max_speed_ms: float = 45.0
    static_tol_ms: float = 0.45       # velocity tolerance of the stationary model
    cluster_eps_m: float = 2.2
    max_range_m: float = 85.0


# ------------------------------------------------------------- ego motion
@dataclass
class EgoState:
    speed: float = 0.0                # m/s, forward ground speed of the car
    lateral: float = 0.0              # m/s, sideways slip (small)
    accel: float = 0.0                # m/s^2
    confidence: float = 0.0           # 0..1
    inliers: int = 0
    valid: bool = False


class EgoMotionEstimator:
    def __init__(self, vmax: float, settings: DriveSettings):
        self.vmax = vmax
        self.s = settings
        self.state = EgoState()
        self._lost = 0

    def update(self, x: np.ndarray, y: np.ndarray, vm: np.ndarray, dt: float,
               weight: np.ndarray | None = None) -> tuple[EgoState, np.ndarray]:
        """Returns the new state and a boolean mask of stationary detections."""
        st = self.state
        n = len(vm)
        rh = np.hypot(x, y)
        ok = (rh > 1.0) & (y > 0.3)
        static_mask = np.zeros(n, bool)
        if ok.sum() < 4:
            self._lost += 1
            st.confidence *= 0.9
            st.valid = st.confidence > 0.2
            return st, static_mask
        c, s_ = y[ok] / rh[ok], x[ok] / rh[ok]
        v = vm[ok]
        w = np.ones(len(v)) if weight is None else weight[ok]
        tol = self.s.static_tol_ms

        # search window: around the previous speed when we are confident
        if st.valid and self._lost < 10:
            lo, hi = st.speed - 3.0, st.speed + 3.0
        else:
            lo, hi = -5.0, self.s.max_speed_ms
        cand = np.arange(lo, hi + 1e-9, 0.05)
        pred = -cand[:, None] * c[None, :]
        res = wrap(v[None, :] - pred, self.vmax)
        score = ((np.abs(res) < tol) * w[None, :]).sum(axis=1)
        # small preference for continuity / low speed resolves rare ties
        prior = cand - (st.speed if st.valid else 0.0)
        score = score - 0.02 * np.abs(prior)
        best = cand[int(np.argmax(score))]

        inl = np.abs(wrap(v - (-best * c), self.vmax)) < tol
        if inl.sum() < 4 or inl.mean() < 0.15:
            self._lost += 1
            st.confidence *= 0.9
            st.valid = st.confidence > 0.2
            return st, static_mask
        # least squares refinement of forward + lateral speed on unwrapped inliers
        vu = -best * c[inl] + wrap(v[inl] - (-best * c[inl]), self.vmax)
        a = -np.column_stack([c[inl], s_[inl]])
        (vf, vl), *_ = np.linalg.lstsq(a, vu, rcond=None)
        if not np.isfinite(vf) or abs(vf - best) > 1.0:
            vf, vl = best, 0.0
        vl = float(np.clip(vl, -2.0, 2.0))

        # alpha-beta smoothing (speed + acceleration)
        if st.valid:
            pred_v = st.speed + st.accel * dt
            r = vf - pred_v
            st.speed = pred_v + 0.5 * r
            st.accel = float(np.clip(st.accel + 0.15 * r / max(dt, 1e-3), -12, 12))
        else:
            st.speed, st.accel = float(vf), 0.0
        st.lateral = 0.8 * st.lateral + 0.2 * vl
        st.inliers = int(inl.sum())
        st.confidence = float(min(1.0, 0.7 * st.confidence + 0.3 * min(1.0, inl.sum() / 12)))
        st.valid = True
        self._lost = 0
        static_mask[np.flatnonzero(ok)[inl]] = True
        # points too close / behind: classify with the model as well
        rest = ~ok & (rh > 0.2)
        if rest.any():
            cr = y[rest] / np.maximum(rh[rest], 1e-6)
            static_mask[rest] = np.abs(wrap(vm[rest] + st.speed * cr, self.vmax)) < tol
        return st, static_mask


# --------------------------------------------------------------- clustering
def lateral_tolerance(rng) -> np.ndarray:
    """Sideways spread of detections of one object: grows with range because
    the 4-antenna angle accuracy is a few degrees."""
    return 1.8 + 0.025 * np.asarray(rng)


def cluster(points_xy: np.ndarray, eps: float, extra: np.ndarray | None = None,
            extra_eps: float = 1.0) -> np.ndarray:
    """Connected-component clustering with a range-dependent lateral
    tolerance; ``extra`` (e.g. Doppler) must also agree.  Returns labels."""
    n = len(points_xy)
    labels = np.arange(n)
    if n == 0:
        return labels
    rng = np.hypot(points_xy[:, 0], points_xy[:, 1])
    scaled = np.column_stack([points_xy[:, 0] * eps / lateral_tolerance(rng), points_xy[:, 1]])
    pairs = cKDTree(scaled).query_pairs(eps, output_type="ndarray")
    if extra is not None and len(pairs):
        keep = np.abs(extra[pairs[:, 0]] - extra[pairs[:, 1]]) < extra_eps
        pairs = pairs[keep]

    def find(i):
        while labels[i] != i:
            labels[i] = labels[labels[i]]
            i = labels[i]
        return i

    for a, b in pairs:
        ra, rb = find(a), find(b)
        if ra != rb:
            labels[max(ra, rb)] = min(ra, rb)
    return np.array([find(i) for i in range(n)])


@dataclass
class Detection:
    x: float
    y: float
    vm: float            # measured (possibly wrapped) radial velocity
    width: float
    depth: float
    snr: float
    n: int
    static: bool = False      # only used to keep stopped vehicles tracked


# ------------------------------------------------------------------ tracker
_next_id = 1


@dataclass
class Track:
    x: np.ndarray                      # [x, y, vx, vy] relative to the radar
    P: np.ndarray
    id: int = 0
    hits: int = 1
    misses: int = 0
    age: int = 1
    confirmed: bool = False
    width: float = 1.0
    depth: float = 1.0
    snr: float = 10.0
    ground_vx: float = 0.0
    ground_vy: float = 0.0
    ground_accel: float = 0.0
    kind: str = "unknown"              # vehicle / pedestrian / unknown
    motion: str = "unknown"            # preceding / oncoming / crossing / stationary
    lane: int = 99
    ttc: float = math.inf
    history: deque = field(default_factory=lambda: deque(maxlen=40))

    @property
    def range(self) -> float:
        return float(math.hypot(self.x[0], self.x[1]))

    @property
    def speed_ground(self) -> float:
        return float(math.hypot(self.ground_vx, self.ground_vy))

    @property
    def velocity_known(self) -> bool:
        return self.P[2, 2] < 0.8 ** 2 and self.P[3, 3] < 0.8 ** 2 and self.hits >= 6

    @property
    def mature(self) -> bool:
        """Well established: alerts may be raised on it."""
        return self.confirmed and self.velocity_known and self.age >= 12 \
            and self.hits >= 0.6 * self.age


class Tracker:
    def __init__(self, vmax: float, settings: DriveSettings):
        self.vmax = vmax
        self.s = settings
        self.tracks: list[Track] = []
        self.q_lat = 1.2          # process noise (m/s^2): cars do not jump sideways
        self.q_lon = 3.5

    def _predict(self, t: Track, dt: float) -> None:
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        q = np.diag([self.q_lat ** 2, self.q_lon ** 2])
        t.x = f @ t.x
        t.P = f @ t.P @ f.T + g @ q @ g.T

    def _r_pos(self, det: Detection) -> np.ndarray:
        rng = math.hypot(det.x, det.y)
        lat = 0.4 + rng * math.radians(2.5)       # 4-antenna angle accuracy
        return np.diag([lat ** 2 + det.width ** 2 / 12, 0.3 ** 2 + det.depth ** 2 / 12])

    def update(self, dets: list[Detection], dt: float, ego: EgoState) -> list[Track]:
        global _next_id
        for t in self.tracks:
            self._predict(t, dt)

        # ---- association (Mahalanobis gating on position)
        nt, nd = len(self.tracks), len(dets)
        cost = np.full((nt, nd), 1e6)
        for i, t in enumerate(self.tracks):
            hp = t.P[:2, :2]
            for j, d in enumerate(dets):
                s = hp + self._r_pos(d)
                r = np.array([d.x - t.x[0], d.y - t.x[1]])
                m2 = float(r @ np.linalg.solve(s, r))
                if m2 < 13.8:                      # chi2(2) 99.9 %
                    cost[i, j] = m2
        assigned_t, assigned_d = set(), set()
        if nt and nd:
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if cost[i, j] < 1e5:
                    self._update(self.tracks[i], dets[j])
                    assigned_t.add(i)
                    assigned_d.add(j)

        for i, t in enumerate(self.tracks):
            t.age += 1
            if i not in assigned_t:
                t.misses += 1
        # ---- births
        for j, d in enumerate(dets):
            if j in assigned_d or d.static:
                continue
            r0 = self._r_pos(d)                     # start with the real (range
            p = np.diag([r0[0, 0], r0[1, 1], 15.0 ** 2, 15.0 ** 2])  # dependent) uncertainty
            self.tracks.append(Track(x=np.array([d.x, d.y, 0.0, 0.0]), P=p, id=_next_id,
                                     width=d.width, depth=d.depth, snr=d.snr))
            _next_id += 1
        # ---- confirm / delete
        keep = []
        for t in self.tracks:
            if not t.confirmed and t.hits >= 3 and t.hits >= 0.6 * t.age:
                t.confirmed = True
            dead = (t.misses > (8 if t.confirmed else 2)) or t.x[1] < -3 or \
                t.range > self.s.max_range_m + 5
            if not dead:
                keep.append(t)
        self.tracks = self._merge(keep)
        for t in self.tracks:
            self._classify(t, ego, dt)
        return self.tracks

    @staticmethod
    def _merge(tracks: list[Track]) -> list[Track]:
        """Two tracks on one object (it was split into two clusters): keep
        the older one."""
        tracks = sorted(tracks, key=lambda t: -t.hits)
        kept: list[Track] = []
        for t in tracks:
            dup = False
            for k in kept:
                tol = float(lateral_tolerance(k.range))
                if abs(t.x[1] - k.x[1]) < 3.0 and abs(t.x[0] - k.x[0]) < tol and \
                        abs(t.x[3] - k.x[3]) < 2.0:
                    dup = True
                    break
            if not dup:
                kept.append(t)
        return kept

    def near_track(self, x: float, y: float) -> bool:
        for t in self.tracks:
            if t.confirmed and abs(t.x[1] - y) < 3.0 and \
                    abs(t.x[0] - x) < float(lateral_tolerance(t.range)):
                return True
        return False

    def _update(self, t: Track, d: Detection) -> None:
        t.hits += 1
        t.misses = 0
        z = np.array([d.x, d.y])
        h = np.zeros((2, 4))
        h[0, 0] = h[1, 1] = 1
        r = self._r_pos(d)
        rng = max(0.5, math.hypot(d.x, d.y))
        # Doppler: usable once the velocity is known well enough to unfold it
        if t.P[2, 2] + t.P[3, 3] < 6.0 ** 2:
            ux, uy = d.x / rng, d.y / rng
            pred_rr = ux * t.x[2] + uy * t.x[3]
            k = round((pred_rr - d.vm) / (2 * self.vmax))   # Doppler unfolding
            rr = d.vm + 2 * self.vmax * k
            if abs(rr - pred_rr) < 2.5:
                z = np.array([d.x, d.y, rr])
                h = np.zeros((3, 4))
                h[0, 0] = h[1, 1] = 1
                h[2, 2], h[2, 3] = ux, uy       # range rate, line of sight held fixed
                r = np.diag([r[0, 0], r[1, 1], 0.35 ** 2])
        zp = h @ t.x
        s = h @ t.P @ h.T + r
        k_gain = t.P @ h.T @ np.linalg.inv(s)
        t.x = t.x + k_gain @ (z - zp)
        t.P = (np.eye(4) - k_gain @ h) @ t.P
        t.width = 0.8 * t.width + 0.2 * d.width
        t.depth = 0.8 * t.depth + 0.2 * d.depth
        t.snr = 0.8 * t.snr + 0.2 * d.snr

    def _classify(self, t: Track, ego: EgoState, dt: float) -> None:
        gvx = t.x[2] + ego.lateral
        gvy = t.x[3] + ego.speed
        sure = t.velocity_known
        new_acc = (gvy - t.ground_vy) / max(dt, 1e-3) if sure else 0.0
        t.ground_accel = 0.92 * t.ground_accel + 0.08 * float(np.clip(new_acc, -15, 15))
        t.ground_vx, t.ground_vy = float(gvx), float(gvy)
        sp = t.speed_ground
        if not sure:
            t.motion = "unknown"
        elif abs(gvx) > 1.0 and abs(gvx) > 0.6 * abs(gvy):
            t.motion = "crossing"            # lateral speed is not affected by ego-speed lag
        elif sp < 1.5:
            t.motion = "stationary"
        elif gvy > 0:
            t.motion = "preceding"
        else:
            t.motion = "oncoming"
        if t.width < 1.1 and t.depth < 1.2 and sp < 4.0 and t.snr < 22:
            t.kind = "pedestrian" if t.motion != "stationary" else "object"
        elif t.width >= 1.1 or sp > 6 or t.snr >= 22:
            t.kind = "vehicle"
        t.lane = int(round(t.x[0] / self.s.lane_width_m))
        closing = -t.x[3]
        t.ttc = t.x[1] / closing if closing > 0.3 and t.x[1] > 0 else math.inf
        t.history.append((float(t.x[0]), float(t.x[1])))


# -------------------------------------------------------- situation analysis
@dataclass
class Alert:
    level: int                # 0 info, 1 caution, 2 warning, 3 critical
    code: str
    text: str


@dataclass
class DriveResult:
    time_s: float = 0.0
    ego: EgoState = field(default_factory=EgoState)
    points: np.ndarray | None = None          # all detections of the frame (x, y)
    static_mask: np.ndarray | None = None
    tracks: list[Track] = field(default_factory=list)
    lead: Track | None = None
    lead_distance: float | None = None
    lead_rel_speed: float | None = None       # m/s, negative = closing
    lead_speed: float | None = None           # ground speed of the lead, m/s
    time_gap_s: float | None = None
    ttc_s: float | None = None
    decel_needed: float | None = None
    stationary_obstacle: float | None = None  # distance of a static object in our lane
    fcw_level: int = 0
    alerts: list[Alert] = field(default_factory=list)
    advice: str = ""
    trip_m: float = 0.0


class DriveAssist:
    def __init__(self, params: RadarParams, settings: DriveSettings | None = None):
        self.params = params
        self.s = settings or DriveSettings()
        self.s.max_range_m = min(self.s.max_range_m, params.max_range_m)
        self.vmax = params.max_velocity_ms
        self.reset()

    def reset(self) -> None:
        self.ego_est = EgoMotionEstimator(self.vmax, self.s)
        self.tracker = Tracker(self.vmax, self.s)
        self._last_fn: int | None = None
        self._static_hist: deque = deque(maxlen=6)
        self._alert_count: dict[str, int] = {}
        self.trip_m = 0.0
        self.frames = 0

    def process(self, frame: Frame) -> DriveResult:
        p = frame.points
        dt = self.params.frame_period_ms / 1000.0
        if self._last_fn is not None:
            gap = frame.frame_number - self._last_fn
            if gap <= 0 or gap > 40:
                self.reset()
            else:
                dt *= gap
        self._last_fn = frame.frame_number
        self.frames += 1

        x = p["x"].astype(float) if len(p) else np.zeros(0)
        y = p["y"].astype(float) if len(p) else np.zeros(0)
        vm = p["velocity"].astype(float) if len(p) else np.zeros(0)
        snr = np.nan_to_num(p["snr_db"].astype(float), nan=15.0) if len(p) else np.zeros(0)
        inside = (np.hypot(x, y) <= self.s.max_range_m) & (y > -0.5)
        x, y, vm, snr = x[inside], y[inside], vm[inside], snr[inside]

        ego, static = self.ego_est.update(x, y, vm, dt, weight=np.clip(snr / 15, 0.3, 2))
        self.trip_m += max(0.0, ego.speed) * dt
        res = DriveResult(time_s=frame.frame_number * self.params.frame_period_ms / 1000.0,
                          ego=EgoState(**vars(ego)), trip_m=self.trip_m)
        res.points = np.column_stack([x, y]) if len(x) else np.zeros((0, 2))
        res.static_mask = static

        # ---- moving objects
        dyn = ~static
        dets: list[Detection] = []
        if dyn.any():
            xd, yd, vd, sd = x[dyn], y[dyn], vm[dyn], snr[dyn]
            labels = cluster(np.column_stack([xd, yd]), self.s.cluster_eps_m,
                             extra=vd, extra_eps=1.2)
            for lab in np.unique(labels):
                m = labels == lab
                near = np.argmin(yd[m])
                dets.append(Detection(
                    x=float(np.median(xd[m])), y=float(yd[m][near] * 0.5 + np.median(yd[m]) * 0.5),
                    vm=float(np.median(vd[m])),
                    width=float(np.ptp(xd[m]) + 0.3), depth=float(np.ptp(yd[m]) + 0.3),
                    snr=float(np.max(sd[m])), n=int(m.sum())))
        # Stationary-looking detections are tracked too when they are on the
        # road (a pedestrian walking straight across has ~zero Doppler, so it
        # looks stationary - its track reveals the sideways motion), or next
        # to a confirmed track (a vehicle that has stopped must stay tracked).
        if static.any():
            corridor = self.s.lane_width_m + 0.5
            xs, ys, vs, ss = x[static], y[static], vm[static], snr[static]
            labels = cluster(np.column_stack([xs, ys]), 1.5)
            for lab in np.unique(labels):
                m = labels == lab
                cx, cy = float(np.median(xs[m])), float(np.min(ys[m]))
                on_road = abs(cx) < corridor and cy < 60
                if on_road or self.tracker.near_track(cx, cy):
                    dets.append(Detection(cx, cy, float(np.median(vs[m])),
                                          float(np.ptp(xs[m]) + 0.3), float(np.ptp(ys[m]) + 0.3),
                                          float(np.max(ss[m])), int(m.sum()),
                                          static=not on_road))
        tracks = self.tracker.update(dets, dt, ego)
        res.tracks = [t for t in tracks if t.confirmed]

        # ---- stationary obstacle in our lane (needs persistence)
        half = self.s.lane_width_m / 2 - 0.4
        in_lane = static & (np.abs(x) < half) & (y > 1.0)
        self._static_hist.append(float(y[in_lane].min()) if in_lane.any() else math.inf)
        seen = [d for d in self._static_hist if math.isfinite(d)]
        if len(seen) >= 4 and np.ptp(seen[-4:]) < 3.0 + ego.speed * dt * 6:
            res.stationary_obstacle = float(seen[-1])

        self._assess(res)
        return res

    # ----------------------------------------------------------------------
    def _assess(self, res: DriveResult) -> None:
        s = self.s
        ego = res.ego
        v = max(0.0, ego.speed)
        alerts: list[Alert] = []
        half = s.lane_width_m / 2

        # lead: closest confirmed object ahead in our lane
        cand = [t for t in res.tracks if t.x[1] > 0 and abs(t.x[0]) < half + 0.2
                and t.velocity_known and t.motion in ("preceding", "stationary")]
        lead = min(cand, key=lambda t: t.x[1]) if cand else None
        obst = res.stationary_obstacle if v > 2.0 else None
        if lead is not None and obst is not None and obst < lead.x[1] - 3:
            lead_d, closing = obst, v                   # stationary thing before the lead
            res.lead = None
        elif lead is not None:
            res.lead = lead
            lead_d, closing = float(lead.x[1]), float(-lead.x[3])
            res.lead_speed = lead.ground_vy
        elif obst is not None:
            lead_d, closing = obst, v
        else:
            lead_d = closing = None

        if lead_d is not None:
            res.lead_distance = lead_d
            res.lead_rel_speed = -closing
            if v > 2.0:
                res.time_gap_s = lead_d / v
            if closing > 0.3:
                res.ttc_s = lead_d / closing
                res.decel_needed = closing ** 2 / (2 * max(lead_d - 2.0, 0.5))
            level = 0
            if res.ttc_s is not None:
                if res.ttc_s < s.ttc_critical_s or (res.decel_needed or 0) > s.decel_critical:
                    level = 3
                elif res.ttc_s < s.ttc_warning_s or (res.decel_needed or 0) > s.decel_warning:
                    level = 2
                elif res.ttc_s < s.ttc_caution_s:
                    level = 1
            if level == 0 and res.time_gap_s is not None and res.time_gap_s < 0.8 and v > 8:
                level = 1
                alerts.append(Alert(1, "tailgating", f"Too close: {res.time_gap_s:.1f} s gap"))
            res.fcw_level = level
            what = "obstacle" if res.lead is None else "vehicle" if lead.kind == "vehicle" \
                else lead.kind
            if level == 3:
                alerts.append(Alert(3, "fcw", f"BRAKE! {what} {lead_d:.0f} m ahead"))
            elif level == 2:
                alerts.append(Alert(2, "fcw", f"Collision warning: {what} {lead_d:.0f} m, "
                                              f"TTC {res.ttc_s:.1f} s"))
            elif level == 1 and res.ttc_s is not None:
                alerts.append(Alert(1, "closing", f"Closing on {what}: TTC {res.ttc_s:.1f} s"))

        # lead braking hard
        if res.lead is not None and res.lead.mature and res.lead.age > 30 \
                and res.lead.ground_accel < -3.0 and res.lead.x[1] < 70 \
                and res.lead.ground_vy > 1.0:
            alerts.append(Alert(2, "lead_braking", "Vehicle ahead is braking hard"))

        for t in res.tracks:
            if not t.mature:
                continue
            # cut-in: neighbour lane, moving towards our lane centre, close ahead
            if (abs(t.lane) == 1 and 0 < t.x[1] < 40 and t.motion == "preceding"
                    and t.x[0] * t.x[2] < 0 and abs(t.x[2]) > 0.5
                    and abs(t.x[0]) < s.lane_width_m * 1.15):
                side = "right" if t.x[0] > 0 else "left"
                alerts.append(Alert(2, f"cutin{t.id}", f"Vehicle cutting in from the {side}"))
            # crossing pedestrian / cyclist / vehicle that will be in our path
            if t.motion == "crossing" and 0 < t.x[1] < 60:
                t_lane = (abs(t.x[0]) - half) / max(0.1, abs(t.x[2])) if t.x[0] * t.x[2] < 0 \
                    else (0.0 if abs(t.x[0]) < half else math.inf)
                t_reach = t.x[1] / max(0.5, v) if v > 0.5 else math.inf
                if t_lane < 5.0 and (t_reach < 6.0 or t.x[1] < 15):
                    who = "Pedestrian" if t.kind == "pedestrian" else "Crossing traffic"
                    lvl = 3 if (t_reach < 2.0 and abs(t.x[0]) < half + 0.5) else 2
                    alerts.append(Alert(lvl, f"cross{t.id}", f"{who} crossing ahead "
                                                              f"({t.x[1]:.0f} m)"))
            # oncoming vehicle in our lane (e.g. overtaker)
            if t.motion == "oncoming" and t.ground_vy < -4.0 and abs(t.x[0]) < half - 0.3 \
                    and 0 < t.x[1] < 70 and t.P[0, 0] < 0.7 ** 2:
                alerts.append(Alert(3, f"wrongway{t.id}", f"Oncoming vehicle in your lane "
                                                           f"({t.x[1]:.0f} m)!"))

        # adaptive-cruise style advice
        if res.time_gap_s is not None and res.lead is not None:
            desired = s.target_gap_s * v + 2.0
            sug = (res.lead_speed or 0) + 0.25 * (res.lead_distance - desired)
            sug = max(0.0, min(sug, v + 3))
            if res.time_gap_s < 0.6 * s.target_gap_s:
                res.advice = f"Drop back - aim for {sug * 3.6:.0f} km/h"
            elif res.time_gap_s < 0.9 * s.target_gap_s:
                res.advice = f"Ease off slightly ({sug * 3.6:.0f} km/h)"
            else:
                res.advice = f"Safe distance ({res.time_gap_s:.1f} s)"
        elif v > 2:
            res.advice = "Road ahead clear"
        else:
            res.advice = "Stationary"
        # debounce: an alert must persist for a few frames (critical: 2)
        counts = {}
        for a in alerts:
            counts[a.code] = self._alert_count.get(a.code, 0) + 1
        self._alert_count = counts
        alerts = [a for a in alerts if counts[a.code] >= (2 if a.level >= 3 else 3)]
        if res.fcw_level and not any(a.code == "fcw" for a in alerts):
            res.fcw_level = min(res.fcw_level, 1)
        res.alerts = sorted(alerts, key=lambda a: -a.level)

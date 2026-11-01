"""Custom-painted HMI widgets for the Car Radar Suite."""

from __future__ import annotations

import math

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from ..cabin import CabinResult
from ..drive import DriveResult, Track
from ..park import SECTORS, ParkResult

QPointF = QtCore.QPointF
Qt = QtCore.Qt

C_BG = QtGui.QColor("#0d121b")
C_ROAD = QtGui.QColor("#1b222e")
C_MARK = QtGui.QColor("#c9d1dc")
C_TEXT = QtGui.QColor("#e6edf3")
C_MUTED = QtGui.QColor("#7b8794")
C_LEAD = QtGui.QColor("#f59e0b")
C_SAME = QtGui.QColor("#3b82f6")
C_ONCOMING = QtGui.QColor("#d946ef")
C_CROSS = QtGui.QColor("#22d3ee")
C_STATIC = QtGui.QColor("#64748b")
C_OK = QtGui.QColor("#4ade80")
C_WARN = QtGui.QColor("#fbbf24")
C_ALERT = QtGui.QColor("#f87171")
LEVEL_COLORS = {0: C_OK, 1: C_WARN, 2: QtGui.QColor("#fb923c"), 3: C_ALERT}


def _font(px: int, bold: bool = False) -> QtGui.QFont:
    f = QtGui.QFont()
    f.setPixelSize(max(8, int(px)))
    f.setBold(bold)
    return f


# =================================================================== road view
class RoadView(QtWidgets.QWidget):
    """Chase-camera perspective of the road ahead with tracked objects as
    3D boxes.  Lane markings scroll with the radar-measured ego speed."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 200)
        self.res: DriveResult | None = None
        self.lane_width = 3.5
        self.max_range = 85.0
        self.blink = False
        # chase camera: the road at the radar projects near the bottom edge,
        # 10 m ahead at about 60 % of the height
        self.cam_back, self.cam_h = 11.4, 2.4
        self.horizon_frac = 0.28

    def set_result(self, res: DriveResult, lane_width: float, max_range: float) -> None:
        self.res, self.lane_width, self.max_range = res, lane_width, max_range
        self.update()

    def _proj(self, x: float, y: float, z: float = 0.0):
        w, h = self.width(), self.height()
        d = y + self.cam_back
        if d < 0.8:
            return None
        f = min(2.86 * h, 1.7 * w)
        horizon = h * self.horizon_frac
        return QPointF(w / 2 + f * x / d, horizon + f * (self.cam_h - z) / d)

    def _poly(self, pts):
        q = [self._proj(*p) for p in pts]
        if any(p is None for p in q):
            return None
        return QtGui.QPolygonF(q)

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        horizon = h * self.horizon_frac
        sky = QtGui.QLinearGradient(0, 0, 0, horizon)
        sky.setColorAt(0, QtGui.QColor("#05080f"))
        sky.setColorAt(1, QtGui.QColor("#16233d"))
        p.fillRect(QtCore.QRectF(0, 0, w, horizon), sky)
        ground = QtGui.QLinearGradient(0, horizon, 0, h)
        ground.setColorAt(0, QtGui.QColor("#0f141d"))
        ground.setColorAt(1, QtGui.QColor("#121925"))
        p.fillRect(QtCore.QRectF(0, horizon, w, h - horizon), ground)

        lw = self.lane_width
        far = self.max_range
        res = self.res
        # road surface: our lane plus one lane each side
        near = 2.0 - self.cam_back           # road continues under our bonnet
        road = self._poly([(-1.5 * lw, near), (1.5 * lw, near), (1.5 * lw, far), (-1.5 * lw, far)])
        if road:
            g = QtGui.QLinearGradient(0, horizon, 0, h)
            g.setColorAt(0, QtGui.QColor("#1a212c"))
            g.setColorAt(1, QtGui.QColor("#262f3d"))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(g)
            p.drawPolygon(road)
        # collision warning: tint our lane up to the hazard
        if res is not None and res.fcw_level >= 2 and res.lead_distance:
            if res.fcw_level < 3 or self.blink:
                tint = self._poly([(-lw / 2, 0), (lw / 2, 0), (lw / 2, res.lead_distance),
                                   (-lw / 2, res.lead_distance)])
                if tint:
                    col = QtGui.QColor(LEVEL_COLORS[res.fcw_level])
                    col.setAlpha(70)
                    p.setBrush(col)
                    p.drawPolygon(tint)
        elif res is not None and res.time_gap_s is not None and res.lead_distance:
            tint = self._poly([(-lw / 2, 0), (lw / 2, 0), (lw / 2, res.lead_distance),
                               (-lw / 2, res.lead_distance)])
            if tint:
                col = QtGui.QColor(C_OK if res.time_gap_s >= 1.5 else C_WARN)
                col.setAlpha(28)
                p.setBrush(col)
                p.drawPolygon(tint)

        # lane markings: solid edges, dashed separators scrolling with ego motion
        phase = (res.trip_m if res else 0.0) % 9.0
        pen = QtGui.QPen(C_MARK, 2)
        p.setPen(pen)
        for xe in (-1.5 * lw, 1.5 * lw):
            a, b = self._proj(xe, near), self._proj(xe, far)
            if a and b:
                p.drawLine(a, b)
        for xs in (-lw / 2, lw / 2):
            y0 = near - phase
            while y0 < far:
                ya, yb = max(near, y0), min(far, y0 + 3.0)
                if yb > ya:
                    poly = self._poly([(xs - 0.07, ya), (xs + 0.07, ya), (xs + 0.07, yb), (xs - 0.07, yb)])
                    if poly:
                        p.setPen(Qt.PenStyle.NoPen)
                        p.setBrush(C_MARK)
                        p.drawPolygon(poly)
                y0 += 9.0
        # distance ticks
        p.setFont(_font(max(10, h / 55)))
        for dist in (10, 20, 40, 60, 80):
            if dist > far:
                continue
            a, b = self._proj(1.5 * lw + 0.3, dist), self._proj(1.5 * lw + 1.2, dist)
            if a and b:
                p.setPen(QtGui.QPen(C_MUTED, 1))
                p.drawLine(a, b)
                p.drawText(b + QPointF(4, 4), f"{dist} m")

        if res is not None:
            # stationary detections (posts, signs...) as small ground dots
            if res.points is not None and len(res.points):
                p.setPen(Qt.PenStyle.NoPen)
                for (x, y), st in zip(res.points, res.static_mask):
                    q = self._proj(x, y)
                    if q is None or not st:
                        continue
                    r = max(1.2, 9.0 * 10 / (y + self.cam_back))
                    p.setBrush(QtGui.QColor(100, 116, 139, 160))
                    p.drawEllipse(q, r, r * 0.6)
            # tracked objects, far to near
            for t in sorted(res.tracks, key=lambda t: -t.x[1]):
                if t.x[1] < 0.5:
                    continue
                self._draw_track(p, t, t is res.lead, res)

        # our car's bonnet
        hood = QtGui.QPainterPath()
        hood.moveTo(w * 0.18, h)
        hood.cubicTo(w * 0.25, h * 0.90, w * 0.75, h * 0.90, w * 0.82, h)
        hood.closeSubpath()
        g = QtGui.QLinearGradient(0, h * 0.9, 0, h)
        g.setColorAt(0, QtGui.QColor("#2b3646"))
        g.setColorAt(1, QtGui.QColor("#121821"))
        p.setPen(QtGui.QPen(QtGui.QColor("#3c4a5f"), 1.5))
        p.setBrush(g)
        p.drawPath(hood)
        p.end()

    def _draw_track(self, p: QtGui.QPainter, t: Track, is_lead: bool, res: DriveResult) -> None:
        if t.kind == "pedestrian":
            wid, hei, length = 0.6, 1.75, 0.5
        elif t.kind == "object":
            wid, hei, length = 0.8, 1.0, 0.8
        else:
            wid, hei, length = max(1.7, min(2.5, t.width)), 1.5, 4.2
        if is_lead:
            col = LEVEL_COLORS[res.fcw_level] if res.fcw_level >= 2 else C_LEAD
        elif t.motion == "oncoming":
            col = C_ONCOMING
        elif t.motion == "crossing" or t.kind == "pedestrian":
            col = C_CROSS
        elif t.motion == "stationary":
            col = C_STATIC
        else:
            col = C_SAME
        x0, y0 = t.x[0] - wid / 2, t.x[1]
        x1, y1 = t.x[0] + wid / 2, t.x[1] + length
        front = self._poly([(x0, y0, 0), (x1, y0, 0), (x1, y0, hei), (x0, y0, hei)])
        top = self._poly([(x0, y0, hei), (x1, y0, hei), (x1, y1, hei), (x0, y1, hei)])
        side_x = x0 if t.x[0] > 0 else x1
        side = self._poly([(side_x, y0, 0), (side_x, y1, 0), (side_x, y1, hei), (side_x, y0, hei)])
        if front is None:
            return
        dark = QtGui.QColor(col).darker(170)
        p.setPen(QtGui.QPen(col.lighter(130), 1.2))
        if side:
            p.setBrush(dark)
            p.drawPolygon(side)
        if top:
            p.setBrush(QtGui.QColor(col).darker(130))
            p.drawPolygon(top)
        p.setBrush(col)
        p.drawPolygon(front)
        # label
        br = front.boundingRect()
        size = max(10, min(15, br.width() / 5 + 9))
        p.setFont(_font(size, bold=is_lead))
        speed = t.ground_vy * 3.6 if t.motion != "crossing" else t.speed_ground * 3.6
        label = f"{t.x[1]:.0f} m"
        if t.motion not in ("stationary", "unknown"):
            label += f" · {abs(speed):.0f} km/h"
        if t.kind == "pedestrian":
            label = "\U0001F6B6 " + label
        p.setPen(C_TEXT)
        p.drawText(QPointF(br.center().x() - p.fontMetrics().horizontalAdvance(label) / 2,
                           br.top() - 6), label)


# ===================================================================== gauge
class Gauge(QtWidgets.QWidget):
    def __init__(self, max_value: float = 160, unit: str = "km/h"):
        super().__init__()
        self.setMinimumSize(170, 150)
        self.value = 0.0
        self.max_value = max_value
        self.unit = unit
        self.subtitle = ""
        self.valid = False

    def set_value(self, v: float, valid: bool, subtitle: str = "") -> None:
        self.value, self.valid, self.subtitle = v, valid, subtitle
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        side = min(w, h * 1.25)
        r = side * 0.42
        c = QPointF(w / 2, h * 0.56)
        rect = QtCore.QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r)
        start, span = 215, -250
        pen = QtGui.QPen(QtGui.QColor("#263042"), r * 0.12)
        pen.setCapStyle(Qt.PenCapStyle.FlatCap)
        p.setPen(pen)
        p.drawArc(rect, start * 16, span * 16)
        frac = max(0.0, min(1.0, self.value / self.max_value))
        col = QtGui.QColor("#22d3ee") if self.valid else QtGui.QColor("#475569")
        pen = QtGui.QPen(col, r * 0.12)
        pen.setCapStyle(Qt.PenCapStyle.FlatCap)
        p.setPen(pen)
        p.drawArc(rect, start * 16, int(span * frac * 16))
        # ticks
        p.setFont(_font(r * 0.13))
        for k in range(0, int(self.max_value) + 1, 20):
            a = math.radians(start + span * k / self.max_value)
            outer = QPointF(c.x() + math.cos(a) * r * 0.86, c.y() - math.sin(a) * r * 0.86)
            inner = QPointF(c.x() + math.cos(a) * r * 0.78, c.y() - math.sin(a) * r * 0.78)
            p.setPen(QtGui.QPen(C_MUTED, 1.5))
            p.drawLine(inner, outer)
            lab = QPointF(c.x() + math.cos(a) * r * 0.64, c.y() - math.sin(a) * r * 0.64)
            txt = str(k)
            fm = p.fontMetrics()
            p.drawText(lab - QPointF(fm.horizontalAdvance(txt) / 2, -fm.ascent() / 3), txt)
        # value
        p.setPen(C_TEXT if self.valid else C_MUTED)
        p.setFont(_font(r * 0.42, bold=True))
        txt = f"{self.value:.0f}" if self.valid else "--"
        fm = p.fontMetrics()
        p.drawText(QPointF(c.x() - fm.horizontalAdvance(txt) / 2, c.y() + fm.ascent() * 0.35), txt)
        p.setFont(_font(r * 0.14))
        p.setPen(C_MUTED)
        fm = p.fontMetrics()
        p.drawText(QPointF(c.x() - fm.horizontalAdvance(self.unit) / 2, c.y() + r * 0.42), self.unit)
        if self.subtitle:
            p.setFont(_font(max(10, r * 0.11)))
            fm = p.fontMetrics()
            p.drawText(QPointF(c.x() - fm.horizontalAdvance(self.subtitle) / 2, c.y() + r * 0.66),
                       self.subtitle)
        p.end()


# ================================================================== park view
class ParkView(QtWidgets.QWidget):
    """Top view of the bumper with three sectors, like a car's park display."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(280, 220)
        self.res: ParkResult | None = None
        self.bands = (0.3, 0.6, 1.2, 2.5, 4.0)
        self.blink = False

    def set_result(self, res: ParkResult, bands=None) -> None:
        self.res = res
        if bands:
            self.bands = bands
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), C_BG)
        bumper_y = h * 0.62
        scale = min(w / 9.0, (bumper_y - 20) / 4.2)       # px per metre
        cx = w / 2
        res = self.res

        def to_px(x, y):
            return QPointF(cx + x * scale, bumper_y - y * scale)

        # car body below the bumper
        car_w = 1.8 * scale
        body = QtCore.QRectF(cx - car_w / 2, bumper_y, car_w, h - bumper_y + 30)
        p.setPen(QtGui.QPen(QtGui.QColor("#3c4a5f"), 2))
        p.setBrush(QtGui.QColor("#1f2937"))
        p.drawRoundedRect(body, car_w * 0.18, car_w * 0.18)
        p.setBrush(QtGui.QColor("#0f172a"))
        p.drawRoundedRect(QtCore.QRectF(cx - car_w * 0.38, bumper_y + car_w * 0.25, car_w * 0.76,
                                        car_w * 0.45), 6, 6)
        p.setBrush(QtGui.QColor("#22d3ee"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRoundedRect(QtCore.QRectF(cx - 10, bumper_y - 4, 20, 6), 2, 2)  # the radar

        # sector fans: equal-width bands (like a car's display); the band
        # holding the obstacle is lit, free space before it stays dark
        edges = [0.0] + list(self.bands)
        n_b = len(self.bands)
        reach = (bumper_y - 24) / scale            # metres of screen height available

        def vis(d):                                 # distance -> display radius (m)
            d = min(max(d, 0.0), edges[-1])
            for i in range(n_b):
                if d <= edges[i + 1]:
                    f = (d - edges[i]) / (edges[i + 1] - edges[i])
                    return reach * (i + f) / n_b
            return reach

        def to_vis(x, y):
            return to_px(x * (1 + vis(y) * 0.18), vis(y))

        spans = {"left": (-1.9, -0.3), "centre": (-0.3, 0.3), "right": (0.3, 1.9)}
        for k in SECTORS:
            d = res.distances[k] if res else math.inf
            zone_color = self._color_for(d)
            x0, x1 = spans[k]
            for i in range(n_b):
                lo, hi = edges[i], edges[i + 1]
                r0, r1 = reach * i / n_b, reach * (i + 1) / n_b
                poly = QtGui.QPolygonF([to_px(x0 * (1 + r0 * 0.18), r0 + 0.03),
                                        to_px(x1 * (1 + r0 * 0.18), r0 + 0.03),
                                        to_px(x1 * (1 + r1 * 0.18), r1),
                                        to_px(x0 * (1 + r1 * 0.18), r1)])
                if lo < d <= hi or (i == 0 and d <= lo):
                    flash_off = zone_color == C_ALERT and d <= self.bands[0] and not self.blink
                    col = QtGui.QColor("#7f1d1d") if flash_off else QtGui.QColor(zone_color)
                elif d < lo:                              # behind the obstacle
                    col = QtGui.QColor(zone_color)
                    col.setAlpha(45)
                else:                                     # free space
                    col = QtGui.QColor("#1e293b")
                p.setBrush(col)
                p.setPen(QtGui.QPen(C_BG, 3))
                p.drawPolygon(poly)
        # band labels
        p.setFont(_font(max(9, scale * 0.16)))
        p.setPen(C_MUTED)
        for i, b in enumerate(self.bands):
            q = to_px(1.9 * (1 + reach * (i + 1) / n_b * 0.18) + 0.15, reach * (i + 1) / n_b)
            p.drawText(q, f"{b:g} m")
        # detections
        if res is not None:
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor(255, 255, 255, 170))
            for x, y in res.points:
                p.drawEllipse(to_vis(x, y), 3, 3)
            # distances per sector
            p.setFont(_font(max(12, scale * 0.28), bold=True))
            for k, (x0, x1) in spans.items():
                d = res.distances[k]
                if math.isfinite(d):
                    txt = f"{d * 100:.0f} cm" if d < 1 else f"{d:.1f} m"
                    q = to_vis((x0 + x1) / 2, d)
                    p.setPen(C_TEXT)
                    fm = p.fontMetrics()
                    p.drawText(q - QPointF(fm.horizontalAdvance(txt) / 2, -fm.ascent() / 2), txt)
        p.end()

    def _color_for(self, d: float) -> QtGui.QColor:
        if d <= 0.6:
            return C_ALERT
        if d <= 1.2:
            return C_WARN
        return C_OK


# ================================================================= cabin view
class CabinView(QtWidgets.QWidget):
    """Top view of the cabin; seats light up when occupied and pulse with the
    measured breathing rate."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(220, 260)
        self.res: CabinResult | None = None
        self.driver_on_left = True
        self._phase: dict[str, float] = {}
        self.blink = False

    def set_result(self, res: CabinResult, driver_on_left: bool) -> None:
        self.res = res
        self.driver_on_left = driver_on_left
        self.update()

    def advance(self, dt: float) -> None:
        if self.res is None:
            return
        for st in self.res.seats:
            if st.breath_bpm:
                self._phase[st.seat.key] = (self._phase.get(st.seat.key, 0.0)
                                            + dt * st.breath_bpm / 60) % 1.0
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), C_BG)
        car_h = h * 0.94
        car_w = min(w * 0.82, car_h * 0.48)
        car = QtCore.QRectF(w / 2 - car_w / 2, h * 0.03, car_w, car_h)
        p.setPen(QtGui.QPen(QtGui.QColor("#3c4a5f"), 2.5))
        p.setBrush(QtGui.QColor("#161d29"))
        p.drawRoundedRect(car, car_w * 0.28, car_w * 0.22)
        # windscreen + dashboard + radar at the mirror
        ws = QtCore.QRectF(car.left() + car_w * 0.12, car.top() + car_h * 0.10, car_w * 0.76, car_h * 0.07)
        p.setBrush(QtGui.QColor("#1e3a5f"))
        p.drawRoundedRect(ws, 10, 10)
        p.setBrush(QtGui.QColor("#22d3ee"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(QPointF(w / 2, ws.bottom() + 6), 6, 6)
        p.setPen(C_MUTED)
        p.setFont(_font(max(9, car_w * 0.04)))
        p.drawText(QPointF(w / 2 + 10, ws.bottom() + 10), "radar")

        seat_w, seat_h = car_w * 0.30, car_h * 0.17
        left_x = car.left() + car_w * 0.12
        right_x = car.right() - car_w * 0.12 - seat_w
        front_y = car.top() + car_h * 0.27
        rear_y = car.top() + car_h * 0.58
        rear_w, mid_w = car_w * 0.26, car_w * 0.20      # three seats across the bench
        layout = {
            "driver": (left_x if self.driver_on_left else right_x, front_y, seat_w),
            "front_pass": (right_x if self.driver_on_left else left_x, front_y, seat_w),
            "rear_left": (car.left() + car_w * 0.08, rear_y, rear_w),
            "rear_mid": (car.left() + car_w * 0.40, rear_y + seat_h * 0.06, mid_w),
            "rear_right": (car.left() + car_w * 0.66, rear_y, rear_w),
        }
        states = {st.seat.key: st for st in self.res.seats} if self.res else {}
        cpd = self.res.cpd_level if self.res else 0
        for key, (sx, sy, wid) in layout.items():
            st = states.get(key)
            rect = QtCore.QRectF(sx, sy, wid, seat_h)
            occupied = st is not None and st.occupied
            if occupied and cpd >= 2 and not (st.seat.driver and cpd < 3):
                col = C_ALERT if (cpd < 3 or self.blink) else QtGui.QColor("#7f1d1d")
            elif occupied:
                col = QtGui.QColor("#15803d")
            else:
                col = QtGui.QColor("#273244")
            p.setPen(QtGui.QPen(col.lighter(140), 2))
            p.setBrush(col)
            p.drawRoundedRect(rect, 12, 12)
            p.setBrush(col.darker(130))
            p.drawRoundedRect(QtCore.QRectF(sx + wid * 0.1, sy + seat_h * 0.78, wid * 0.8, seat_h * 0.2), 6, 6)
            if key == "driver":
                p.setPen(QtGui.QPen(QtGui.QColor("#94a3b8"), 3))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(QPointF(rect.center().x(), sy - seat_h * 0.18), seat_w * 0.22, seat_w * 0.09)
            if occupied:
                ph = self._phase.get(key, 0.0)
                breath = 0.5 - 0.5 * math.cos(2 * math.pi * ph) if st.breath_bpm else 0.3
                rad = min(wid, seat_h) * (0.16 + 0.10 * breath)
                ring = QtGui.QColor(255, 255, 255, 200)
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(ring)
                p.drawEllipse(QPointF(rect.center().x(), sy + seat_h * 0.36), rad, rad)
            p.setPen(C_TEXT)
            p.setFont(_font(max(9, min(seat_h * 0.13, wid * 0.16)), bold=occupied))
            label = "occupied" if occupied else "empty"
            if occupied and st.breath_bpm:
                label = f"{st.breath_bpm:.0f}/min"
            fm = p.fontMetrics()
            p.drawText(QPointF(rect.center().x() - fm.horizontalAdvance(label) / 2,
                               sy + seat_h * 0.70), label)
        p.end()

"""Custom dashboard widgets: metric cards and animated heart / lung icons."""

from __future__ import annotations

import math

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

HEART = "#ff5c7a"
BREATH = "#4fd1ff"
OK = "#4ade80"
WARN = "#fbbf24"
ALERT = "#f87171"
MUTED = "#7b8794"

STYLE = """
QWidget { color: #e6edf3; font-size: 10pt; }
QMainWindow, QScrollArea, QScrollArea > QWidget > QWidget { background: #0f141c; }
QFrame#card { background: #171e2a; border: 1px solid #263042; border-radius: 14px; }
QLabel#cardTitle { color: #9aa7b6; font-size: 9pt; font-weight: 600;
                   letter-spacing: 1px; }
QLabel#bigValue { font-size: 44pt; font-weight: 700; }
QLabel#unit { color: #9aa7b6; font-size: 11pt; }
QLabel#sub { color: #b8c4d2; font-size: 10pt; }
QGroupBox { border: 1px solid #263042; border-radius: 10px; margin-top: 14px;
            padding: 6px; background: #141a24; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px;
                   color: #9aa7b6; font-weight: 600; }
QPushButton { background: #223047; border: 1px solid #2f3f59; border-radius: 7px;
              padding: 6px 10px; }
QPushButton:hover { background: #2b3c58; }
QPushButton:disabled { color: #56606d; background: #19212e; }
QPushButton#primary { background: #2563eb; border-color: #3b82f6; font-weight: 700; }
QPushButton#primary:hover { background: #3b74f0; }
QPushButton#primary:disabled { background: #1c2a44; border-color: #26385a; color: #6b7a90; }
QPushButton#danger:checked { background: #b91c1c; border-color: #ef4444; }
QComboBox, QLineEdit, QDoubleSpinBox, QSpinBox, QPlainTextEdit {
    background: #0f141c; border: 1px solid #2f3f59; border-radius: 6px; padding: 3px 6px; }
QTabWidget::pane { border: 1px solid #263042; border-radius: 10px; background: #141a24; }
QTabBar::tab { background: #171e2a; padding: 7px 16px; border-top-left-radius: 8px;
               border-top-right-radius: 8px; margin-right: 2px; color: #9aa7b6; }
QTabBar::tab:selected { background: #223047; color: #e6edf3; }
QProgressBar { background: #0f141c; border: none; border-radius: 3px; height: 6px;
               text-align: center; }
QStatusBar { color: #9aa7b6; }
"""


class ConfidenceBar(QtWidgets.QProgressBar):
    def __init__(self, color: str):
        super().__init__()
        self.setRange(0, 100)
        self.setTextVisible(False)
        self.setFixedHeight(6)
        self._color = color
        self.set_value(0)

    def set_value(self, conf: float) -> None:
        v = int(max(0.0, min(1.0, conf)) * 100)
        self.setValue(v)
        col = self._color if v >= 50 else (WARN if v >= 25 else ALERT)
        self.setStyleSheet(f"QProgressBar::chunk {{ background: {col}; border-radius: 3px; }}")


class HeartIcon(QtWidgets.QWidget):
    """A heart that beats at the measured rate."""

    def __init__(self, size: int = 74):
        super().__init__()
        self.setFixedSize(size, size)
        self.bpm: float | None = None
        self._phase = 0.0

    def advance(self, dt: float) -> None:
        if self.bpm:
            self._phase = (self._phase + dt * self.bpm / 60.0) % 1.0
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802 (Qt API)
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        beat = math.exp(-self._phase / 0.08) + 0.5 * math.exp(-abs(self._phase - 0.22) / 0.06)
        scale = 0.72 + (0.12 * beat if self.bpm else 0.0)
        p.translate(w / 2, h / 2 + 2)
        p.scale(scale * w / 100, scale * h / 100)
        path = QtGui.QPainterPath()
        path.moveTo(0, 35)
        path.cubicTo(-60, -5, -35, -55, 0, -22)
        path.cubicTo(35, -55, 60, -5, 0, 35)
        col = QtGui.QColor(HEART if self.bpm else MUTED)
        glow = QtGui.QColor(col)
        glow.setAlpha(int(60 * beat) if self.bpm else 0)
        p.setPen(QtGui.QPen(glow, 10))
        p.setBrush(col)
        p.drawPath(path)
        p.end()


class BreathIcon(QtWidgets.QWidget):
    """Concentric rings whose size follows the measured breathing waveform."""

    def __init__(self, size: int = 74):
        super().__init__()
        self.setFixedSize(size, size)
        self.level: float | None = None   # 0 (exhaled) .. 1 (inhaled)

    def set_level(self, level: float | None) -> None:
        self.level = level
        self.update()

    def paintEvent(self, _ev) -> None:  # noqa: N802
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        c = QtCore.QPointF(w / 2, h / 2)
        active = self.level is not None
        lv = self.level if active else 0.3
        base = QtGui.QColor(BREATH if active else MUTED)
        rmax = min(w, h) / 2 - 2
        r = rmax * (0.45 + 0.5 * lv)
        halo = QtGui.QColor(base)
        halo.setAlpha(45)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(halo)
        p.drawEllipse(c, rmax, rmax)
        fill = QtGui.QColor(base)
        fill.setAlpha(170)
        p.setBrush(fill)
        p.drawEllipse(c, r, r)
        p.setBrush(base)
        p.drawEllipse(c, r * 0.45, r * 0.45)
        p.end()


class MetricCard(QtWidgets.QFrame):
    def __init__(self, title: str, unit: str, color: str, icon: QtWidgets.QWidget | None = None):
        super().__init__()
        self.setObjectName("card")
        self.color = color
        lay = QtWidgets.QGridLayout(self)
        lay.setContentsMargins(16, 12, 16, 12)
        lay.setVerticalSpacing(2)
        t = QtWidgets.QLabel(title.upper())
        t.setObjectName("cardTitle")
        lay.addWidget(t, 0, 0, 1, 2)
        self.value = QtWidgets.QLabel("--")
        self.value.setObjectName("bigValue")
        self.unit = QtWidgets.QLabel(unit)
        self.unit.setObjectName("unit")
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(8)
        row.addWidget(self.value)
        row.addWidget(self.unit, 0, QtCore.Qt.AlignmentFlag.AlignBottom)
        row.addStretch(1)
        lay.addLayout(row, 1, 0)
        if icon is not None:
            lay.addWidget(icon, 1, 1, QtCore.Qt.AlignmentFlag.AlignRight)
        self.sub = QtWidgets.QLabel("")
        self.sub.setObjectName("sub")
        lay.addWidget(self.sub, 2, 0, 1, 2)
        self.conf = ConfidenceBar(color)
        lay.addWidget(self.conf, 3, 0, 1, 2)
        self.set_value(None)

    def set_value(self, text: str | None, dim: bool = False) -> None:
        self.value.setText(text if text is not None else "--")
        col = MUTED if (text is None or dim) else self.color
        self.value.setStyleSheet(f"color: {col};")

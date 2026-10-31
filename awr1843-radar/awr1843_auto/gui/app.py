"""AWR1843 Car Radar Suite - main window (Drive Assist / Park Assist / Cabin Guard)."""

from __future__ import annotations

import csv
import math
import sys
import threading
import time
import traceback
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

from awr1843.config import RadarParams, load_cfg_file, parse_cfg
from awr1843.device import FrameSource, RadarDevice, list_ports
from awr1843.gui.util import fill_port_combo, fit_panel_width, fit_to_screen, setup_port_combo
from awr1843.recorder import Recorder, ReplaySource, new_recording_dir
from awr1843.tlv import Frame
from awr1843_vitals.gui.widgets import STYLE

from ..cabin import CabinGuard, CabinResult, default_seats
from ..drive import DriveAssist, DriveResult
from ..park import ParkAssist, ParkResult
from ..simulator import scene_for, simulated_auto_device
from .sound import Sounds
from .views import (C_ALERT, C_LEAD, C_OK, C_WARN, LEVEL_COLORS, CabinView, Gauge, ParkView,
                    RoadView)

CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"
MODES = {
    "drive": ("\U0001F697  Drive Assist", "drive_long_range.cfg"),
    "park": ("\U0001F17F️  Park Assist", "park_short_range.cfg"),
    "cabin": ("\U0001F476  Cabin Guard", "cabin_occupancy.cfg"),
}
Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal

EXTRA_STYLE = """
QPushButton#mode { font-size: 12pt; font-weight: 600; padding: 9px 18px; border-radius: 10px;
                   background: #171e2a; border: 1px solid #263042; color: #9aa7b6; }
QPushButton#mode:checked { background: #1d4ed8; border-color: #3b82f6; color: #ffffff; }
QLabel#banner { border-radius: 10px; padding: 10px 12px; font-size: 13pt; font-weight: 700; }
QLabel#big { font-size: 30pt; font-weight: 700; }
QLabel#k { color: #9aa7b6; }
QLabel#v { font-weight: 700; font-size: 12pt; }
QListWidget { background: #0f141c; border: 1px solid #263042; border-radius: 8px; }
QPushButton#lock { font-size: 13pt; font-weight: 700; padding: 12px; border-radius: 10px; }
QPushButton#lock:checked { background: #b91c1c; border-color: #ef4444; }
"""


class Bridge(QtCore.QObject):
    log = Signal(str)
    task_done = Signal(str, bool, str)


def _group(title):
    box = QtWidgets.QGroupBox(title)
    lay = QtWidgets.QVBoxLayout(box)
    lay.setContentsMargins(10, 14, 10, 10)
    return box, lay


def _row(*ws):
    lay = QtWidgets.QHBoxLayout()
    for w in ws:
        lay.addStretch(w) if isinstance(w, int) else lay.addWidget(w)
    return lay


def _scroll(widget: QtWidgets.QWidget, min_w: int, max_w: int) -> QtWidgets.QScrollArea:
    """Side column that scrolls instead of forcing a tall window."""
    area = QtWidgets.QScrollArea()
    area.setWidget(widget)
    area.setWidgetResizable(True)
    area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
    area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    area.setMinimumWidth(min_w)
    area.setMaximumWidth(max_w)
    return area


def _card():
    f = QtWidgets.QFrame()
    f.setObjectName("card")
    return f


class CsvLog:
    def __init__(self, path: Path, columns: list[str]):
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(columns)
        self.n = 0

    def write(self, row) -> None:
        self.w.writerow(["" if v is None else (f"{v:.3f}" if isinstance(v, float) else v)
                         for v in row])
        self.n += 1
        if self.n % 50 == 0:
            self.fh.flush()

    def close(self) -> None:
        self.fh.close()


class AutoWindow(QtWidgets.QMainWindow):
    def __init__(self, sim: bool = False, mode: str = "drive",
                 cli_port: str | None = None, data_port: str | None = None):
        super().__init__()
        self.setWindowTitle("AWR1843 Car Radar Suite")
        self.source: FrameSource | None = None
        self.mode = mode
        self.busy = False
        self.recorder: Recorder | None = None
        self.log: CsvLog | None = None
        self.sounds = Sounds()
        self._incoming: deque[Frame] = deque(maxlen=4000)
        self.drive_res: DriveResult | None = None
        self.park_res: ParkResult | None = None
        self.cabin_res: CabinResult | None = None
        self._last_cabin_update = -1e9
        self._alert_seen: set[str] = set()
        self._next_sound = 0.0
        self._blink_t = 0.0
        self._last_tick = time.monotonic()
        self._session_t0 = time.monotonic()
        self.max_speed = 0.0

        self.bridge = Bridge()
        self.bridge.log.connect(self._on_log)
        self.bridge.task_done.connect(self._on_task_done)
        self._build_ui()
        self._refresh_ports(cli_port, data_port)
        if sim:
            self.source_combo.setCurrentIndex(1)
        self._set_mode(mode)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(40)
        self._update_controls()

    # ================================================================ UI
    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QHBoxLayout(root)
        outer.setContentsMargins(8, 8, 8, 4)
        self.split = QtWidgets.QSplitter()
        self.split.setHandleWidth(8)
        outer.addWidget(self.split)

        # ---------------------------------------------------------- sidebar
        side = QtWidgets.QWidget()
        sl = QtWidgets.QVBoxLayout(side)
        sl.setContentsMargins(0, 0, 4, 0)
        sl.addWidget(QtWidgets.QLabel(
            "<span style='font-size:15pt;font-weight:700'>Car Radar Suite</span><br>"
            "<span style='color:#9aa7b6'>AWR1843BOOST mmWave radar</span>"))

        box, lay = _group("Radar")
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.addItems(["AWR1843BOOST (USB)", "Simulator (virtual car)"])
        self.source_combo.currentIndexChanged.connect(self._update_controls)
        lay.addWidget(self.source_combo)
        form = QtWidgets.QFormLayout()
        self.cli_combo = QtWidgets.QComboBox(editable=True)
        self.data_combo = QtWidgets.QComboBox(editable=True)
        for c in (self.cli_combo, self.data_combo):
            setup_port_combo(c)
        form.addRow("CLI port", self.cli_combo)
        form.addRow("Data port", self.data_combo)
        lay.addLayout(form)
        self.detect_btn = QtWidgets.QPushButton("Detect")
        self.detect_btn.clicked.connect(lambda: self._refresh_ports())
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        lay.addLayout(_row(self.detect_btn, self.connect_btn))
        self.cfg_label = QtWidgets.QLabel()
        self.cfg_label.setWordWrap(True)
        self.cfg_label.setStyleSheet("color:#9aa7b6")
        lay.addWidget(self.cfg_label)
        self.start_btn = QtWidgets.QPushButton("▶  Start")
        self.start_btn.setObjectName("primary")
        self.start_btn.clicked.connect(self._start)
        self.stop_btn = QtWidgets.QPushButton("■  Stop")
        self.stop_btn.clicked.connect(self._stop)
        lay.addWidget(self.start_btn)
        lay.addWidget(self.stop_btn)
        sl.addWidget(box)

        box, lay = _group("Settings")
        self.settings_stack = QtWidgets.QStackedWidget()
        lay.addWidget(self.settings_stack)
        self.settings_stack.addWidget(self._drive_settings())
        self.settings_stack.addWidget(self._park_settings())
        self.settings_stack.addWidget(self._cabin_settings())
        self.sound_cb = QtWidgets.QCheckBox("Warning sounds")
        self.sound_cb.setChecked(True)
        self.sound_cb.toggled.connect(lambda on: setattr(self.sounds, "enabled", on))
        lay.addWidget(self.sound_cb)
        sl.addWidget(box)

        box, lay = _group("Session")
        self.rec_dir = QtWidgets.QLineEdit(str(Path.cwd() / "recordings"))
        self.rec_dir.setMinimumWidth(60)
        lay.addWidget(self.rec_dir)
        self.rec_btn = QtWidgets.QPushButton("●  Record")
        self.rec_btn.setObjectName("danger")
        self.rec_btn.setCheckable(True)
        self.rec_btn.toggled.connect(self._toggle_record)
        self.replay_btn = QtWidgets.QPushButton("Replay...")
        self.replay_btn.clicked.connect(self._open_replay)
        lay.addLayout(_row(self.rec_btn, self.replay_btn))
        self.rec_label = QtWidgets.QLabel("")
        self.rec_label.setWordWrap(True)
        self.rec_label.setStyleSheet("color:#9aa7b6")
        lay.addWidget(self.rec_label)
        sl.addWidget(box)
        warn = QtWidgets.QLabel("<span style='color:#7b8794'>Experimental driver information "
                                "only - never rely on it. Keep your eyes on the road.</span>")
        warn.setWordWrap(True)
        sl.addWidget(warn)
        sl.addStretch(1)
        self.sidebar = QtWidgets.QScrollArea()
        self.sidebar.setWidget(side)
        self.sidebar.setWidgetResizable(True)
        self.sidebar.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.split.addWidget(self.sidebar)

        # ------------------------------------------------------- main area
        main_w = QtWidgets.QWidget()
        main = QtWidgets.QVBoxLayout(main_w)
        main.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        self.mode_btns: dict[str, QtWidgets.QPushButton] = {}
        group = QtWidgets.QButtonGroup(self)
        for key, (label, _) in MODES.items():
            b = QtWidgets.QPushButton(label)
            b.setObjectName("mode")
            b.setCheckable(True)
            b.clicked.connect(lambda _=False, k=key: self._set_mode(k, restart=True))
            group.addButton(b)
            bar.addWidget(b)
            self.mode_btns[key] = b
        bar.addStretch(1)
        main.addLayout(bar)
        self.pages = QtWidgets.QStackedWidget()
        main.addWidget(self.pages, 1)
        self.pages.addWidget(self._drive_page())
        self.pages.addWidget(self._park_page())
        self.pages.addWidget(self._cabin_page())
        self.split.addWidget(main_w)
        self.split.setCollapsible(0, True)
        self.split.setCollapsible(1, False)
        self.split.setStretchFactor(1, 1)
        self._panel_width = fit_panel_width(self.sidebar, self.split)
        self.split.setSizes([self._panel_width, 1300])

        btn = QtWidgets.QPushButton("☰ Panel")
        btn.setToolTip("Show / hide the side panel (F2)")
        btn.clicked.connect(self._toggle_panel)
        self.statusBar().addPermanentWidget(btn)
        (getattr(QtGui, "QShortcut", None) or QtWidgets.QShortcut)(
            QtGui.QKeySequence("F2"), self).activated.connect(self._toggle_panel)
        self.statusBar().showMessage("Choose a mode, connect the radar (or the simulator), "
                                     "then press Start.")

    # ---- settings panels ---------------------------------------------
    def _drive_settings(self):
        w = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(w)
        f.setContentsMargins(0, 0, 0, 0)
        self.lane_spin = QtWidgets.QDoubleSpinBox(minimum=2.5, maximum=4.5, singleStep=0.1,
                                                  value=3.5, suffix=" m")
        self.gap_spin = QtWidgets.QDoubleSpinBox(minimum=1.0, maximum=3.0, singleStep=0.1,
                                                 value=2.0, suffix=" s")
        self.timing_combo = QtWidgets.QComboBox()
        self.timing_combo.addItems(["Early", "Normal", "Late"])
        self.timing_combo.setCurrentIndex(1)
        f.addRow("Lane width", self.lane_spin)
        f.addRow("Target gap", self.gap_spin)
        f.addRow("Warnings", self.timing_combo)
        for wd in (self.lane_spin, self.gap_spin):
            wd.valueChanged.connect(self._apply_drive_settings)
        self.timing_combo.currentIndexChanged.connect(self._apply_drive_settings)
        return w

    def _park_settings(self):
        w = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(w)
        f.setContentsMargins(0, 0, 0, 0)
        self.carw_spin = QtWidgets.QDoubleSpinBox(minimum=1.4, maximum=2.6, singleStep=0.1,
                                                  value=1.8, suffix=" m")
        self.mount_combo = QtWidgets.QComboBox()
        self.mount_combo.addItems(["Rear bumper", "Front bumper"])
        f.addRow("Car width", self.carw_spin)
        f.addRow("Radar on", self.mount_combo)
        self.carw_spin.valueChanged.connect(self._apply_park_settings)
        self.mount_combo.currentIndexChanged.connect(self._apply_park_settings)
        return w

    def _cabin_settings(self):
        w = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(w)
        f.setContentsMargins(0, 0, 0, 0)
        self.side_combo = QtWidgets.QComboBox()
        self.side_combo.addItems(["Left-hand drive", "Right-hand drive"])
        self.autoarm_cb = QtWidgets.QCheckBox("Arm when the driver leaves")
        self.autoarm_cb.setChecked(True)
        self.arm_spin = QtWidgets.QSpinBox(minimum=5, maximum=300, value=15, suffix=" s")
        f.addRow("Car", self.side_combo)
        f.addRow(self.autoarm_cb)
        f.addRow("after", self.arm_spin)
        self.side_combo.currentIndexChanged.connect(self._apply_cabin_settings)
        self.autoarm_cb.toggled.connect(self._apply_cabin_settings)
        self.arm_spin.valueChanged.connect(self._apply_cabin_settings)
        return w

    # ---- pages ----------------------------------------------------------
    def _kv(self, grid, row, key):
        k = QtWidgets.QLabel(key)
        k.setObjectName("k")
        v = QtWidgets.QLabel("--")
        v.setObjectName("v")
        grid.addWidget(k, row, 0)
        grid.addWidget(v, row, 1)
        return v

    def _drive_page(self):
        page = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(page)
        h.setContentsMargins(0, 6, 0, 0)
        left = QtWidgets.QVBoxLayout()
        self.view_combo = QtWidgets.QComboBox()
        self.view_combo.addItems(["Driver view (3D)", "Radar map (top view)"])
        self.view_combo.currentIndexChanged.connect(lambda i: self.view_stack.setCurrentIndex(i))
        left.addLayout(_row(self.view_combo, 1))
        self.view_stack = QtWidgets.QStackedWidget()
        self.road = RoadView()
        self.view_stack.addWidget(self.road)
        self.top_plot = pg.PlotWidget()
        self.top_plot.setAspectLocked(False)
        self.top_plot.showGrid(x=True, y=True, alpha=0.15)
        self.top_plot.setLabel("bottom", "x (m)")
        self.top_plot.setLabel("left", "y ahead (m)")
        self.top_plot.setMenuEnabled(False)
        self.top_static = pg.ScatterPlotItem(size=4, brush=pg.mkBrush(100, 116, 139, 180), pen=None)
        self.top_dyn = pg.ScatterPlotItem(size=7, brush=pg.mkBrush("#22d3ee"), pen=None)
        self.top_plot.addItem(self.top_static)
        self.top_plot.addItem(self.top_dyn)
        self._top_items: list = []
        self.view_stack.addWidget(self.top_plot)
        left.addWidget(self.view_stack, 1)
        h.addLayout(left, 3)

        right = QtWidgets.QVBoxLayout()
        self.gauge = Gauge()
        right.addWidget(self.gauge, 2)
        self.drive_banner = QtWidgets.QLabel("Road ahead clear")
        self.drive_banner.setObjectName("banner")
        self.drive_banner.setWordWrap(True)
        right.addWidget(self.drive_banner)
        card = _card()
        g = QtWidgets.QGridLayout(card)
        g.setContentsMargins(14, 10, 14, 10)
        t = QtWidgets.QLabel("VEHICLE AHEAD")
        t.setObjectName("cardTitle")
        g.addWidget(t, 0, 0, 1, 2)
        self.v_dist = self._kv(g, 1, "Distance")
        self.v_rel = self._kv(g, 2, "Relative speed")
        self.v_lead = self._kv(g, 3, "Its speed")
        self.v_gap = self._kv(g, 4, "Time gap")
        self.gap_bar = QtWidgets.QProgressBar()
        self.gap_bar.setRange(0, 300)
        self.gap_bar.setTextVisible(False)
        self.gap_bar.setFixedHeight(8)
        g.addWidget(self.gap_bar, 5, 0, 1, 2)
        self.v_ttc = self._kv(g, 6, "Time to collision")
        right.addWidget(card)
        self.advice = QtWidgets.QLabel("")
        self.advice.setWordWrap(True)
        self.advice.setStyleSheet("font-size:11pt; color:#b8c4d2; padding:2px 4px")
        right.addWidget(self.advice)
        self.events = QtWidgets.QListWidget()
        self.events.setWordWrap(True)
        self.events.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        right.addWidget(QtWidgets.QLabel("Event log"))
        right.addWidget(self.events, 2)
        rw = QtWidgets.QWidget()
        rw.setLayout(right)
        self.events.setMinimumHeight(90)
        h.addWidget(_scroll(rw, 260, 390), 1)
        return page

    def _park_page(self):
        page = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(page)
        h.setContentsMargins(0, 6, 0, 0)
        self.park_view = ParkView()
        h.addWidget(self.park_view, 3)
        right = QtWidgets.QVBoxLayout()
        card = _card()
        v = QtWidgets.QVBoxLayout(card)
        t = QtWidgets.QLabel("NEAREST OBSTACLE")
        t.setObjectName("cardTitle")
        v.addWidget(t)
        self.park_big = QtWidgets.QLabel("--")
        self.park_big.setObjectName("big")
        v.addWidget(self.park_big)
        self.park_zone = QtWidgets.QLabel("")
        self.park_zone.setObjectName("banner")
        v.addWidget(self.park_zone)
        g = QtWidgets.QGridLayout()
        self.p_left = self._kv(g, 0, "Left")
        self.p_centre = self._kv(g, 1, "Centre")
        self.p_right = self._kv(g, 2, "Right")
        self.p_speed = self._kv(g, 3, "Approach speed")
        v.addLayout(g)
        right.addWidget(card)
        tip = QtWidgets.QLabel("Mount the board on the bumper, antennas facing outwards, "
                               "about 50 cm above the ground. The beeps speed up as you get "
                               "closer and become continuous below 30 cm.")
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#9aa7b6")
        right.addWidget(tip)
        right.addStretch(1)
        rw = QtWidgets.QWidget()
        rw.setLayout(right)
        h.addWidget(_scroll(rw, 240, 370), 1)
        return page

    def _cabin_page(self):
        page = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(page)
        h.setContentsMargins(0, 6, 0, 0)
        self.cabin_view = CabinView()
        h.addWidget(self.cabin_view, 2)
        right = QtWidgets.QVBoxLayout()
        self.cpd_banner = QtWidgets.QLabel("Child presence detection idle")
        self.cpd_banner.setObjectName("banner")
        self.cpd_banner.setWordWrap(True)
        right.addWidget(self.cpd_banner)
        self.lock_btn = QtWidgets.QPushButton("\U0001F513  Car unlocked - press to lock")
        self.lock_btn.setObjectName("lock")
        self.lock_btn.setCheckable(True)
        self.lock_btn.setToolTip("Tell Cabin Guard the car has been locked (in a real car this "
                                 "would come from the CAN bus)")
        self.lock_btn.toggled.connect(self._on_lock)
        right.addWidget(self.lock_btn)
        card = _card()
        g = QtWidgets.QGridLayout(card)
        g.setContentsMargins(14, 10, 14, 10)
        t = QtWidgets.QLabel("DRIVER")
        t.setObjectName("cardTitle")
        g.addWidget(t, 0, 0, 1, 2)
        self.c_br = self._kv(g, 1, "Breathing")
        self.c_hr = self._kv(g, 2, "Heart rate")
        self.c_alert = self._kv(g, 3, "Alertness")
        self.c_alert.setWordWrap(True)
        right.addWidget(card)
        card2 = _card()
        v2 = QtWidgets.QVBoxLayout(card2)
        t2 = QtWidgets.QLabel("SEATS")
        t2.setObjectName("cardTitle")
        v2.addWidget(t2)
        self.seat_text = QtWidgets.QLabel("")
        self.seat_text.setTextFormat(QtCore.Qt.TextFormat.RichText)
        v2.addWidget(self.seat_text)
        right.addWidget(card2)
        self.cabin_events = QtWidgets.QListWidget()
        self.cabin_events.setWordWrap(True)
        self.cabin_events.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        right.addWidget(self.cabin_events, 1)
        rw = QtWidgets.QWidget()
        rw.setLayout(right)
        self.cabin_events.setMinimumHeight(90)
        h.addWidget(_scroll(rw, 260, 410), 1)
        return page

    # ============================================================ helpers
    def _toggle_panel(self) -> None:
        sizes = self.split.sizes()
        if sizes[0] > 0:
            self._panel_width = sizes[0]
            self.split.setSizes([0, sum(sizes)])
        else:
            w = max(self._panel_width, fit_panel_width(self.sidebar, self.split))
            self.split.setSizes([w, max(1, sum(sizes) - w)])

    def _on_log(self, text: str) -> None:
        pass  # CLI traffic is not shown in this app; kept for debugging hooks

    def _run_task(self, name, fn) -> None:
        if self.busy:
            return
        self.busy = True
        self._update_controls()

        def work():
            try:
                self.bridge.task_done.emit(name, True, str(fn() or ""))
            except Exception as exc:
                traceback.print_exc()
                self.bridge.task_done.emit(name, False, str(exc))

        threading.Thread(target=work, daemon=True).start()

    def _on_task_done(self, name, ok, msg) -> None:
        self.busy = False
        if ok:
            self.statusBar().showMessage(f"{name}: {msg or 'done'}", 6000)
        else:
            self.statusBar().showMessage(f"{name} failed: {msg}")
            QtWidgets.QMessageBox.warning(self, f"{name} failed", msg)
            if name == "Connect":
                self._disconnect()
        self._update_controls()

    @property
    def device(self) -> RadarDevice | None:
        return self.source if isinstance(self.source, RadarDevice) else None

    def _update_controls(self) -> None:
        connected = self.source is not None
        dev = self.device
        usb = self.source_combo.currentIndex() == 0
        idle = not self.busy
        replay = isinstance(self.source, ReplaySource)
        for w in (self.cli_combo, self.data_combo, self.detect_btn):
            w.setEnabled(not connected and usb)
        self.source_combo.setEnabled(not connected)
        self.connect_btn.setText("Disconnect" if connected else "Connect")
        self.connect_btn.setEnabled(idle)
        self.start_btn.setEnabled(idle and dev is not None and not dev.running)
        self.stop_btn.setEnabled(idle and ((dev is not None and dev.running) or replay))
        self.rec_btn.setEnabled(dev is not None or self.recorder is not None)
        self.replay_btn.setEnabled(not connected or replay)
        for b in self.mode_btns.values():
            b.setEnabled(idle and not replay)

    # =========================================================== modes
    def _cfg_path(self, mode: str | None = None) -> Path:
        return CONFIG_DIR / MODES[mode or self.mode][1]

    def _set_mode(self, mode: str, restart: bool = False) -> None:
        self.mode = mode
        idx = list(MODES).index(mode)
        self.mode_btns[mode].setChecked(True)
        self.pages.setCurrentIndex(idx)
        self.settings_stack.setCurrentIndex(idx)
        params = parse_cfg(load_cfg_file(self._cfg_path()))
        self.cfg_label.setText(f"Config: <b>{self._cfg_path().name}</b><br>"
                               f"{params.frame_rate_hz:.0f} fps · up to {params.max_range_m:.0f} m"
                               f" · ±{params.max_velocity_ms:.1f} m/s")
        self._make_processors(params)
        dev = self.device
        if restart and dev is not None and dev.running:
            self._start()

    def _make_processors(self, params: RadarParams) -> None:
        self.params = params
        self.drive = DriveAssist(params)
        self.park = ParkAssist(params)
        self.cabin = CabinGuard(params)
        self._apply_drive_settings()
        self._apply_park_settings()
        self._apply_cabin_settings()
        self.drive_res = self.park_res = self.cabin_res = None
        self._alert_seen.clear()
        self._last_cabin_update = -1e9

    def _apply_drive_settings(self, *_) -> None:
        if not hasattr(self, "drive"):
            return
        s = self.drive.s
        s.lane_width_m = self.lane_spin.value()
        s.target_gap_s = self.gap_spin.value()
        k = (1.25, 1.0, 0.8)[self.timing_combo.currentIndex()]
        s.ttc_caution_s, s.ttc_warning_s, s.ttc_critical_s = 4.0 * k, 2.6 * k, 1.6 * k

    def _apply_park_settings(self, *_) -> None:
        if hasattr(self, "park"):
            self.park.s.car_width_m = self.carw_spin.value()

    def _apply_cabin_settings(self, *_) -> None:
        if not hasattr(self, "cabin"):
            return
        left = self.side_combo.currentIndex() == 0
        if getattr(self, "_cabin_left", None) != left:
            self._cabin_left = left
            self.cabin.set_seats(default_seats(left))
        self.cabin.auto_arm = self.autoarm_cb.isChecked()
        self.cabin.auto_arm_s = self.arm_spin.value()

    def _on_lock(self, on: bool) -> None:
        self.cabin.locked = on
        self.lock_btn.setText("\U0001F512  Car locked" if on else "\U0001F513  Car unlocked - press to lock")
        self._cabin_event("Car locked" if on else "Car unlocked")

    # ======================================================== connection
    def _refresh_ports(self, cli=None, data=None) -> None:
        ports = list_ports()
        for combo, role, explicit in ((self.cli_combo, "cli", cli), (self.data_combo, "data", data)):
            fill_port_combo(combo, ports)
            match = [i for i, p in enumerate(ports) if p.role == role]
            if explicit:
                combo.setEditText(explicit)
            elif match:
                combo.setCurrentIndex(match[0])

    @staticmethod
    def _combo_port(combo) -> str:
        idx = combo.currentIndex()
        if idx >= 0 and combo.currentText() == combo.itemText(idx):
            return combo.itemData(idx)
        return combo.currentText().split(" - ")[0].split(" · ")[0].strip()

    def _toggle_connect(self) -> None:
        if self.source is not None:
            self._disconnect()
            return
        try:
            if self.source_combo.currentIndex() == 1:
                dev = simulated_auto_device()
            else:
                dev = RadarDevice.connect(self._combo_port(self.cli_combo),
                                          self._combo_port(self.data_combo))
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Connection failed", str(exc))
            return
        self._attach(dev)

        def check():
            res = dev.send_command("version")
            if not res.ok:
                raise RuntimeError(res.response)
            return "radar responded"

        self._run_task("Connect", check)

    def _attach(self, src: FrameSource) -> None:
        self.source = src
        src.add_frame_listener(self._incoming.append)

    def _disconnect(self) -> None:
        if self.rec_btn.isChecked():
            self.rec_btn.setChecked(False)
        src, self.source = self.source, None
        if src is not None:
            src.remove_frame_listener(self._incoming.append)
            threading.Thread(target=src.close, daemon=True).start()
        self._update_controls()

    def _start(self) -> None:
        dev = self.device
        if dev is None:
            return
        lines = load_cfg_file(self._cfg_path())
        self._make_processors(parse_cfg(lines))
        self._incoming.clear()
        self.max_speed = 0.0
        self._session_t0 = time.monotonic()
        name = MODES[self.mode][0].split("  ")[-1]
        self._run_task("Start", lambda: (dev.configure(lines), f"{name} running")[1])

    def _stop(self) -> None:
        if isinstance(self.source, ReplaySource):
            self._disconnect()
        elif self.device:
            self._run_task("Stop", lambda: self.device.stop().response)

    # =================================================== record / replay
    def _toggle_record(self, on: bool) -> None:
        if on:
            if self.device is None:
                self.rec_btn.setChecked(False)
                return
            out = new_recording_dir(self.rec_dir.text() or "recordings")
            self.recorder = Recorder(out, self.device.cfg_lines or load_cfg_file(self._cfg_path()),
                                     save_heatmaps=False)
            self.recorder.attach(self.device)
            cols = {"drive": ["radar_time_s", "ego_speed_kmh", "ego_valid", "lead_distance_m",
                              "lead_rel_speed_ms", "lead_speed_kmh", "time_gap_s", "ttc_s",
                              "fcw_level", "tracks", "alerts"],
                    "park": ["radar_time_s", "nearest_m", "sector", "left_m", "centre_m",
                             "right_m", "closing_ms", "zone"],
                    "cabin": ["radar_time_s", "occupants", "armed", "cpd_level", "cpd_text",
                              "driver_br", "driver_hr", "seats"]}[self.mode]
            self.log = CsvLog(out / f"{self.mode}_log.csv", cols)
            self.rec_btn.setText("■  Stop recording")
        elif self.recorder is not None:
            rec, self.recorder = self.recorder, None
            if self.source is not None:
                rec.detach(self.source)
            s = rec.close()
            if self.log:
                self.log.close()
                self.log = None
            self.rec_btn.setText("●  Record")
            self.rec_label.setText(f"Saved {s['frames']} frames to {s['dir']}")
        self._update_controls()

    def _open_replay(self) -> None:
        if isinstance(self.source, ReplaySource):
            self._disconnect()
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open recording", self.rec_dir.text(),
                                                        "Raw capture (raw_data.bin *.bin)")
        if path:
            self.start_replay(path)

    def start_replay(self, path: str, speed: float = 1.0) -> None:
        src = ReplaySource(path, speed=speed, loop=False)
        if src.params is None:
            QtWidgets.QMessageBox.warning(self, "Replay", "config.cfg not found next to the capture.")
            return
        self.mode = scene_for(src.params)
        idx = list(MODES).index(self.mode)
        self.mode_btns[self.mode].setChecked(True)
        self.pages.setCurrentIndex(idx)
        self.settings_stack.setCurrentIndex(idx)
        self._make_processors(src.params)
        self._attach(src)
        src.start()
        self.rec_label.setText(f"Replaying {path}")
        self._update_controls()

    # ============================================================== loop
    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        if now - self._blink_t > 0.3:
            self._blink_t = now
            for v in (self.road, self.park_view, self.cabin_view):
                v.blink = not v.blink
        frames = []
        while self._incoming:
            frames.append(self._incoming.popleft())
        for f in frames:
            if self.mode == "drive":
                self.drive_res = self.drive.process(f)
                self._log_drive(self.drive_res)
            elif self.mode == "park":
                self.park_res = self.park.process(f)
                self._log_park(self.park_res)
            else:
                self.cabin.add_frame(f)
                t = f.frame_number * self.params.frame_period_ms / 1000
                if t - self._last_cabin_update >= 0.5:
                    self._last_cabin_update = t
                    prev = self.cabin_res
                    self.cabin_res = self.cabin.update()
                    self._cabin_changes(prev, self.cabin_res)
                    self._log_cabin(self.cabin_res)
        if self.mode == "drive" and self.drive_res is not None and frames:
            self._show_drive(self.drive_res)
        elif self.mode == "park" and self.park_res is not None and frames:
            self._show_park(self.park_res)
        elif self.mode == "cabin" and self.cabin_res is not None:
            if frames:
                self._show_cabin(self.cabin_res)
            self.cabin_view.advance(dt)
        self._sounds(now)
        self._status()
        self._update_controls()

    # ------------------------------------------------------------- drive
    def _show_drive(self, r: DriveResult) -> None:
        s = self.drive.s
        self.road.set_result(r, s.lane_width_m, s.max_range_m)
        kmh = r.ego.speed * 3.6
        if r.ego.valid:
            self.max_speed = max(self.max_speed, kmh)
        self.gauge.set_value(max(0.0, kmh), r.ego.valid,
                             f"trip {r.trip_m / 1000:.2f} km · radar odometry")
        if r.lead_distance is not None:
            self.v_dist.setText(f"{r.lead_distance:.1f} m")
            rel = r.lead_rel_speed or 0.0
            self.v_rel.setText(f"{rel * 3.6:+.0f} km/h")
            self.v_rel.setStyleSheet("color:#f87171" if rel < -1.0 else "")
            self.v_lead.setText(f"{r.lead_speed * 3.6:.0f} km/h" if r.lead_speed is not None
                                else "stationary")
        else:
            for w in (self.v_dist, self.v_rel, self.v_lead):
                w.setText("--")
        if r.time_gap_s is not None:
            self.v_gap.setText(f"{r.time_gap_s:.1f} s")
            g = r.time_gap_s
            col = "#4ade80" if g >= s.target_gap_s * 0.9 else "#fbbf24" if g >= 1.0 else "#f87171"
            self.gap_bar.setValue(int(min(3.0, g) * 100))
            self.gap_bar.setStyleSheet(f"QProgressBar::chunk {{ background:{col}; border-radius:3px; }}")
        else:
            self.v_gap.setText("--")
            self.gap_bar.setValue(0)
        self.v_ttc.setText(f"{r.ttc_s:.1f} s" if r.ttc_s is not None and r.ttc_s < 20 else "--")
        if r.alerts:
            a = r.alerts[0]
            col = LEVEL_COLORS[a.level]
            self._banner(self.drive_banner, a.text, col, flash=a.level >= 3)
        else:
            self._banner(self.drive_banner, r.advice or "Road ahead clear", C_OK, subtle=True)
        self.advice.setText(r.advice if r.alerts else "")
        # new alerts -> event log
        codes = {a.code for a in r.alerts}
        for a in r.alerts:
            if a.code not in self._alert_seen:
                self._event(self.events, a.text, LEVEL_COLORS[a.level])
        self._alert_seen = codes
        if self.view_stack.currentIndex() == 1:
            self._draw_top(r)

    def _draw_top(self, r: DriveResult) -> None:
        for it in self._top_items:
            self.top_plot.removeItem(it)
        self._top_items = []
        lw = self.drive.s.lane_width_m
        for xl in (-1.5 * lw, -lw / 2, lw / 2, 1.5 * lw):
            line = pg.PlotCurveItem([xl, xl], [0, self.drive.s.max_range_m],
                                    pen=pg.mkPen((90, 100, 115), width=1,
                                                 style=QtCore.Qt.PenStyle.DashLine))
            self.top_plot.addItem(line)
            self._top_items.append(line)
        if r.points is not None and len(r.points):
            st = r.static_mask
            self.top_static.setData(r.points[st, 0], r.points[st, 1])
            self.top_dyn.setData(r.points[~st, 0], r.points[~st, 1])
        for t in r.tracks:
            col = C_LEAD if t is r.lead else {"oncoming": "#d946ef", "crossing": "#22d3ee",
                                              "stationary": "#64748b"}.get(t.motion, "#3b82f6")
            col = col.name() if hasattr(col, "name") else col
            w, l = (0.6, 0.5) if t.kind == "pedestrian" else (1.8, 4.2)
            rect = QtWidgets.QGraphicsRectItem(t.x[0] - w / 2, t.x[1], w, l)
            pen = pg.mkPen(col, width=2)
            pen.setCosmetic(True)
            rect.setPen(pen)
            rect.setBrush(pg.mkBrush(QtGui.QColor(col).darker(200)))
            self.top_plot.addItem(rect)
            arrow = pg.PlotCurveItem([t.x[0], t.x[0] + t.ground_vx], [t.x[1], t.x[1] + t.ground_vy],
                                     pen=pg.mkPen(col, width=2))
            self.top_plot.addItem(arrow)
            txt = pg.TextItem(f"#{t.id} {t.x[1]:.0f}m", color=col, anchor=(0, 1))
            txt.setPos(t.x[0] + w / 2 + 0.3, t.x[1])
            self.top_plot.addItem(txt)
            self._top_items += [rect, arrow, txt]
        self.top_plot.setXRange(-12, 12, padding=0)
        self.top_plot.setYRange(0, self.drive.s.max_range_m, padding=0)

    # -------------------------------------------------------------- park
    def _show_park(self, r: ParkResult) -> None:
        self.park_view.set_result(r)
        d = r.nearest
        self.park_big.setText("--" if not math.isfinite(d) else
                              f"{d * 100:.0f} cm" if d < 1 else f"{d:.2f} m")
        text = {"clear": "Nothing nearby", "green": "Clear to move", "amber": "Getting close",
                "red": "Very close - slow down", "stop": "STOP"}[r.zone]
        col = {"clear": C_OK, "green": C_OK, "amber": C_WARN, "red": C_ALERT, "stop": C_ALERT}[r.zone]
        self._banner(self.park_zone, text, col, flash=r.zone == "stop")
        for w, k in ((self.p_left, "left"), (self.p_centre, "centre"), (self.p_right, "right")):
            v = r.distances[k]
            w.setText("--" if not math.isfinite(v) else f"{v:.2f} m")
        self.p_speed.setText(f"{r.closing_speed * 100:+.0f} cm/s" if math.isfinite(d) else "--")

    # ------------------------------------------------------------- cabin
    def _show_cabin(self, r: CabinResult) -> None:
        self.cabin_view.set_result(r, self.side_combo.currentIndex() == 0)
        if r.cpd_level:
            col = {1: C_WARN, 2: QtGui.QColor("#fb923c"), 3: C_ALERT}[r.cpd_level]
            self._banner(self.cpd_banner, r.cpd_text, col, flash=r.cpd_level >= 3)
        elif r.armed:
            self._banner(self.cpd_banner, f"Armed ({r.armed_reason}) - cabin empty", C_OK)
        else:
            self._banner(self.cpd_banner, r.status or "Watching the cabin", C_OK, subtle=True)
        d = r.driver
        if d is not None and d.occupied:
            self.c_br.setText(f"{d.breath_bpm:.0f} /min" if d.breath_bpm else "measuring...")
            self.c_hr.setText(f"{d.heart_bpm:.0f} bpm" if d.heart_bpm else "measuring...")
            self.c_alert.setText(r.driver_alertness or "--")
        else:
            self.c_br.setText("--")
            self.c_hr.setText("--")
            self.c_alert.setText("driver seat empty")
        rows = []
        for st in r.seats:
            if st.occupied:
                br = f"{st.breath_bpm:.0f} br/min" if st.breath_bpm else "occupied"
                rows.append(f"<tr><td>{st.seat.label}</td><td>&nbsp;<b style='color:#4ade80'>{br}</b>"
                            f"</td></tr>")
            else:
                rows.append(f"<tr><td style='color:#7b8794'>{st.seat.label}</td>"
                            f"<td style='color:#7b8794'>&nbsp;empty</td></tr>")
        self.seat_text.setText("<table>" + "".join(rows) + "</table>")

    def _cabin_changes(self, prev: CabinResult | None, cur: CabinResult) -> None:
        if prev is None or not prev.seats:
            return
        before = {st.seat.key: st.occupied for st in prev.seats}
        for st in cur.seats:
            if st.seat.key in before and before[st.seat.key] != st.occupied:
                self._cabin_event(f"{st.seat.label}: {'occupied' if st.occupied else 'now empty'}")
        if cur.armed and not prev.armed:
            self._cabin_event(f"Child presence detection armed ({cur.armed_reason})", C_WARN)
        if cur.cpd_level > prev.cpd_level and cur.cpd_level >= 2:
            self._cabin_event(cur.cpd_text, C_ALERT)

    def _cabin_event(self, text, color=None) -> None:
        self._event(self.cabin_events, text, color or QtGui.QColor("#b8c4d2"))

    # ------------------------------------------------------------ common
    def _event(self, widget: QtWidgets.QListWidget, text: str, color) -> None:
        t = time.monotonic() - self._session_t0
        item = QtWidgets.QListWidgetItem(f"{int(t // 60):02d}:{int(t % 60):02d}  {text}")
        item.setForeground(QtGui.QBrush(QtGui.QColor(color)))
        widget.insertItem(0, item)
        while widget.count() > 200:
            widget.takeItem(widget.count() - 1)

    def _banner(self, label: QtWidgets.QLabel, text: str, color, flash: bool = False,
                subtle: bool = False) -> None:
        col = QtGui.QColor(color)
        if subtle:
            bg, fg = "#171e2a", col.name()
        elif flash and self.road.blink:
            bg, fg = "#ffffff", col.darker(150).name()
        else:
            bg, fg = col.darker(260).name(), col.lighter(115).name()
        label.setText(text)
        label.setStyleSheet(f"background:{bg}; color:{fg}; border:1px solid {col.name()};")

    def _sounds(self, now: float) -> None:
        if now < self._next_sound:
            return
        if self.mode == "drive" and self.drive_res is not None and self.source is not None:
            lvl = max([a.level for a in self.drive_res.alerts], default=0)
            if lvl >= 3:
                self.sounds.play("alarm")
                self._next_sound = now + 0.5
            elif lvl == 2:
                self.sounds.play("warn")
                self._next_sound = now + 1.2
        elif self.mode == "park" and self.park_res is not None and self.source is not None:
            iv = self.park_res.beep_interval_s
            if iv is not None:
                self.sounds.play("beep")
                self._next_sound = now + max(0.12, iv)
        elif self.mode == "cabin" and self.cabin_res is not None and self.source is not None:
            if self.cabin_res.cpd_level >= 3:
                self.sounds.play("alarm")
                self._next_sound = now + 1.2
            elif self.cabin_res.cpd_level == 2:
                self.sounds.play("chime")
                self._next_sound = now + 4.0

    def _status(self) -> None:
        if self.source is None:
            return
        parts = [f"{self.source.stats.fps:.0f} fps"]
        sim = getattr(self.device, "simulator", None)
        if sim is not None and sim.truth:
            tr = sim.truth
            if tr.get("scene") == "drive":
                parts.append(f"simulator: {tr['ego_speed'] * 3.6:.0f} km/h true"
                             + (f" · {tr['event']}" if tr["event"] else ""))
            elif tr.get("scene") == "park":
                parts.append(f"simulator: wall at {tr['wall_distance']:.2f} m")
            elif tr.get("scene") == "cabin":
                occ = [k for k, v in tr["occupants"].items() if v != "out"]
                parts.append("simulator: " + (", ".join(occ) or "nobody") + " in the car")
        if self.recorder:
            parts.append(f"recording {self.recorder.frames} frames")
        if not self.statusBar().currentMessage() or self.statusBar().currentMessage().startswith(
                tuple("0123456789")):
            self.statusBar().showMessage(" · ".join(parts))

    # ---------------------------------------------------------------- logs
    def _log_drive(self, r: DriveResult) -> None:
        if self.log:
            self.log.write([r.time_s, r.ego.speed * 3.6, int(r.ego.valid), r.lead_distance,
                            r.lead_rel_speed, None if r.lead_speed is None else r.lead_speed * 3.6,
                            r.time_gap_s, r.ttc_s, r.fcw_level, len(r.tracks),
                            " | ".join(a.text for a in r.alerts)])

    def _log_park(self, r: ParkResult) -> None:
        if self.log:
            fin = lambda v: v if math.isfinite(v) else None  # noqa: E731
            self.log.write([r.time_s, fin(r.nearest), r.nearest_sector, fin(r.distances["left"]),
                            fin(r.distances["centre"]), fin(r.distances["right"]),
                            r.closing_speed, r.zone])

    def _log_cabin(self, r: CabinResult) -> None:
        if self.log and r.seats:
            d = r.driver
            self.log.write([r.time_s, r.occupants, int(r.armed), r.cpd_level, r.cpd_text,
                            d.breath_bpm if d else None, d.heart_bpm if d else None,
                            " ".join(f"{st.seat.key}={'1' if st.occupied else '0'}" for st in r.seats)])

    def closeEvent(self, ev) -> None:  # noqa: N802
        self.timer.stop()
        if self.recorder is not None:
            self.rec_btn.setChecked(False)
        if self.source is not None:
            self.source.close()
        super().closeEvent(ev)


def run_gui(sim: bool = False, mode: str = "drive", cli_port=None, data_port=None,
            replay: str | None = None) -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setApplicationName("AWR1843 Car Radar Suite")
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE + EXTRA_STYLE)
    pg.setConfigOptions(background="#141a24", foreground="#9aa7b6", antialias=True)
    win = AutoWindow(sim=sim, mode=mode, cli_port=cli_port, data_port=data_port)
    if fit_to_screen(win, 1560, 960):
        win.showMaximized()
    else:
        win.show()
    if replay:
        win.start_replay(replay)
    return app.exec()

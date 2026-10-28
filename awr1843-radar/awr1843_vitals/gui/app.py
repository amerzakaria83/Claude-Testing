"""AWR1843 Vital Signs Monitor - main window."""

from __future__ import annotations

import csv
import sys
import threading
import time
import traceback
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from awr1843.config import RadarParams, parse_cfg, read_cfg_lines
from awr1843.device import FrameSource, RadarDevice, list_ports
from awr1843.recorder import Recorder, ReplaySource, new_recording_dir
from awr1843.tlv import Frame

from ..processing import VitalSignsProcessor, VitalsResult, VitalsSettings
from ..recorder import VitalsLog
from ..simulator import simulated_vitals_device
from .widgets import (ALERT, BREATH, HEART, OK, STYLE, WARN, BreathIcon, HeartIcon,
                      MetricCard)

CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"
DEFAULT_CFG = "vitals_20fps.cfg"
UPDATE_MS = 250

pg.setConfigOptions(imageAxisOrder="row-major", antialias=True, background="#141a24",
                    foreground="#9aa7b6")
Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal


class Bridge(QtCore.QObject):
    log = Signal(str)
    task_done = Signal(str, bool, str)


def _group(title: str) -> tuple[QtWidgets.QGroupBox, QtWidgets.QVBoxLayout]:
    box = QtWidgets.QGroupBox(title)
    lay = QtWidgets.QVBoxLayout(box)
    lay.setContentsMargins(10, 14, 10, 10)
    return box, lay


def _row(*widgets) -> QtWidgets.QHBoxLayout:
    lay = QtWidgets.QHBoxLayout()
    for w in widgets:
        lay.addStretch(w) if isinstance(w, int) else lay.addWidget(w)
    return lay


def _plot(glw, row, col, title, ylabel, xlabel="time (s)", **kw):
    p = glw.addPlot(row=row, col=col, title=title, **kw)
    p.setLabel("left", ylabel)
    p.setLabel("bottom", xlabel)
    p.showGrid(x=True, y=True, alpha=0.15)
    p.setMenuEnabled(False)
    for ax in ("left", "bottom"):
        p.getAxis(ax).enableAutoSIPrefix(False)
    return p


class VitalsWindow(QtWidgets.QMainWindow):
    def __init__(self, sim: bool = False, cfg: str | None = None,
                 cli_port: str | None = None, data_port: str | None = None):
        super().__init__()
        self.setWindowTitle("AWR1843 Vital Signs Monitor")
        self.resize(1520, 940)
        self.source: FrameSource | None = None
        self.settings = VitalsSettings()
        self.params: RadarParams = parse_cfg(self._read_cfg(cfg or str(CONFIG_DIR / DEFAULT_CFG)))
        self.processor = VitalSignsProcessor(self.params, self.settings)
        self.recorder: Recorder | None = None
        self.vlog: VitalsLog | None = None
        self.result = VitalsResult()
        self.busy = False
        self._incoming: deque[Frame] = deque(maxlen=2000)
        self._trend: deque[tuple] = deque(maxlen=20000)
        self._session_t0: float | None = None
        self._last_update = 0.0
        self._last_tick = time.monotonic()

        self.bridge = Bridge()
        self.bridge.log.connect(lambda s: self.console.appendPlainText(s.rstrip()))
        self.bridge.task_done.connect(self._on_task_done)

        self._build_ui()
        self._refresh_ports(cli_port, data_port)
        self._load_configs(cfg)
        if sim:
            self.source_combo.setCurrentIndex(1)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(40)
        self._update_controls()

    # ================================================================== UI
    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QHBoxLayout(root)
        outer.setContentsMargins(10, 10, 10, 6)
        outer.setSpacing(10)

        # ------------------------------------------------- setup sidebar
        side = QtWidgets.QWidget()
        sl = QtWidgets.QVBoxLayout(side)
        sl.setContentsMargins(0, 0, 4, 0)
        brand = QtWidgets.QLabel("<span style='font-size:15pt;font-weight:700'>Vital Signs</span>"
                                 "<br><span style='color:#9aa7b6'>AWR1843BOOST mmWave radar</span>")
        sl.addWidget(brand)

        box, lay = _group("Radar")
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.addItems(["AWR1843BOOST (USB)", "Simulator (virtual person)"])
        self.source_combo.currentIndexChanged.connect(self._update_controls)
        lay.addWidget(self.source_combo)
        form = QtWidgets.QFormLayout()
        self.cli_combo = QtWidgets.QComboBox(editable=True)
        self.data_combo = QtWidgets.QComboBox(editable=True)
        form.addRow("CLI port", self.cli_combo)
        form.addRow("Data port", self.data_combo)
        lay.addLayout(form)
        self.detect_btn = QtWidgets.QPushButton("Detect")
        self.detect_btn.clicked.connect(lambda: self._refresh_ports())
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self._toggle_connect)
        lay.addLayout(_row(self.detect_btn, self.connect_btn))
        self.cfg_combo = QtWidgets.QComboBox()
        self.cfg_combo.currentIndexChanged.connect(self._on_cfg_selected)
        browse = QtWidgets.QPushButton("...")
        browse.setFixedWidth(34)
        browse.clicked.connect(self._browse_cfg)
        lay.addLayout(_row(self.cfg_combo, browse))
        self.cfg_info = QtWidgets.QLabel()
        self.cfg_info.setStyleSheet("color:#9aa7b6")
        self.cfg_info.setWordWrap(True)
        lay.addWidget(self.cfg_info)
        self.start_btn = QtWidgets.QPushButton("▶  Start measuring")
        self.start_btn.setObjectName("primary")
        self.start_btn.clicked.connect(self._start)
        self.stop_btn = QtWidgets.QPushButton("■  Stop")
        self.stop_btn.clicked.connect(self._stop)
        lay.addWidget(self.start_btn)
        lay.addWidget(self.stop_btn)
        sl.addWidget(box)

        box, lay = _group("Measurement zone")
        form = QtWidgets.QFormLayout()
        self.rmin = QtWidgets.QDoubleSpinBox(minimum=0.1, maximum=5, singleStep=0.1,
                                             value=self.settings.range_min_m, suffix=" m")
        self.rmax = QtWidgets.QDoubleSpinBox(minimum=0.2, maximum=8, singleStep=0.1,
                                             value=self.settings.range_max_m, suffix=" m")
        self.amax = QtWidgets.QDoubleSpinBox(minimum=5, maximum=80, singleStep=5,
                                             value=self.settings.angle_max_deg, suffix=" °")
        form.addRow("Nearest", self.rmin)
        form.addRow("Farthest", self.rmax)
        form.addRow("Max angle ±", self.amax)
        lay.addLayout(form)
        self.dc_cb = QtWidgets.QCheckBox("Static reflection removal")
        self.dc_cb.setChecked(self.settings.dc_removal)
        lay.addWidget(self.dc_cb)
        for w in (self.rmin, self.rmax, self.amax):
            w.valueChanged.connect(self._apply_settings)
        self.dc_cb.toggled.connect(self._apply_settings)
        sl.addWidget(box)

        box, lay = _group("Session")
        self.rec_dir = QtWidgets.QLineEdit(str(Path.cwd() / "recordings"))
        lay.addWidget(self.rec_dir)
        self.rec_btn = QtWidgets.QPushButton("●  Record")
        self.rec_btn.setObjectName("danger")
        self.rec_btn.setCheckable(True)
        self.rec_btn.toggled.connect(self._toggle_record)
        self.replay_btn = QtWidgets.QPushButton("Replay...")
        self.replay_btn.clicked.connect(self._open_replay)
        lay.addLayout(_row(self.rec_btn, self.replay_btn))
        self.snap_btn = QtWidgets.QPushButton("Save waveforms (CSV)")
        self.snap_btn.clicked.connect(self._save_snapshot)
        lay.addWidget(self.snap_btn)
        self.rec_label = QtWidgets.QLabel("")
        self.rec_label.setWordWrap(True)
        self.rec_label.setStyleSheet("color:#9aa7b6")
        lay.addWidget(self.rec_label)
        sl.addWidget(box)

        tips = QtWidgets.QLabel(
            "<b>For a good reading</b><br>• Sit 0.4–1.2 m in front of the board, "
            "chest facing it<br>• Stay still and breathe normally<br>• First result "
            "after ~10 s still<br><br><span style='color:#7b8794'>Not a medical device. "
            "For experimentation only.</span>")
        tips.setWordWrap(True)
        tips.setStyleSheet("color:#b8c4d2; padding:6px")
        sl.addWidget(tips)
        sl.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(300)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        outer.addWidget(scroll)

        # ---------------------------------------------------- main area
        main = QtWidgets.QVBoxLayout()
        main.setSpacing(10)
        outer.addLayout(main, 1)

        cards = QtWidgets.QHBoxLayout()
        cards.setSpacing(10)
        self.heart_icon = HeartIcon()
        self.heart_card = MetricCard("Heart rate", "bpm", HEART, self.heart_icon)
        self.breath_icon = BreathIcon()
        self.breath_card = MetricCard("Breathing rate", "breaths/min", BREATH, self.breath_icon)
        self.subject_card = MetricCard("Person", "m", OK)
        self.status_card = QtWidgets.QFrame()
        self.status_card.setObjectName("card")
        stl = QtWidgets.QVBoxLayout(self.status_card)
        stl.setContentsMargins(16, 12, 16, 12)
        t = QtWidgets.QLabel("STATUS")
        t.setObjectName("cardTitle")
        stl.addWidget(t)
        self.status_dot = QtWidgets.QLabel("●")
        self.status_text = QtWidgets.QLabel("Not connected")
        self.status_text.setWordWrap(True)
        self.status_text.setStyleSheet("font-size:15pt; font-weight:600")
        stl.addLayout(_row(self.status_dot, self.status_text, 1))
        self.warmup = QtWidgets.QProgressBar()
        self.warmup.setTextVisible(False)
        self.warmup.setFixedHeight(6)
        stl.addWidget(self.warmup)
        self.status_sub = QtWidgets.QLabel("")
        self.status_sub.setObjectName("sub")
        self.status_sub.setWordWrap(True)
        stl.addWidget(self.status_sub)
        stl.addStretch(1)
        for c, stretch in ((self.heart_card, 3), (self.breath_card, 3),
                           (self.subject_card, 2), (self.status_card, 3)):
            c.setMinimumHeight(170)
            cards.addWidget(c, stretch)
        main.addLayout(cards)

        self.tabs = QtWidgets.QTabWidget()
        main.addWidget(self.tabs, 1)
        self.tabs.addTab(self._build_wave_tab(), "Waveforms")
        self.tabs.addTab(self._build_spectra_tab(), "Spectra")
        self.tabs.addTab(self._build_doppler_tab(), "Micro-Doppler && range")
        self.tabs.addTab(self._build_trend_tab(), "Trends")
        self.console = QtWidgets.QPlainTextEdit(readOnly=True)
        self.console.setMaximumBlockCount(3000)
        self.tabs.addTab(self.console, "Radar log")
        self.statusBar().showMessage("Connect the radar or choose the simulator, then press Start.")

    def _build_wave_tab(self):
        glw = pg.GraphicsLayoutWidget()
        p = _plot(glw, 0, 0, "Chest displacement", "mm")
        self.disp_curve = p.plot(pen=pg.mkPen("#c4b5fd", width=1.6))
        self.motion_curve = p.plot(pen=pg.mkPen(WARN, width=6), connect="finite")
        self.disp_plot = p
        p2 = _plot(glw, 1, 0, "Breathing", "mm")
        self.breath_curve = p2.plot(pen=pg.mkPen(BREATH, width=2))
        self.breath_peaks = pg.ScatterPlotItem(size=9, brush=pg.mkBrush("#ffffff"), pen=None,
                                               symbol="t1")
        p2.addItem(self.breath_peaks)
        p3 = _plot(glw, 2, 0, "Heartbeat", "mm")
        self.heart_curve = p3.plot(pen=pg.mkPen(HEART, width=1.6))
        for q in (p2, p3):
            q.setXLink(p)
        p.setXRange(-self.settings.history_s, 0, padding=0)
        return glw

    def _build_spectra_tab(self):
        glw = pg.GraphicsLayoutWidget()
        p = _plot(glw, 0, 0, "Breathing spectrum", "power (norm.)", "breaths / min")
        self.bspec_curve = p.plot(pen=pg.mkPen(BREATH, width=2), fillLevel=0,
                                  brush=pg.mkBrush(79, 209, 255, 50))
        self.bspec_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ffffff", width=1,
                                                                  style=QtCore.Qt.PenStyle.DashLine))
        p.addItem(self.bspec_line)
        p.setXRange(*(np.array(self.settings.breath_band_hz) * 60))
        p2 = _plot(glw, 0, 1, "Heart spectrum (breathing harmonics suppressed)", "power (norm.)",
                   "beats / min")
        self.hspec_curve = p2.plot(pen=pg.mkPen(HEART, width=2), fillLevel=0,
                                   brush=pg.mkBrush(255, 92, 122, 50))
        self.hspec_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ffffff", width=1,
                                                                  style=QtCore.Qt.PenStyle.DashLine))
        p2.addItem(self.hspec_line)
        p2.setXRange(*(np.array(self.settings.heart_band_hz) * 60))
        return glw

    def _build_doppler_tab(self):
        glw = pg.GraphicsLayoutWidget()
        cmap = pg.colormap.get("inferno")
        p = _plot(glw, 0, 0, "Micro-Doppler of the chest (velocity vs time)", "velocity (mm/s)")
        self.md_img = pg.ImageItem()
        self.md_img.setColorMap(cmap)
        p.addItem(self.md_img)
        self.md_plot = p
        p2 = _plot(glw, 1, 0, "Range vs time (person's angle)", "range (m)")
        self.rt_img = pg.ImageItem()
        self.rt_img.setColorMap(pg.colormap.get("viridis"))
        p2.addItem(self.rt_img)
        self.rt_lock = pg.InfiniteLine(angle=0, pen=pg.mkPen(HEART, width=1,
                                                             style=QtCore.Qt.PenStyle.DashLine))
        p2.addItem(self.rt_lock)
        self.rt_plot = p2
        p3 = _plot(glw, 0, 1, "Where is the person? (motion map)", "range (m)", "angle (°)",
                   rowspan=2)
        self.ra_img = pg.ImageItem()
        self.ra_img.setColorMap(cmap)
        p3.addItem(self.ra_img)
        self.ra_mark = pg.ScatterPlotItem(size=16, pen=pg.mkPen(HEART, width=2), brush=None,
                                          symbol="o")
        p3.addItem(self.ra_mark)
        self.ra_plot = p3
        glw.ci.layout.setColumnStretchFactor(0, 3)
        glw.ci.layout.setColumnStretchFactor(1, 2)
        return glw

    def _build_trend_tab(self):
        glw = pg.GraphicsLayoutWidget()
        p = _plot(glw, 0, 0, "Breathing rate", "breaths/min", "session time (min)")
        self.br_trend = p.plot(pen=pg.mkPen(BREATH, width=2), connect="finite")
        p2 = _plot(glw, 1, 0, "Heart rate", "bpm", "session time (min)")
        self.hr_trend = p2.plot(pen=pg.mkPen(HEART, width=2), connect="finite")
        p2.setXLink(p)
        self.sim_truth_br = p.plot(pen=pg.mkPen("#ffffff", width=1, style=QtCore.Qt.PenStyle.DotLine))
        self.sim_truth_hr = p2.plot(pen=pg.mkPen("#ffffff", width=1, style=QtCore.Qt.PenStyle.DotLine))
        return glw

    # ============================================================ helpers
    @staticmethod
    def _read_cfg(path: str) -> list[str]:
        try:
            return read_cfg_lines(Path(path).read_text(encoding="utf-8", errors="replace"))
        except OSError:
            return []

    def _log(self, text: str) -> None:
        self.bridge.log.emit(text)

    def _run_task(self, name: str, fn) -> None:
        if self.busy:
            return
        self.busy = True
        self._update_controls()

        def work():
            try:
                self.bridge.task_done.emit(name, True, str(fn() or ""))
            except Exception as exc:
                self._log(traceback.format_exc())
                self.bridge.task_done.emit(name, False, str(exc))

        threading.Thread(target=work, daemon=True).start()

    def _on_task_done(self, name: str, ok: bool, msg: str) -> None:
        self.busy = False
        if ok:
            self.statusBar().showMessage(f"{name}: {msg or 'done'}", 8000)
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
        self.cfg_combo.setEnabled(not (dev and dev.running))
        self.start_btn.setEnabled(idle and dev is not None and not dev.running)
        self.stop_btn.setEnabled(idle and ((dev is not None and dev.running) or replay))
        self.rec_btn.setEnabled(dev is not None or self.recorder is not None)
        self.replay_btn.setEnabled(not connected or replay)

    # ======================================================== connection
    def _refresh_ports(self, cli: str | None = None, data: str | None = None) -> None:
        ports = list_ports()
        for combo, role, explicit in ((self.cli_combo, "cli", cli), (self.data_combo, "data", data)):
            combo.clear()
            for p in ports:
                combo.addItem(str(p), p.device)
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
        return combo.currentText().split(" - ")[0].strip()

    def _toggle_connect(self) -> None:
        if self.source is not None:
            self._disconnect()
            return
        try:
            if self.source_combo.currentIndex() == 1:
                dev = simulated_vitals_device()
            else:
                dev = RadarDevice.connect(self._combo_port(self.cli_combo),
                                          self._combo_port(self.data_combo))
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self, "Connection failed",
                f"{exc}\n\nCheck the 5 V supply, that the board runs the out-of-box demo in "
                "functional mode (SOP2 on, SOP0 off) and that no other program has the ports open.")
            return
        dev.cli_listeners.append(self._log)
        self._attach(dev)

        def check():
            res = dev.send_command("version")
            if not res.ok:
                raise RuntimeError(res.response)
            return "radar responded"

        self._run_task("Connect", check)

    def _attach(self, source: FrameSource) -> None:
        self.source = source
        source.add_frame_listener(self._incoming.append)
        self._reset_session()

    def _disconnect(self) -> None:
        if self.rec_btn.isChecked():
            self.rec_btn.setChecked(False)
        src, self.source = self.source, None
        if src is not None:
            src.remove_frame_listener(self._incoming.append)
            threading.Thread(target=src.close, daemon=True).start()
        self.result = VitalsResult(status="Not connected")
        self._show_result(self.result)
        self._update_controls()

    # ============================================================ config
    def _load_configs(self, select: str | None) -> None:
        self.cfg_combo.blockSignals(True)
        for path in sorted(CONFIG_DIR.glob("*.cfg")):
            self.cfg_combo.addItem(path.name, str(path))
        if select:
            self.cfg_combo.addItem(Path(select).name, str(Path(select).resolve()))
            self.cfg_combo.setCurrentIndex(self.cfg_combo.count() - 1)
        else:
            self.cfg_combo.setCurrentIndex(max(0, self.cfg_combo.findText(DEFAULT_CFG)))
        self.cfg_combo.blockSignals(False)
        self._on_cfg_selected()

    def _browse_cfg(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open configuration",
                                                        str(CONFIG_DIR), "*.cfg")
        if path:
            self.cfg_combo.addItem(Path(path).name, path)
            self.cfg_combo.setCurrentIndex(self.cfg_combo.count() - 1)

    def _on_cfg_selected(self, *_) -> None:
        lines = self._read_cfg(self.cfg_combo.currentData() or "")
        p = parse_cfg(lines)
        g = p.gui
        warn = []
        if not g.range_azimuth_heatmap:
            warn.append("guiMonitor does not send the range-azimuth heatmap")
        if p.clutter_removal:
            warn.append("clutterRemoval must be 0")
        if p.frame_rate_hz < 10:
            warn.append("low frame rate: fast/deep breathing may read as motion")
        self.cfg_info.setText(
            f"{p.frame_rate_hz:.0f} fps · {p.range_resolution_m * 100:.1f} cm bins · "
            f"up to {p.max_range_m:.1f} m"
            + "".join(f"<br><span style='color:{WARN}'>⚠ {w}</span>" for w in warn))
        self._set_params(p)

    def _set_params(self, p: RadarParams) -> None:
        self.params = p
        self.processor = VitalSignsProcessor(p, self.settings)

    def _apply_settings(self, *_) -> None:
        s = self.settings
        s.range_min_m = self.rmin.value()
        s.range_max_m = max(self.rmax.value(), s.range_min_m + 0.2)
        s.angle_max_deg = self.amax.value()
        s.dc_removal = self.dc_cb.isChecked()
        self.processor._lock = None  # re-search the person in the new zone

    # =========================================================== control
    def _start(self) -> None:
        dev = self.device
        if dev is None:
            return
        lines = self._read_cfg(self.cfg_combo.currentData() or "")
        self._set_params(parse_cfg(lines))
        self._reset_session()

        def go():
            dev.configure(lines)
            return f"running at {self.params.frame_rate_hz:.0f} fps"

        self._run_task("Start", go)

    def _stop(self) -> None:
        if isinstance(self.source, ReplaySource):
            self._disconnect()
        elif self.device:
            self._run_task("Stop", lambda: self.device.stop().response)

    def _reset_session(self) -> None:
        self._incoming.clear()
        self._trend.clear()
        self._session_t0 = None
        self.processor.reset()

    # ====================================================== record/replay
    def _toggle_record(self, on: bool) -> None:
        if on:
            if self.device is None:
                self.rec_btn.setChecked(False)
                return
            out = new_recording_dir(self.rec_dir.text() or "recordings")
            self.recorder = Recorder(out, self.device.cfg_lines or
                                     self._read_cfg(self.cfg_combo.currentData()),
                                     save_heatmaps=False)
            self.recorder.attach(self.device)
            self.vlog = VitalsLog(out / "vitals.csv")
            self.rec_btn.setText("■  Stop recording")
        elif self.recorder is not None:
            rec, self.recorder = self.recorder, None
            if self.source is not None:
                rec.detach(self.source)
            s = rec.close()
            if self.vlog:
                self.vlog.close()
                self.vlog = None
            self.rec_btn.setText("●  Record")
            self.rec_label.setText(f"Saved {s['frames']} frames to {s['dir']}")
        self._update_controls()

    def _open_replay(self) -> None:
        if isinstance(self.source, ReplaySource):
            self._disconnect()
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open recording", self.rec_dir.text(), "Raw capture (raw_data.bin *.bin)")
        if path:
            self.start_replay(path)

    def start_replay(self, path: str, speed: float = 1.0) -> None:
        src = ReplaySource(path, speed=speed, loop=False)
        if src.params is not None:
            self._set_params(src.params)
        self._attach(src)
        src.start()
        self.rec_label.setText(f"Replaying {path}")
        self._update_controls()

    def _save_snapshot(self) -> None:
        r = self.result
        if not len(r.t):
            QtWidgets.QMessageBox.information(self, "Nothing to save", "No waveform data yet.")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save waveforms", str(Path(self.rec_dir.text()) / "waveforms.csv"), "*.csv")
        if not path:
            return
        n = len(r.t)
        cols = {"t_s": r.t, "displacement_mm": r.displacement_mm,
                "breath_mm": r.breath_wave if len(r.breath_wave) == n else np.full(n, np.nan),
                "heart_mm": r.heart_wave if len(r.heart_wave) == n else np.full(n, np.nan),
                "motion": r.motion_mask.astype(int) if len(r.motion_mask) == n else np.zeros(n)}
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(list(cols))
            for row in zip(*cols.values()):
                w.writerow([f"{v:.5f}" if isinstance(v, float) else v for v in row])
        self.statusBar().showMessage(f"Saved {path}", 6000)

    # ============================================================== loop
    def _tick(self) -> None:
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        while self._incoming:
            self.processor.add_frame(self._incoming.popleft())
        if self.source is not None and now - self._last_update >= UPDATE_MS / 1000:
            self._last_update = now
            if self.processor.frames_seen or self.processor.missing_heatmap:
                self.result = self.processor.update()
                self._record_trend(self.result)
                self._show_result(self.result)
                self._draw_plots(self.result)
                if self.vlog:
                    self.vlog.write(self.result)
        self._animate(dt, now)
        self._update_controls()

    def _animate(self, dt: float, now: float) -> None:
        r = self.result
        self.heart_icon.bpm = r.heart_bpm if (r.heart_bpm and not r.motion) else None
        self.heart_icon.advance(dt)
        level = None
        bw = r.breath_wave
        if len(bw) > 10 and np.isfinite(bw[-1]) and not r.motion:
            # play the measured waveform back with a delay of one update period
            fs = self.params.frame_rate_hz
            lag = (now - self._last_update) - UPDATE_MS / 1000
            idx = int(np.clip(len(bw) - 1 + lag * fs, 0, len(bw) - 1))
            recent = bw[-int(10 * fs):]
            recent = recent[np.isfinite(recent)]
            lo, hi = np.min(recent), np.max(recent)
            if hi - lo > 1e-3 and np.isfinite(bw[idx]):
                level = float((bw[idx] - lo) / (hi - lo))
        self.breath_icon.set_level(level)

    def _record_trend(self, r: VitalsResult) -> None:
        if self._session_t0 is None:
            self._session_t0 = time.time()
        tmin = (time.time() - self._session_t0) / 60
        truth = getattr(self.device, "simulator", None)
        tb = th = np.nan
        if truth is not None and truth.truth:
            tb = truth.truth.get("breath_bpm") or np.nan
            th = truth.truth.get("heart_bpm") or np.nan
        self._trend.append((tmin, r.breath_bpm if r.breath_bpm else np.nan,
                            r.heart_bpm if r.heart_bpm else np.nan, tb, th))

    def _show_result(self, r: VitalsResult) -> None:
        # heart
        self.heart_card.set_value(f"{r.heart_bpm:.0f}" if r.heart_bpm else None,
                                  dim=r.heart_confidence < 0.25)
        self.heart_card.conf.set_value(r.heart_confidence if r.heart_bpm else 0)
        self.heart_card.sub.setText(f"confidence {r.heart_confidence * 100:.0f} %"
                                    if r.heart_bpm else "waiting for a still segment")
        # breath
        self.breath_card.set_value(f"{r.breath_bpm:.0f}" if r.breath_bpm else None,
                                   dim=r.breath_confidence < 0.3)
        self.breath_card.conf.set_value(r.breath_confidence if r.breath_bpm else 0)
        bits = []
        if r.breath_depth_mm:
            bits.append(f"depth {r.breath_depth_mm:.1f} mm")
        if r.breath_interval_s:
            bits.append(f"last breath {r.breath_interval_s:.1f} s")
        if r.apnea_s >= 6:
            bits = [f"<span style='color:{ALERT}'>no breathing for {r.apnea_s:.0f} s</span>"]
        self.breath_card.sub.setText(" · ".join(bits))
        # person
        if r.distance_m is not None and r.present:
            self.subject_card.set_value(f"{r.distance_m:.2f}")
            self.subject_card.sub.setText(
                f"angle {r.angle_deg:+.0f}° · still {r.still_for_s:.0f} s")
        else:
            self.subject_card.set_value(None)
            self.subject_card.sub.setText("nobody in the measurement zone")
        q = 10 * np.log10(max(r.signal_quality, 1.0))
        self.subject_card.conf.set_value(min(1.0, q / 40))
        # status
        if r.apnea or r.apnea_s >= 6:
            col = ALERT
        elif r.motion or not r.present or r.warmup_left_s > 0:
            col = WARN
        elif "Measuring" in r.status:
            col = OK
        else:
            col = "#7b8794"
        self.status_dot.setStyleSheet(f"color:{col}; font-size:18pt")
        self.status_text.setText(r.status)
        total = self.settings.min_breath_s
        self.warmup.setValue(int(100 * (1 - min(total, r.warmup_left_s) / total))
                             if r.present else 0)
        sub = []
        if self.source is not None:
            sub.append(f"{self.source.stats.fps:.0f} fps · signal {q:.0f} dB")
            if self.source.stats.missed_frames:
                sub.append(f"{self.source.stats.missed_frames} frames lost")
        sim = getattr(self.device, "simulator", None)
        if sim is not None and sim.truth:
            tb = sim.truth.get("breath_bpm")
            sub.append(f"simulator truth: BR {tb:.1f}" if tb else "simulator truth: breath hold")
            sub[-1] += f", HR {sim.truth['heart_bpm']:.1f}"
        self.status_sub.setText(" · ".join(sub))
        if self.recorder:
            self.rec_label.setText(f"Recording to {self.recorder.out_dir}\n"
                                   f"{self.recorder.frames} frames")

    def _draw_plots(self, r: VitalsResult) -> None:
        t = r.t
        if len(t) and len(r.displacement_mm) == len(t):
            d = r.displacement_mm
            self.disp_curve.setData(t, d)
            if len(r.motion_mask) == len(t) and r.motion_mask.any():
                top = np.nanmax(d) + 0.05 * (np.ptp(d) + 1)
                self.motion_curve.setData(t, np.where(r.motion_mask, top, np.nan))
            else:
                self.motion_curve.setData([], [])
        for curve, wave in ((self.breath_curve, r.breath_wave), (self.heart_curve, r.heart_wave)):
            if len(wave) == len(t) and len(t):
                curve.setData(t, wave, connect="finite")
            else:
                curve.setData([], [])
        if len(r.breath_peaks_t) and len(r.breath_wave) == len(t):
            self.breath_peaks.setData(r.breath_peaks_t, np.interp(r.breath_peaks_t, t, r.breath_wave))
        else:
            self.breath_peaks.setData([], [])

        if self.tabs.currentIndex() == 1:
            for spec, curve, line, val in ((r.breath_spectrum, self.bspec_curve, self.bspec_line,
                                            r.breath_bpm),
                                           (r.heart_spectrum, self.hspec_curve, self.hspec_line,
                                            r.heart_bpm)):
                if spec is not None:
                    curve.setData(*spec)
                line.setVisible(bool(val))
                if val:
                    line.setValue(val)
        if self.tabs.currentIndex() == 2:
            self._draw_maps(r)
        if self.tabs.currentIndex() == 3 and self._trend:
            a = np.array(self._trend, float)
            self.br_trend.setData(a[:, 0], a[:, 1])
            self.hr_trend.setData(a[:, 0], a[:, 2])
            if np.isfinite(a[:, 4]).any():
                self.sim_truth_br.setData(a[:, 0], a[:, 3], connect="finite")
                self.sim_truth_hr.setData(a[:, 0], a[:, 4], connect="finite")

    def _draw_maps(self, r: VitalsResult) -> None:
        if r.doppler is not None and r.doppler.size and r.doppler_t is not None and len(r.doppler_t):
            img = r.doppler
            v = r.doppler_axis_mm_s
            t0 = r.doppler_t[0] if len(r.doppler_t) > 1 else -1
            dtt = (r.doppler_t[-1] - t0) / max(1, len(r.doppler_t) - 1) or 1
            dv = v[1] - v[0] if len(v) > 1 else 1
            self.md_img.setImage(img, levels=(np.percentile(img, 30), img.max()))
            self.md_img.setRect(QtCore.QRectF(t0 - dtt / 2, v[0] - dv / 2,
                                              dtt * len(r.doppler_t), dv * len(v)))
        if r.range_time is not None and len(r.t):
            img = r.range_time.T
            rax = r.range_axis_m
            dr = rax[1] - rax[0] if len(rax) > 1 else 1
            self.rt_img.setImage(img, levels=(np.percentile(img, 20), img.max()))
            self.rt_img.setRect(QtCore.QRectF(r.t[0], -dr / 2, r.t[-1] - r.t[0], dr * len(rax)))
            if r.distance_m is not None:
                self.rt_lock.setValue(r.distance_m)
        if r.range_angle is not None:
            var = 10 * np.log10(r.range_angle + 1e-9)
            angles = self.processor._angles()
            res = self.params.range_resolution_m
            r0 = max(1, int(np.floor(self.settings.range_min_m / res)))
            # angle bins are non-uniform in degrees; draw them on a sin(theta) grid
            self.ra_img.setImage(var, levels=(np.percentile(var, 50), var.max()))
            n_a = var.shape[1]
            self.ra_img.setRect(QtCore.QRectF(-90, (r0 - 0.5) * res, 180, var.shape[0] * res))
            self.ra_plot.getAxis("bottom").setTicks([[(-90 + 180 * (k + 0.5) / n_a,
                                                       f"{angles[k]:.0f}")
                                                      for k in range(0, n_a, 4)]])
            if r.distance_m is not None and r.angle_deg is not None:
                k = int(np.argmin(np.abs(angles - r.angle_deg)))
                self.ra_mark.setData([-90 + 180 * (k + 0.5) / n_a], [r.distance_m])

    def closeEvent(self, ev) -> None:  # noqa: N802
        self.timer.stop()
        if self.recorder is not None:
            self.rec_btn.setChecked(False)
        if self.source is not None:
            self.source.close()
        super().closeEvent(ev)


def run_gui(sim: bool = False, cfg: str | None = None, cli_port: str | None = None,
            data_port: str | None = None, replay: str | None = None) -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setApplicationName("AWR1843 Vital Signs Monitor")
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    win = VitalsWindow(sim=sim, cfg=cfg, cli_port=cli_port, data_port=data_port)
    win.show()
    if replay:
        win.start_replay(replay)
    return app.exec()

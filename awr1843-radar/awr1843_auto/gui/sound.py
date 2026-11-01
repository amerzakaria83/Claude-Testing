"""Warning sounds (generated tones; falls back to the system beep)."""

from __future__ import annotations

import math
import struct
import tempfile
import wave
from pathlib import Path

from pyqtgraph.Qt import QtCore, QtWidgets

TONES = {
    "beep": [(1400, 0.07)],                       # parking sensor
    "chime": [(880, 0.12), (1320, 0.18)],         # caution
    "warn": [(1700, 0.09), (0, 0.04), (1700, 0.09)],
    "alarm": [(950, 0.22), (650, 0.22)],          # collision / child left behind
}


def _write_wav(path: Path, parts, rate: int = 44100) -> None:
    frames = bytearray()
    for freq, dur in parts:
        n = int(rate * dur)
        for i in range(n):
            env = min(1.0, i / 200, (n - i) / 200)
            v = 0.0 if freq == 0 else 0.45 * env * math.sin(2 * math.pi * freq * i / rate)
            frames += struct.pack("<hh", int(v * 32767), int(v * 32767))   # stereo
    with wave.open(str(path), "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(bytes(frames))


class Sounds:
    def __init__(self) -> None:
        self.enabled = True
        self._effects = {}
        try:
            from PySide6.QtMultimedia import QSoundEffect  # type: ignore
        except Exception:  # pragma: no cover - multimedia missing
            QSoundEffect = None
        folder = Path(tempfile.gettempdir()) / "awr1843_auto_sounds_v2"
        folder.mkdir(exist_ok=True)
        for name, parts in TONES.items():
            path = folder / f"{name}.wav"
            if not path.exists():
                _write_wav(path, parts)
            if QSoundEffect is not None:
                try:
                    eff = QSoundEffect()
                    eff.setSource(QtCore.QUrl.fromLocalFile(str(path)))
                    eff.setVolume(0.8)
                    self._effects[name] = eff
                except Exception:  # pragma: no cover
                    pass

    def play(self, name: str) -> None:
        if not self.enabled:
            return
        eff = self._effects.get(name)
        if eff is not None:
            eff.play()
        else:
            QtWidgets.QApplication.beep()

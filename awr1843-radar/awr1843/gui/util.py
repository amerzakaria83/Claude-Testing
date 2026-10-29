"""Window helpers shared by the apps."""

from __future__ import annotations

from pyqtgraph.Qt import QtWidgets


def fit_to_screen(win: QtWidgets.QWidget, pref_w: int, pref_h: int, margin: float = 0.92) -> bool:
    """Size ``win`` to its preferred size, but never larger than the screen.

    Returns True when the screen is smaller than the preferred size; the
    caller should then show the window maximised.  Sizes are in logical
    pixels, so Windows display scaling (125 %, 150 %...) is accounted for.
    """
    screen = win.screen() or QtWidgets.QApplication.primaryScreen()
    if screen is None:
        win.resize(pref_w, pref_h)
        return False
    avail = screen.availableGeometry()
    w = min(pref_w, int(avail.width() * margin))
    h = min(pref_h, int(avail.height() * margin))
    win.resize(w, h)
    win.move(avail.x() + (avail.width() - w) // 2, avail.y() + (avail.height() - h) // 2)
    return pref_w > avail.width() * margin or pref_h > avail.height() * margin

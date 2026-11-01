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


def port_label(p) -> str:
    """Short combo-box text for a serial port, e.g. 'COM3 · radar CLI'."""
    if p.role == "cli":
        return f"{p.device} · radar CLI"
    if p.role == "data":
        return f"{p.device} · radar data"
    desc = p.description if p.description and p.description != "n/a" else ""
    desc = desc.replace(f"({p.device})", "").strip()
    return f"{p.device} · {desc[:18]}" if desc else p.device


def setup_port_combo(combo: QtWidgets.QComboBox) -> None:
    """Keep port combos narrow; the full description is in tooltip / popup."""
    combo.setSizeAdjustPolicy(
        QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
    combo.setMinimumContentsLength(10)
    combo.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding,
                        QtWidgets.QSizePolicy.Policy.Fixed)


def fill_port_combo(combo: QtWidgets.QComboBox, ports) -> None:
    from pyqtgraph.Qt import QtCore

    combo.clear()
    for p in ports:
        combo.addItem(port_label(p), p.device)
        combo.setItemData(combo.count() - 1, f"{p.device} - {p.description}",
                          QtCore.Qt.ItemDataRole.ToolTipRole)
    view = combo.view()
    view.setMinimumWidth(max(combo.width(), view.sizeHintForColumn(0) + 30))


def fit_panel_width(scroll: QtWidgets.QScrollArea, splitter: QtWidgets.QSplitter,
                    extra: int = 0) -> int:
    """Make a side panel at least as wide as its contents so nothing is cut
    off behind the neighbouring widget.  Returns the width used."""
    content = scroll.widget()
    sb = scroll.verticalScrollBar().sizeHint().width()
    need = content.minimumSizeHint().width() + sb + 6 + extra
    scroll.setMinimumWidth(need)
    if scroll.maximumWidth() < need:
        scroll.setMaximumWidth(need)
    sizes = splitter.sizes()
    total = sum(sizes) or splitter.width()
    if sizes and 0 < sizes[0] < need:
        splitter.setSizes([need, max(1, total - need)])
    return need

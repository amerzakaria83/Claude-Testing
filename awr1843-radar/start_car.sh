#!/usr/bin/env sh
# Set up (first run only) and launch the AWR1843 Car Radar Suite on Linux / macOS.
set -e
cd "$(dirname "$0")"
if [ ! -d .venv ]; then
    echo "Creating Python environment, this happens only once..."
    python3 -m venv .venv
    .venv/bin/python -m pip install --upgrade pip
fi
if ! .venv/bin/python -c "import numpy, serial, pyqtgraph, PySide6, scipy" 2>/dev/null; then
    echo "Installing required packages..."
    .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python -m awr1843_auto gui "$@"

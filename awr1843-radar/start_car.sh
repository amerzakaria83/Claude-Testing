#!/usr/bin/env sh
# Set up and launch the AWR1843 Car Radar Suite on Linux / macOS.
# Creates .venv on the first run; rebuilds it if its Python has disappeared.
set -e
cd "$(dirname "$0")"
if [ -d .venv ] && ! .venv/bin/python -c "import sys" >/dev/null 2>&1; then
    echo "The Python environment in .venv no longer works - rebuilding it..."
    rm -rf .venv
fi
if [ ! -d .venv ]; then
    echo "Creating the Python environment, this happens only once..."
    for py in python3.12 python3.11 python3.13 python3.14 python3; do
        if command -v "$py" >/dev/null 2>&1 && "$py" -m venv .venv; then
            break
        fi
    done
    [ -x .venv/bin/python ] || { echo "Python 3.11 or 3.12 is required."; exit 1; }
    .venv/bin/python -m pip install --upgrade pip
fi
if ! .venv/bin/python -c "import numpy, serial, pyqtgraph, PySide6, scipy" 2>/dev/null; then
    echo "Installing required packages, this can take a few minutes..."
    .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python -m awr1843_auto gui "$@"

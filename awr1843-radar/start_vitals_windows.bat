@echo off
rem Double-click to set up (first run only) and launch the AWR1843 Vital Signs Monitor.
cd /d "%~dp0"
if not exist .venv (
    echo Creating Python environment, this happens only once...
    py -3 -m venv .venv || python -m venv .venv || (echo Python 3.9+ is required: https://www.python.org/downloads/ & pause & exit /b 1)
    .venv\Scripts\python -m pip install --upgrade pip
)
.venv\Scripts\python -c "import numpy, serial, pyqtgraph, PySide6, scipy" 2>nul || (
    echo Installing required packages...
    .venv\Scripts\python -m pip install -r requirements.txt || (pause & exit /b 1)
)
.venv\Scripts\python -m awr1843_vitals gui %*
if errorlevel 1 pause

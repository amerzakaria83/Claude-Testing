@echo off
rem Double-click to set up and launch the AWR1843 Vital Signs Monitor.
rem The first run creates a private Python environment in the .venv folder and
rem installs the packages. If the Python that .venv was made with has been
rem removed or upgraded, .venv is rebuilt automatically.
setlocal
cd /d "%~dp0"

if not exist .venv\Scripts\python.exe goto create
.venv\Scripts\python.exe -c "import sys" >nul 2>&1
if not errorlevel 1 goto packages
echo The Python environment in .venv no longer works (its Python was removed or upgraded).
echo Rebuilding it...
rmdir /s /q .venv
if exist .venv (
    echo Could not delete the .venv folder. Close all radar apps and run this file again.
    pause
    exit /b 1
)

:create
echo Creating the Python environment, this happens only once...
for %%V in (3.12 3.11 3.13 3.14 3) do (
    py -%%V -m venv .venv >nul 2>&1 && goto created
)
python -m venv .venv >nul 2>&1 && goto created
echo.
echo Python was not found. Install Python 3.12 from https://www.python.org/downloads/
echo and tick "Add python.exe to PATH" in the installer, then run this file again.
pause
exit /b 1

:created
.venv\Scripts\python.exe -c "import sys; print('Using Python', sys.version.split()[0])"
.venv\Scripts\python.exe -m pip install --upgrade pip

:packages
.venv\Scripts\python.exe -c "import numpy, serial, pyqtgraph, PySide6, scipy" >nul 2>&1 && goto run
echo Installing required packages, this can take a few minutes...
.venv\Scripts\python.exe -m pip install -r requirements.txt
if not errorlevel 1 goto run

rem Installing failed. Brand-new Python versions (e.g. 3.15) may not have the
rem packages yet: switch to Python 3.12 / 3.11 if one is installed as well.
.venv\Scripts\python.exe -c "import sys; sys.exit(0 if sys.version_info[:2] in ((3, 11), (3, 12)) else 1)" && goto failed
for %%V in (3.12 3.11) do (
    py -%%V -c "import sys" >nul 2>&1 && goto rebuild
)
goto failed

:rebuild
echo.
echo The packages are not available for this Python version yet.
echo Switching to the Python 3.12 / 3.11 that is also installed...
rmdir /s /q .venv
goto create

:failed
echo.
echo Installing the packages failed. Your Python version is probably too new for
echo some of them. Install Python 3.12 from https://www.python.org/downloads/
echo (it can be installed next to your current Python), then run this file again.
pause
exit /b 1

:run
.venv\Scripts\python.exe -m awr1843_vitals gui %*
if errorlevel 1 pause

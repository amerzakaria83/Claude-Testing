# How to run the apps (Windows step-by-step)

This guide covers both apps:

- **Vital Signs Monitor:** heart and breathing rate. Start it with `start_vitals_windows.bat`.
- **Radar Studio:** general radar control and data. Start it with `start_windows.bat`.
- **Car Radar Suite:** Drive Assist, Park Assist and Cabin Guard. Start it with
  `start_car_windows.bat`. See [awr1843_auto/README.md](awr1843_auto/README.md).

Linux and macOS users run `./start_vitals.sh`, `./start.sh` and `./start_car.sh` instead.

---

## 1. Get the latest code

The apps live on the branch **`ccr-0802a93c-6izo2z`** of `amerzakaria83/ac-animations-pages`.
They are not on `main` yet.

**If you cloned with git:**

```bat
cd path\to\ac-animations-pages
git fetch origin
git checkout ccr-0802a93c-6izo2z
git pull
```

**If you downloaded a ZIP:** on GitHub, switch the branch selector to `ccr-0802a93c-6izo2z`,
then choose *Code → Download ZIP* again and unzip it to a new folder. An old copy has the old
window layout and the old vitals configuration, which would explain "not working".

Everything is in the **`awr1843-radar`** folder.

## 2. Install Python (one time)

1. Install **Python 3.11 or 3.12** from <https://www.python.org/downloads/>. Brand-new Python
   versions can lack ready-made packages for the Qt user-interface library.
2. In the installer, tick **"Add python.exe to PATH"**.

## 3. Try it without the radar first

1. Open the `awr1843-radar` folder and double-click **`start_vitals_windows.bat`**.
   - The first start creates a `.venv` folder and downloads the packages, which takes 1–3
     minutes. Later starts take a few seconds.
   - If you used Radar Studio before, the launcher only adds the one missing package (scipy).
2. In the app, set the source list (top left) to **Simulator (virtual person)**.
3. Click **Connect**, then **Start measuring**.
4. Expected result:
   - *Status* shows "Measuring... 10 s" and counts down.
   - After about 10–15 s, the **heart rate** (about 66–78) and **breathing rate** (about
     12–18) appear. The status card also shows the simulator's true values for comparison.
   - Every 2½ minutes, the simulated person moves for 4 s ("Motion detected") and later holds
     their breath ("Breathing paused" / "No breathing detected").

If the simulator works, the software is installed correctly.

## 4. Prepare the board

| Check | How |
|---|---|
| Firmware | The out-of-box demo `xwr18xx_mmw_demo.bin` (mmWave SDK 3.5 / 3.6) is flashed. It's the same one that works with Radar Studio. |
| Mode switches | **SOP2 on, SOP0 off, SOP1 off** (functional mode) |
| Power | The **5 V barrel supply** is plugged in, as well as USB |
| Reset | Press **NRST** after power-up or after changing switches |
| Ports | Device Manager shows *XDS110 Class Application/User UART (COMx)* and *XDS110 Class Auxiliary Data Port (COMy)* |
| Nothing else is using the ports | Close Radar Studio, the TI visualizer, UniFlash and any serial terminals. **Only one program can use the radar at a time.** |

## 5. Measure

1. Start the Vital Signs Monitor and leave the source as **AWR1843BOOST (USB)**.
2. Click **Detect**. The CLI and Data ports fill in automatically. If they don't, choose them
   by hand: CLI is the "Application/User UART" port and Data is the "Auxiliary Data Port".
3. Click **Connect**. The status bar should say "Connect: radar responded".
4. Keep **`vitals_15fps.cfg`** selected and click **Start measuring**.
5. Sit **0.5–1 m** in front of the board with the board at chest height and the antennas
   facing you. Keep your hands still, don't talk, and breathe normally.
6. After about 10 s of stillness, the breathing rate appears. The heart rate follows. Its
   confidence bar shows how reliable it is (below 25 % it's dimmed).

**Recording:** click **Record** before or during a measurement. The files go to
`awr1843-radar\recordings\<date_time>\`. Send me `raw_data.bin` and `config.cfg` if you want me
to tune the processing on your real data.

## 6. Window size and small screens

- The window opens at a size that fits your screen. On small screens it opens maximised.
- **☰ Setup panel** (bottom right) or **F2** hides and shows the left panel. On a narrow
  window, the panel hides automatically once measuring starts, so the readings get the space.
- You can drag the divider between the panel and the dashboard.
- On a narrow window the four cards switch to a 2 × 2 grid, and all numbers scale with the
  window.
- If everything still looks too big, lower Windows' display scaling (*Settings → System →
  Display → Scale*, for example from 150 % to 125 %).

## 7. If something goes wrong

**See the actual error message:** open a Command Prompt in the `awr1843-radar` folder and run:

```bat
.venv\Scripts\python -m awr1843_vitals gui
```

Any error is printed in that window. Copy it and send it to me.

| Problem | What to do |
|---|---|
| Double-clicking the `.bat` flashes a window and closes it | Run it from a Command Prompt (see above) to read the message. Usually Python isn't installed or isn't on PATH. |
| `No module named ...` | Delete the `.venv` folder and double-click the `.bat` again to reinstall. |
| *Connection failed* / *Access denied* | Another program has the port open. Close it, unplug and replug USB, then try again. |
| *Connect failed: no response* | No 5 V supply, the wrong SOP switches, or the CLI and data ports are swapped. Press NRST and try again. |
| *Start failed: Command failed: ...* | The firmware rejected a configuration line. The **Radar log** tab shows which one. Select **`vitals_safe_8fps.cfg`**, click Start again, and send me the log. |
| Status keeps saying *No person detected* | Move closer (0.5–1 m) and face the board. Check that *Measurement zone* (nearest/farthest) covers your distance. |
| Status keeps saying *Motion detected* | Sit still, rest your arms and don't talk. With the 8 fps config, deep or fast breathing also counts as motion, so breathe calmly. |
| Heart rate jumps around or has low confidence | Sit closer and straight on, stay still for 30 s or more, and breathe slowly. The heartbeat moves your chest only about 0.1–0.5 mm. |
| No data at all after Start | Check the Data port choice. *Radar log* should show `Done` after every command. |

## 8. Terminal mode (no window)

```bat
.venv\Scripts\python -m awr1843_vitals run --sim --duration 30
.venv\Scripts\python -m awr1843_vitals run --cli-port COM3 --data-port COM4
.venv\Scripts\python -m awr1843_vitals run --cfg awr1843_vitals\configs\vitals_safe_8fps.cfg
```

The first command uses the simulator. The other two use the board: replace COM3 / COM4 with
your port numbers. The terminal prints the breathing rate, heart rate, distance and status
every second.

## 9. Car Radar Suite in short

1. Double-click **`start_car_windows.bat`** and pick a mode at the top (Drive / Park /
   Cabin).
2. Try **Simulator (virtual car)** → Connect → Start first. Every mode has a scripted scene:
   in Drive Assist, watch the collision warning at about 70 s of the drive.
3. With the board, choose AWR1843BOOST (USB) → Detect → Connect → Start. Each mode sends its
   own configuration; switching modes reconfigures the radar.
4. **Mounting:**
   - Drive: behind the windscreen or on the bumper, looking forward.
   - Park: on a bumper, looking outward.
   - Cabin: at the mirror, looking into the cabin.
5. **For experiments only.** Have a passenger operate the laptop, and never rely on the
   warnings while driving.

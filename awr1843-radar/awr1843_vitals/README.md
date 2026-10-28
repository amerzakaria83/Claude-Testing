# AWR1843 Vital Signs Monitor

Measures **heart rate** and **breathing rate** without contact, using the AWR1843BOOST. The
radar sits in front of a seated person. The app also reports breathing depth,
breath-by-breath intervals, pauses in breathing (apnea), body movement, and the person's
distance and angle.

It uses the **same out-of-box demo firmware** as AWR1843 Radar Studio, so you don't need to
reflash the board.

> ⚠️ **Not a medical device.** This is for experiments and learning only. Don't use it for
> diagnosis or monitoring anyone's health.

## Quick start

1. Board setup is the same as for Radar Studio: 5 V supply, out-of-box demo flashed, and
   functional mode (**SOP2 on, SOP0 off**).
2. **Windows:** double-click `start_vitals_windows.bat`. **Linux / macOS:** run
   `./start_vitals.sh`. Or run `python -m awr1843_vitals` after
   `pip install -r requirements.txt`.
3. Click *Connect*, keep **`vitals_20fps.cfg`** selected, and click **Start measuring**.
4. Sit **0.4–1.2 m** in front of the board with your chest facing it. Stay still and breathe
   normally. The first readings appear after about **10 s** of stillness.

Pick *Simulator (virtual person)* in the source list to try the app without the board. It
simulates a person whose true breathing and heart rates are known. The app shows those true
values next to its estimates, and every 150 s the simulated person moves and then holds their
breath, so you can see motion and apnea detection work.

## Positioning tips

- Put the radar at chest height, 0.5–1 m away, pointing at the sternum. Any position inside
  the radar's ±60° field of view works, but straight on is best.
- Clothing is fine because radar passes through fabric. Thick, stiff coats weaken the signal.
- Only one person should be in the **measurement zone** (sidebar: nearest/farthest distance
  and max angle). Narrow the zone if other people or moving objects are nearby.
- Don't put fans, curtains or moving machinery behind the person.
- Talking and fidgeting get flagged as motion. The readings pause until you're still again.

## What you see

| Panel | Content |
|---|---|
| **Heart rate** card | beats/min. The heart animates at the measured rate. The bar is the spectral confidence. |
| **Breathing rate** card | breaths/min, breath depth (mm of chest movement) and the last breath interval. The circle follows your actual measured breathing. |
| **Person** card | distance (m), angle, seconds held still, signal quality |
| **Status** | Measuring / Measuring… countdown / Motion detected / Breathing paused / No person |
| *Waveforms* tab | raw chest displacement with motion marked in amber, the filtered breathing wave with detected breaths, and the heartbeat wave |
| *Spectra* tab | breathing and heart spectra with the chosen peak. Breathing harmonics are notched out of the heart spectrum. |
| *Micro-Doppler & range* tab | chest velocity spectrogram, range vs time at the person's angle, and the range/angle motion map with the tracked person circled |
| *Trends* tab | breathing and heart rate over the whole session. In simulator mode the true values are drawn as dotted lines. |
| *Radar log* | all CLI traffic with the board |

**Recording:** *Record* saves `raw_data.bin` (replayable through the same views with
*Replay…*), `config.cfg` and `vitals.csv` (one row of estimates every 0.25 s). *Save
waveforms* exports the current 30 s of waveforms to CSV.

## Command line

```bash
python -m awr1843_vitals                       # dashboard
python -m awr1843_vitals gui --sim             # dashboard with the simulator selected
python -m awr1843_vitals run                   # print estimates in the terminal every second
python -m awr1843_vitals run --record --duration 300
python -m awr1843_vitals analyze recordings/<ts>/raw_data.bin   # re-process a recording
```

Python API:

```python
from awr1843 import RadarDevice
from awr1843.config import load_cfg_file, parse_cfg
from awr1843_vitals import VitalSignsProcessor

cfg = load_cfg_file("awr1843_vitals/configs/vitals_20fps.cfg")
with RadarDevice.connect() as radar:
    radar.configure(cfg)
    proc = VitalSignsProcessor(parse_cfg(cfg))
    for i, frame in enumerate(radar.frames(timeout=3)):
        proc.add_frame(frame)
        if i % 20 == 0:                      # once per second at 20 fps
            r = proc.update()
            print(r.status, r.breath_bpm, r.heart_bpm, r.distance_m)
```

## How it works

The out-of-box demo has no vital-signs mode, but it can send the **range-azimuth static
heatmap**. This is the complex range-FFT value of every range bin on each of the 8 azimuth
virtual antennas, after averaging over each frame's chirps. Its **phase** is what matters:

```
chest moves d  ->  round trip changes 2d  ->  phase changes 4*pi*d / lambda
```

With λ = 3.9 mm, a 0.1 mm heartbeat movement is an 18° phase change, which is easy to
measure. Each update (4 per second) runs these steps:

1. **Find the person.** Beamform every frame into 32 angle bins. In the measurement zone, pick
   the (range, angle) cell whose complex value varies most over the last 5 s. A breathing
   chest moves, while walls and furniture don't. The lock only moves to a new cell when that
   cell wins clearly 3 times in a row.
2. **Extract the chest signal.** Beamform toward the person and fit a circle to the complex
   samples to remove the static reflection in the same cell. Then unwrap the phase to get
   displacement in mm.
3. **Detect motion.** A displacement change of more than 8 mm within 0.5 s, or repeated
   near-π phase jumps, counts as motion (fast movement aliases the phase). Estimates use only
   the still segment since the last movement.
4. **Breathing.** Band-pass 0.1–0.6 Hz (6–36 /min), then take the FFT peak of the last 25 s
   for the rate. Peak-to-trough gives depth and peak detection gives breath intervals. The
   local 5 s amplitude falling below ¼ of the person's typical depth counts as a pause, and
   10 s or longer is apnea.
5. **Heartbeat.** Band-pass 0.8–2.5 Hz (48–150 bpm) on the chest **velocity**, which favours
   the heart over the much larger breathing motion. Notch out breathing harmonics, then score
   the peaks of a 12 s FFT with a harmonic sum so the fundamental beats its 2nd harmonic.
   Finally a median tracker corrects octave errors and accepts a big jump only once it has
   lasted a few seconds.

### Configurations

| File | Frames/s | Range bins | Notes |
|---|---|---|---|
| `vitals_20fps.cfg` (**recommended**) | 20 | 64 × 6.1 cm, up to 3.5 m | 2 TX × 4 RX. Uses about 60 % of the UART. Handles chest speeds up to about 15 mm/s, which covers deep breathing. |
| `vitals_safe_8fps.cfg` | 8 | 256 × 4.4 cm | TI's unchanged SDK chirp. Use it if your firmware rejects the 20 fps profile. Fast or deep breathing aliases at 8 fps and is reported as motion. |

Both configs keep **`clutterRemoval` off**, because static clutter removal would delete the
zero-Doppler signal that the measurement relies on. If you write your own config, it also
needs `guiMonitor` with rangeAzimuthHeatMap = 1.

## Accuracy and limitations

On the simulator, over 300 s with motion and breath-hold events, the 20 fps profile gave a
breathing rate within **1.0 /min** (mean absolute error) and a heart rate within
**2.2 bpm**, with 97 % of heart readings within ±5 bpm. The 8 fps profile with calm breathing
gave 1.0 /min and 2.6 bpm. The simulator is idealised. On a real person, expect:

- **Breathing rate is robust.** Breathing is a millimetre-scale movement with high SNR.
- **Heart rate is harder.** The heartbeat moves the chest by only about 0.1–0.5 mm, much less
  than breathing does. It is most reliable when the person sits still, close (under 1 m) and
  straight on, and breathes calmly. Check the confidence bar. Readings below 25 % are shown
  dimmed.
- When the heart rate is close to a multiple of the breathing rate, harmonic suppression can
  bias the heart estimate by a few bpm.
- Rates are averages over a 12–25 s window, so sudden changes appear with a delay of a few
  seconds.
- One person at a time.
- **HRV and blood pressure are not available.** The 20 Hz frame rate and the phase noise are
  too coarse for beat-to-beat timing.
- **The 20 fps profile hasn't been tried on a real board yet.** It's built from standard
  commands of the SDK 3.x demo. If a command fails, the *Radar log* tab shows which one. Use
  `vitals_safe_8fps.cfg` meanwhile and report the error.

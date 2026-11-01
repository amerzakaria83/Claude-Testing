# AWR1843 Car Radar Suite

Three automotive radar applications for the **AWR1843BOOST** and a laptop. They use the same
out-of-box demo firmware as the other apps, so you don't need to reflash.

| Mode | Where the radar goes | What it does |
|---|---|---|
| 🚗 **Drive Assist** | Behind the windscreen or on the front bumper, looking forward | Radar speedometer, tracking of vehicles and pedestrians, forward collision warning, time gap to the car ahead, warnings for cut-ins, hard braking ahead, crossing pedestrians and oncoming cars in your lane |
| 🅿️ **Park Assist** | On a bumper, looking outward | Left, centre and right distance sectors with a parking-sensor beeper |
| 👶 **Cabin Guard** | At the rear-view mirror or roof console, looking into the cabin | Occupancy of each seat from breathing, a **child-left-behind alarm**, and the driver's breathing and heart rate |

> ⚠️ **This is an experiment, not a safety system.** It is not certified, is not connected to
> the car, and can miss things or raise false alarms. Never rely on it while driving. Never
> leave a child or a pet in a car because a radar is watching. Mount the board securely,
> keep cables away from pedals and airbags, and don't operate the laptop while driving: have a
> passenger do it.

## Start

- **Windows:** double-click `start_car_windows.bat`.
- **Linux / macOS:** run `./start_car.sh`.
- **Any system:** run `python -m awr1843_auto` (add `--sim` to start with the simulator selected).

1. Pick a mode at the top: **Drive Assist**, **Park Assist** or **Cabin Guard**. Each mode
   loads its own radar configuration.
2. Pick **AWR1843BOOST (USB)** and click **Connect**. To try it without the board, pick
   **Simulator (virtual car)**, which provides a scripted scene for every mode.
3. Click **Start**. If you switch modes while the radar is running, it reconfigures itself.

**Terminal mode (no window):**

```
python -m awr1843_auto run --mode drive --sim --duration 30
```

## 🚗 Drive Assist

### What you see

- **Driver view (3D):** a chase-camera view of the road with the tracked objects drawn as 3D
  boxes. The lane markings scroll at your **measured** speed. The car ahead is orange (red
  during a warning), cars going your way are blue, oncoming traffic is magenta, crossing
  pedestrians are cyan, and stationary objects are grey. Your lane is tinted green or amber by
  time gap, and red when a collision is likely. Switch to **Radar map** for the raw top view
  with every detection, track IDs and velocity vectors.
- **Speedometer:** your speed in km/h and the trip distance, measured by the radar itself.
- **Vehicle ahead:** distance, relative speed, its speed, the time gap (a bar showing the
  2-second rule) and the time to collision.
- **Alert banner, sounds and event log**, from least to most urgent:
  - *Closing* (caution)
  - *Collision warning* (two-tone)
  - *BRAKE!* (alarm)
  - *Vehicle ahead is braking hard*
  - *Vehicle cutting in from the left/right*
  - *Pedestrian crossing ahead*
  - *Oncoming vehicle in your lane!*
  - *Too close* (tailgating)
- **Advice:** a suggested speed for holding your target gap, like adaptive cruise control.

**Settings:** lane width, target time gap (1.0–3.0 s), and warning timing (early / normal /
late).

### How it works (`drive.py`)

1. **Radar odometry: speed without GPS or the CAN bus.** Everything stationary (posts, guard
   rails, signs, parked cars) appears to approach the radar at
   `-speed × cos(angle)`. The app tests every candidate speed and keeps the one that explains
   the most detections. The radar's velocity readings wrap around above ±15.7 m/s (56 km/h),
   but the wrap differs with each point's angle, so the search still finds the right speed. On
   the simulator the error is about 0.06 km/h, including at 60 km/h and above.
2. Each detection is labelled **stationary or moving** by comparing it with that speed.
3. **Clustering.** Moving detections are grouped into objects using position and Doppler. The
   sideways tolerance grows with range because the angle is less precise far away.
   Stationary-looking detections *on the road* are kept too: a person walking straight across
   your path has almost no Doppler.
4. **Tracking.** Each object gets a Kalman filter of its position and velocity relative to the
   radar. Detections are matched to tracks with the Hungarian algorithm. Wrapped Doppler is
   unfolded against each track's predicted speed. Duplicate tracks are merged, and a vehicle
   that comes to a stop stays tracked.
5. **Situation assessment:**
   - **Lane:** each track is assigned a lane by its sideways offset.
   - **Lead vehicle:** the nearest car ahead in your lane.
   - **Collision warning:** comes from the time gap, the time to collision and the braking
     needed to avoid impact. Its levels are caution < 4 s, warning < 2.6 s and critical < 1.6 s.
     These scale with the warning-timing setting.
   - **Lead braking:** the car ahead decelerating faster than 3 m/s².
   - **Cut-ins, crossers and oncoming cars in your lane:** detected from each track's sideways
     motion and direction.
   - **Debouncing:** every alert must persist for 2–3 frames before it shows.

### Radar configuration (`configs/drive_long_range.cfg`)

| Setting | Value |
|---|---|
| Range | 0.38 m bins, up to ~85 m |
| Velocity | ±15.7 m/s unambiguous, unfolded in software; 0.49 m/s resolution |
| Frame rate | 20 frames/s |

- **ADC timing:** identical to TI's SDK profiles (256 samples at 5209 ksps), so every monitor
  setting is a known-good SDK value. Only the chirp slope changes, to 8 MHz/µs, for long range.
- **One TX antenna:** with TDM-MIMO, a target faster than the maximum unambiguous velocity
  would also get a wrong angle. With one TX antenna only its speed wraps, and the app unfolds
  that.

**Mounting:** put the board upright behind the windscreen or on the bumper, antennas forward,
level with the road. Glass and plastic bumpers are mostly transparent at 77 GHz. Metallic
windscreen coatings and tinted films are not.

### Limits

- **Angle accuracy.** With 4 RX antennas the angle accuracy is a few degrees, about 1.5 m
  sideways at 50 m. Lane assignment beyond about 60 m is approximate, and adjacent-lane cars at
  the same far range can merge into one object.
- **No elevation.** Bridges, overhead signs and manhole covers can't be told apart from
  stopped cars. That's why stationary in-lane objects only alert once they persist.
- **Straight roads only.** No yaw rate is measured, so in curves the "your lane" corridor is
  straight.
- **Range.** About 85 m covers city and rural speeds. It isn't enough warning time at motorway
  speeds.

## 🅿️ Park Assist

- **Display:** three sectors (left, centre, right). Each shows the nearest persistent obstacle,
  using bands at 0.3 / 0.6 / 1.2 / 2.5 / 4 m drawn like a car's parking display.
- **Beeper:** speeds up from 1.5 m and becomes continuous below 30 cm.
- **Approach speed:** shown in cm/s.
- **Settings:** car width (sets how wide the sectors are) and front or rear mounting.
- **Configuration:** `park_short_range.cfg` (3 TX, 4.4 cm bins, 10 m, 10 frames/s).
- **Mounting:** about 50 cm above the ground on the bumper, antennas outward. Very low obstacles
  (kerbs) close to the bumper fall outside the radar's vertical beam, as with ultrasonic
  sensors.

## 👶 Cabin Guard

- **Seat map:** five seat zones (range × angle from the mirror position). Each zone is analysed
  with the vital-signs processing of the Vital Signs Monitor. A seat is **occupied** when it
  shows a breathing or moving person. Occupied seats glow green and pulse at the measured
  breathing rate.
- **Leakage rejection:** a strong breather can leak into a neighbouring empty seat. That seat
  is ignored when its chest signal is a copy (correlation above 0.8) of a much stronger seat's
  signal.
- **Child-presence detection** (as required by Euro NCAP from 2025). It arms when you press
  **Car locked**, or automatically when the driver's seat has been empty for 15 s (adjustable).
  Once armed, an occupied seat escalates:
  1. *checking*
  2. *occupant detected* (10 s, chime)
  3. *ALARM* (30 s, siren, with the seat flashing red)
- **Driver card:** breathing and heart rate. An experimental alertness hint appears when
  breathing slows well below the driver's usual rate.
- **Settings:** left- or right-hand drive (mirrors the seat map), auto-arm on/off and delay.
- **Configuration:** `cabin_occupancy.cfg`, the same profile as the Vital Signs Monitor
  (15 frames/s, range-azimuth heatmap).
- **Mounting:** at the rear-view mirror or overhead console, tilted down towards the seats.
  Antennas face the cabin and the board's long edge is horizontal.

### Limits

- **Seat geometry:** the zones are defaults for a mid-size sedan. Large cars or unusual
  mounting may need the zone ranges and angles edited (`cabin.py`, `default_seats`).
- **Sleeping infants:** they breathe shallowly. Detection is tested on the simulator, not on
  real children.
- **No temperature reading:** the radar's temperature sensors measure the chip, not the cabin.

## Recording and replay

**Record** saves `raw_data.bin`, `config.cfg` and a mode-specific log:

- `drive_log.csv`: speed, lead distance, gap, time to collision, warning level and alerts,
  every frame.
- `park_log.csv`: sector distances and zone.
- `cabin_log.csv`: occupancy, arming and alarm level.

**Replay…** plays a recording back and picks the right mode from its configuration.

## The simulator

The simulator chooses its scene from the configuration it receives:

- **Drive (130 s loop):**
  - You set off behind a stopped car.
  - Cruise at about 50 km/h.
  - A car cuts in from the right (~38 s).
  - The car ahead brakes to a stop while the simulated driver is distracted (70 s).
  - An oncoming overtaker swings into your lane (~88 s).
  - A pedestrian crosses (~105 s).

  Throughout, there is oncoming traffic, roadside posts and trees, with radar noise and the
  real velocity wrapping. The status bar shows the true speed and the current event.
- **Park (35 s loop):** reversing towards a wall, with a pole and the corner of a parked car.
- **Cabin (180 s loop):** a driver, a front passenger and a child behind the driver. The
  passenger leaves at 40 s and the driver at 60 s; the child stays. Auto-arm and the alarm
  then follow.

The tests (`tests/test_auto.py`) check every scripted event against the warnings it should
raise, with no stray alerts.

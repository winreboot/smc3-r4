# Diagnostics

Diagnostics is in Settings. It's for when something isn't right and you don't know why. It works with any firmware
from 1.3 on; firmware 1.7 adds the board's own counters, 1.8 the motion sensor and 1.9 the seat set-up.

## Quick check

Nothing moves, so it's safe while SimHub runs. It takes about 5 seconds.

- Reads every setting, plus the firmware version, build options and power state.
- With 1.7, also reads:
  - why the board last started;
  - SimHub's USB counters;
  - pot glitches counted at 4 kHz;
  - why each motor is off;
  - the longest main-loop pause;
  - WiFi signal strength;
  - **the EEPROM**, so settings that are running but not saved show up.
- With 1.8, also reads the motion sensor: found or not, baud rate, angles per second, bad packets, tilt limit and
  how often it tripped, level offsets, and what the LED matrix shows.
- Listens to the motors for 3 seconds: pot noise at rest, buzz (PID jitter), positions, and with the sensor the
  seat's angle and how much it wobbles while the motors hold still. Page 2.9.1 adds the same numbers as the Sensor
  check (below) for those 3 s.
- Adds what the page recorded: the last hour of per-minute statistics (how often each motor ran at its power
  limit, how far it lagged behind SimHub), the problems it noticed, and the event log.

## Full check

Stop SimHub's motion output and keep the seat empty. Each motor in turn, with the others switched off meanwhile:

1. **Direction and drive needed.** Gentle settings, nudges of 25 counts, more drive step by step. It records the
   drive at which the motor starts moving up and down. If the motor moves the **wrong way**, or **another motor's
   pot moves instead** (swapped wiring), it stops at once and leaves that motor off.
2. **Step with its own settings.** A 50-count step and back, measuring rise time, overshoot, settling, the error
   left over and top speed. If the motor swings out of control, the check calms it with gentle settings.
   With the motion sensor, the same steps also show what the **seat** did: how many degrees it moved (degrees per
   count, and the whole range that gives), whether it **bounces** after the motor has stopped (frame, mounts or
   linkage flexing), and whether there is **play** - the motor turning before the seat follows. On a traction-loss
   rig, motor 3's response is how far the seat **turns** (the sensor's yaw); a lift that turns the seat, or a
   traction-loss motor that tilts it, means the wiring or the rig setting doesn't match.
3. **What you saw.** The motor nudges towards higher numbers and you answer: up, down, didn't move, or not sure,
   and where on the seat it is. For the traction-loss motor: whether the rear went left or right (or up or down -
   then it isn't the rear slide); the answer is kept for the 3D view. If a motor doesn't move at all, you're asked whether it moved, hummed, or did
   nothing. That tells a slipped pot from a jammed motor from no power.
4. Everything is put back afterwards: settings, positions and which motors were on.

## Sensor check

Page 2.9.1. Nothing moves; pause SimHub's motion output first. It records 10 s of every motor under one condition you
set up and pick next to the button:

| | Set up | Tells |
|---|---|---|
| **A** | Motor power supply switched off (the Arduino and the sensors stay on) | the sensor's own noise, its supply and cable |
| **B** | Motor power on, Disable Motors pressed (nothing driven) | what the motor supply and its ground add |
| **C** | Motor power on, motors enabled and holding | what the drive and the control loop add |

For each motor:

- **Feedback** (the 100 Hz telemetry, after the firmware's glitch filter): the noise as a median spread in 1 s
  windows with each window's creep taken out (quietest, worst, and the worst 250 ms), lowest to highest, drift,
  steps between readings over 2 / 4 / 8 / 16 counts per second, and **events** - runs of readings off the window's
  line by more than max(4 × the noise, 1 count): an impulse lasts up to 25 ms, a burst up to 500 ms, longer is a
  shift - measured from the samples' own timestamps plus one sample period (page 2.9.2; before, n samples × 10 ms),
  and `sample_ms` says the period it saw.
  A verdict - quiet, impulses, bursts, continuous jitter, jitter that comes and goes, slow drift, shifts - with the
  levels it used (starting values; this rig's own A/B/C checks are the real reference).
- **Raw readings** (from the board, firmware 1.7 and newer): a *summary* of every reading before the glitch filter
  over the same 10 s - lowest, highest, the biggest change between two readings, and the jumps over 16 counts - not
  the readings themselves, so how long a spike lasted isn't known. The board reads the pot once per control cycle:
  4 kHz with Fpid divider 1, 4000 ÷ divider otherwise; `rate_hz` and `divider` say which (page 2.9.2). (The board's
  count stops at 65,535; the report says when it's full.)
- **PID output**: its jitter, split into an *estimated* P term (Kp × error ÷ PowerScale, from the telemetry with the
  board's formula) and the rest - D and I, together with the board's integer rounding and clipping, which it doesn't
  report apart. `p_share` is the estimate's share of the variance; a `note` in the file says so (page 2.9.2).
- **Drive (calc.)**: how much of the time the motor was braked (inside the dead zone the bridge shorts it, PWM 0),
  driven, on PWMrev or switched off, the drive's own jitter and how often it switched - calculated from the telemetry
  and the settings the page last read or sent, the way the firmware's output stage does it; not read back from the
  board and not measured on the bridge.

Each check is saved as a report (`..._sensor_A.json` and so on) and its recording can be downloaded as CSV. With two
or three conditions recorded, the result puts the newest A, B and C side by side and reports three parts for each
motor (page 2.9.2; 2.9.1 named the first cause only): the sensor side on its own (A: its 5 V and GND, the cable, the
connectors), what the motor power adds (B over A: the supply or its ground getting into the sensor) and what holding
adds (C over B: motor current in a shared ground, the motor's vibration, or the control loop dithering). Each added
part is √(new² − previous²), which assumes the sources are independent, next to the plain ratio; a part counts when
the new condition is 1.5 × the previous + 0.2 counts noisier or has 3 more events per 10 s. The advice names every
part that counts, biggest first, and says *compare with care* when the checks were taken at positions more than 40
counts apart or more than 7 days apart. The file holds it under `compare`: `table[].parts`, `table[].cautions`,
`method`.

## Motion sensor check

Page 2.9.3; also the **Check (20 s)** button in Settings, Motion sensor. Nothing moves, so it's safe while SimHub runs.
It records 20 s of the link between the board and the WT901C: the 100 Hz telemetry (whether each sample carries
angles) and the board's diagnostics frame four times a second - with firmware 2.1.1 also its sensor link frame
(`[rdj]`, see [PROTOCOL.md](PROTOCOL.md)).

- **The link:** the share of the time angles arrived, and every gap - when it started, how soon the board switched
  baud rate after the last angles (within a second while it holds the sensor too loosely; after 3 s for a sensor that
  went quiet), which rates it tried and where it found the sensor again.
- **What the sensor sends:** angles, acceleration and gyro per second, packets and bad checksums per second, and with
  firmware 2.1.1 the bytes received at any rate, the times it lost the sensor, baud changes, the packet types and the
  result of the last **Set up the sensor**.
- **At rest:** the noise and drift of the angles, the acceleration and the gyro.

The verdicts tell apart: a board that isn't looking for the sensor (Use the sensor not ticked), nothing arriving at
all (power or wiring - with 2.1.1's byte count), bytes that never make a packet (a baud rate the board doesn't try, or
the RS-232 side wired straight to D0), a link that keeps dropping out, a sensor sending only angles or at the factory
10 per second, and a healthy link. The result shows the 20 s as a strip - green while angles arrive, amber while the
board looks, labelled with the rate it tries - and the recording downloads as CSV (every telemetry sample with the
angles, and the board's link readings). The report is saved as `..._imu.json`; its data is under `imu_check`.

## What the page notices while you drive

A background watcher checks every sample, all the time:

| Problem | How it's spotted |
|---|---|
| Pot glitch | Reading jumps more than 60 counts in 10 ms (telemetry), or more than 16 counts in 1/4 ms (firmware 1.7 counter) |
| Moving the wrong way | Driven towards the target for 300 ms but moving away from it |
| Stall | Pushed at full allowed power for 1 s without moving |
| Hunting | Swinging around a still target, after allowing for pot noise |
| Motor switched itself off | Safety cutoff tripped (firmware 1.7 says where) |
| Board restarted | Board uptime went backwards (firmware 1.7 says why) |
| Data gap | No telemetry for more than 1.5 s |
| Tilt limit | Every motor stopped with the seat at the tilt limit (firmware 1.8) |
| Sensor went quiet | The motion sensor stopped sending while it is switched on |

### Load map

While SimHub drives, the watcher also keeps a **load map** for each motor: the Clip window is split into eight
position bands, and for each band and each direction (towards higher or lower counts) it records how far the motor
runs behind and how often it is at full allowed power - plus the seat's angle there when the sensor sends. Lifting a
heavy load shows as one direction falling behind at full power while the other keeps up, often only in part of the
travel. The finding names the band ("struggles moving towards higher numbers around pot 600-820") and, with the
sensor, the seat angles there. It fills while you drive, so look at a quick check after a session.

Each problem keeps about 4 seconds of data from before and 1.5 seconds after, in `diagnostics/incidents/`.

## The findings

Each finding is a **problem**, **warning**, **note** or **OK**, with a plain explanation and **what to do**. They
cover:

- the connection and WiFi signal;
- board restarts;
- old firmware;
- the wrong bridge mode;
- SimHub not sending, or garbage on the USB port;
- the USB port (page 2.9): who has the board's USB connector (firmware 2.0: the main chip's own port), the board's
  own report on its last WiFi/USB chip restart (when it came back, how WiFi rejoined), and a 1.9.7 board that should
  update (its chip restart left the board off WiFi);
- impossible settings: PWMmax 0, PWMmin at or above PWMmax, Kp 0, no cutoff, Clip outside the cutoff;
- unsaved settings;
- motors that are off;
- pots at the end of their travel, noisy pots or glitchy pots (with the RC filter in the README's Pot noise filter,
  and whether the firmware holds glitches back - 1.9.4);
- buzz and hunting;
- motors at their power limit or lagging;
- full-check results: reversed, swapped, not moving, moving one way only, unstable, PWMmin too low;
- where a motor struggles under load (load map);
- the motion sensor: silent, not switched on, too slow (factory 10 Hz), garbled data, tilt limit off or tripping,
  not levelled, wobbling, sending angles only (1.9 can add gyro and acceleration); the seat bouncing, play in a
  linkage, how many degrees each motor tilts the seat (or, for the rear slide, turns it);
- the rig layout: a lift that turns the seat, a traction-loss motor that tilts it or that you saw lift the seat;
  two front lifts that seem to run opposite ways for higher numbers (from which way each corner sank under the
  seat's weight - drifting while its motor was off, sinking at low drive, or needing clearly less drive that way;
  compared in SimHub's numbers, so REVERSE_MOTOR1 is allowed for); an answer to the full check's question that
  disagrees with how that corner sinks;
- soft start (firmware 1.9.1): off, or a motor still creeping to its position; firmware 1.9 without it;
- firmware updates (1.9.2): the WiFi module's own firmware too old to update from the page (or to show progress), the
  last update failed and why, the board joined WiFi with the details it remembers, firmware 1.9.1 that can't be
  updated from the page yet;
- the seat set-up (firmware 1.9): none yet, running but not saved (Seat setup left open), a motor switched off by
  its end stop, the page holding SimHub's positions back;
- where each motor rests with the motors off (page 2.9.3): past its safety cutoff (the board can't switch it on from
  there) or past its end stop (firmware 2.1.1 brings it back when the motors are switched on; Seat setup now puts the
  end stop past it); a contactless sensor that reaches the end of its output (a TT 6127 puts out about 41-982 counts);
- motors that don't agree on which way is up;
- a motor whose name doesn't match where you saw it move.

## The file

A diagnostics file is one JSON document that describes itself, so whoever reads it doesn't need this project open.
The newest 20 are kept in `diagnostics/reports/` and can be downloaded from the page.

| Section | Contents |
|---|---|
| `about` | What the file is, units, how to read the traces |
| `made`, `kind`, `problem` | When, quick, full, sensor or imu (Motion sensor check, page 2.9.3), and what the owner typed |
| `summary` | The findings as plain text |
| `findings` | The findings, structured |
| `board` | Connection, firmware, build options (2.0: who has the USB port), uptime, last start cause, WiFi, USB counters, per-motor counters, motion sensor and LED matrix (1.8), seat set-up: level trim, travel limits, end stops, tilt limits (1.9), soft start (1.9.1), the WiFi module's firmware, WiFi memory and the last update (1.9.2), the last WiFi/USB chip restart (`module_restart`, 1.9.8) |
| `firmware_update` | The page's own record of the last firmware update: from and to which version, every step, the result |
| `rig` | Motors in use, motor names, layout (front lifts + rear traction loss, or lifts), which motor is the rear slide and which way it goes for higher numbers; where each motor rests with the motors off (`rest_with_motors_off`, page 2.9.3) |
| `imu_check` | Motion sensor check: the link (share with angles, gaps, rates tried and found), what the sensor sends, the strip, at-rest noise and drift, and the board's sensor link frames at the start and end (firmware 2.1.1) |
| `settings` | Running settings, saved settings (EEPROM), the differences, the last save made from the page |
| `live` | The 3-second listen (2.9.1: per motor also `still`, the Sensor check's numbers, and `raw`, the board's raw readings) |
| `capture` | Sensor check: the condition, and per motor `feedback` (noise, events, PID split, drive) and `raw` |
| `compare` | Sensor check: the newest A, B and C side by side, and what they say |
| `check` | Full check: per motor the result, the drive needed, step metrics, what you saw, and a 50 Hz trace |
| `watch` | Statistics for the last hour (including each motor's load map and the seat's range), per minute, and problem counts for 24 hours |
| `incidents` | Recorded problems; the four newest include traces |
| `events` | The last 250 events |
| `telemetry` | The last 10 s at 100 Hz and the last 2 minutes at 5 Hz; a sensor check's 10 s (`capture`) with every motor's target, goal, feedback, PID output, drive and its state |
| `scope`, `profiles`, `backups` | Page version and settings, profile names, recent backups |

Traces are CSV text: a `columns` header and one row per sample. `t`, `g`, `f` and `p` are target, goal, feedback
and PID output; `off` holds three 0/1 flags for motors switched off; `low` is 1 at quarter power; `roll` and
`pitch` (degrees) appear when the motion sensor sends.

The file contains no passwords - not even the WiFi password the board remembers (1.9.2). It does contain your
board's network address and motor names.

[examples/diagnostics-full-check.json](examples/diagnostics-full-check.json) is a full check from the simulated
rig with motor 2 wired backwards, a glitchy pot on motor 3, and the motion sensor finding play in motor 3's
linkage.

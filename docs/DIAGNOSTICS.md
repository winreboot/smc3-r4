# Diagnostics

Diagnostics is in Settings. It's for when something isn't right and you don't know why. It works with any firmware
from 1.3 on; firmware 1.7 adds the board's own counters.

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
- Listens to the motors for 3 seconds: pot noise at rest, buzz (PID jitter), positions.
- Adds what the page recorded: the last hour of per-minute statistics (how often each motor ran at its power
  limit, how far it lagged behind SimHub), the problems it noticed, and the event log.

## Full check

Stop SimHub's motion output and keep the seat empty. Each motor in turn, with the others switched off meanwhile:

1. **Direction and drive needed.** Gentle settings, nudges of 25 counts, more drive step by step. It records the
   drive at which the motor starts moving up and down. If the motor moves the **wrong way**, or **another motor's
   pot moves instead** (swapped wiring), it stops at once and leaves that motor off.
2. **Step with its own settings.** A 50-count step and back, measuring rise time, overshoot, settling, the error
   left over and top speed. If the motor swings out of control, the check calms it with gentle settings.
3. **What you saw.** The motor nudges towards higher numbers and you answer: up, down, didn't move, or not sure,
   and where on the seat it is. If a motor doesn't move at all, you're asked whether it moved, hummed, or did
   nothing. That tells a slipped pot from a jammed motor from no power.
4. Everything is put back afterwards: settings, positions and which motors were on.

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

Each problem keeps about 4 seconds of data from before and 1.5 seconds after, in `diagnostics/incidents/`.

## The findings

Each finding is a **problem**, **warning**, **note** or **OK**, with a plain explanation and **what to do**. They
cover:

- the connection and WiFi signal;
- board restarts;
- old firmware;
- the wrong bridge mode;
- SimHub not sending, or garbage on the USB port;
- impossible settings: PWMmax 0, PWMmin at or above PWMmax, Kp 0, no cutoff, Clip outside the cutoff;
- unsaved settings;
- motors that are off;
- pots at the end of their travel, noisy pots or glitchy pots;
- buzz and hunting;
- motors at their power limit or lagging;
- full-check results: reversed, swapped, not moving, moving one way only, unstable, PWMmin too low;
- motors that don't agree on which way is up;
- a motor whose name doesn't match where you saw it move.

## The file

A diagnostics file is one JSON document that describes itself, so whoever reads it doesn't need this project open.
The newest 20 are kept in `diagnostics/reports/` and can be downloaded from the page.

| Section | Contents |
|---|---|
| `about` | What the file is, units, how to read the traces |
| `made`, `kind`, `problem` | When, quick or full, and what the owner typed |
| `summary` | The findings as plain text |
| `findings` | The findings, structured |
| `board` | Connection, firmware, build options, uptime, last start cause, WiFi, USB counters, per-motor counters |
| `rig` | Motors in use, motor names |
| `settings` | Running settings, saved settings (EEPROM), the differences, the last save made from the page |
| `live` | The 3-second listen |
| `check` | Full check: per motor the result, the drive needed, step metrics, what you saw, and a 50 Hz trace |
| `watch` | Statistics for the last hour, per minute, and problem counts for 24 hours |
| `incidents` | Recorded problems; the four newest include traces |
| `events` | The last 250 events |
| `telemetry` | The last 10 s at 100 Hz and the last 2 minutes at 5 Hz |
| `scope`, `profiles`, `backups` | Page version and settings, profile names, recent backups |

Traces are CSV text: a `columns` header and one row per sample. `t`, `g`, `f` and `p` are target, goal, feedback
and PID output; `off` holds three 0/1 flags for motors switched off; `low` is 1 at quarter power.

The file contains no passwords. It does contain your board's network address and motor names.

[examples/diagnostics-full-check.json](examples/diagnostics-full-check.json) is a full check from the simulated
rig with motor 2 wired backwards and a glitchy pot on motor 3.

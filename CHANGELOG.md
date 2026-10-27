# Changelog

`smc3_scope` (the page) and `SMC3_R4` (the firmware) are versioned separately. The page works with older
firmware too; the notes say which firmware a feature needs.

## Page 1.8.1 (firmware unchanged at 1.7)

- A single glitchy pot reading no longer distorts step measurements, so the full check doesn't report a false
  "721% overshoot", and auto-tune isn't misled either.
- The Test Signal motor menu is wider, so motor names fit, and its number boxes no longer show spinner arrows.
- A browser closing in the middle of a request no longer prints a traceback in the log.

## Page 1.8 (firmware unchanged at 1.7)

- **Motor names.** In Settings, Motor names, call each motor by where it sits: Left, Right, Rear, Front left, and
  so on, or a custom name. The names show on the motor cards, graphs, PID table, menus, recommendations,
  auto-tune and diagnostics, always next to the number.
- In the full check, the page compares the name you gave a motor with where you saw it move, and warns if they
  contradict each other.

## Page 1.7 + firmware 1.7

- **Diagnostics** (Settings):
  - **Quick check:** nothing moves.
  - **Full check:** moves each motor a little to find its direction, the drive it needs to start moving, swapped
    wiring and its step response. It can ask you which way each motor moved.
  - **Results** are in plain words, each with a fix.
  - **One self-describing file** you can give to whoever helps you.
- **Watcher.** The page records problems while you drive, with a few seconds of data around each one:
  - pot glitches;
  - a motor moving away from its target;
  - a motor stalling;
  - hunting;
  - safety cutoffs;
  - board restarts;
  - data gaps.
- **Event log** of connections, settings changes, tests and auto-tune results.
- **Firmware 1.7:** `[rdd]` returns a diagnostics frame with the reset cause, SimHub USB counters, the longest
  main-loop pause, WiFi signal strength, and, per motor, why it is off, cutoff trips, pot glitches counted at
  4 kHz and the pot range seen. It also includes a copy of the EEPROM. `[rd!]` now reports the build options too.
- Fixed a start-up race that could miss the board's firmware version.

## Page 1.6 + firmware 1.6

- **EEPROM backups** on the NAS. One is kept for every save and every bigger change, and you can switch back to
  any of them.
- **Recommended starting points** for circuit racing, rally, flight, space and trucks. They adjust your own tuned
  settings and come with SimHub tips for each kind of sim.
- **Selected motor** is highlighted across the page.
- Live graphs expand and collapse.
- **Motors in use** can be set to 1, 2 or 3; unused motors can be switched off on the board too.
- Favicon.
- **Firmware 1.6:** target smoothing and the max-speed limit are now settings (`[f a b]` sets them, `[rdf]` reads
  them) and are saved to EEPROM.

## Page 1.5 + firmware 1.5

- New dashboard look with sidebar sections, motor cards, quick controls, colour-coded graphs and the PID table.
- Named **profiles**.
- EEPROM section with download and restore.
- Test-signal presets.
- Day / Dark / Auto theme.
- System Status card.
- Works down to phone width.
- **Firmware 1.5:** `[dis]` and `[di1]`–`[di3]` are a software stop for Disable Motors.

## Page 1.4 + firmware 1.4

- **Auto-tune** per motor. It measures friction (PWMmin) and pot noise (dead zone), sweeps Kp to find the sweet
  spot, checks small moves, and adds Ki or Kd only if they help. Gentle, Normal and Firm styles; Revert undoes it.
- Settings dialog: board address, find the board on the network, auto-find.
- Day/night theme.
- Hover help on every setting.
- Kp, Ki and Kd are shown in SMC3Utils units.
- **Firmware 1.4:** position commands over WiFi keep the board at full power, so tests from the page aren't
  stuck at quarter power. `[rd!]` reports the version.

## Page 1.3 + firmware 1.3

- First release.
- **Live scope:** target, goal, feedback and PID output at 100 Hz over WiFi, while SimHub keeps the USB port.
- **Live parameter table:** Apply sends settings straight to the board; Save to EEPROM keeps them.
- Test signal: sine, square, triangle or hold a position.
- Docker image for a NAS.
- **Firmware SMC3_R4** (UNO R4 WiFi):
  - the PID loop runs in a 4 kHz timer interrupt;
  - 14-bit pot readings;
  - 25 kHz PWM at full timer resolution;
  - target smoothing;
  - a working integrator, with derivative taken on the measured position;
  - an LED matrix status display;
  - WiFi telemetry.

  The SMC3 protocol and EEPROM layout are unchanged.

## SMC3_smooth 1.0 (UNO R3)

The original SMC3 v0.70 for the UNO R3, with target smoothing and the PID fixes. It's kept for anyone staying on
an R3.

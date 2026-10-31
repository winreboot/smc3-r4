# Changelog

`smc3_scope` (the page) and `SMC3_R4` (the firmware) are versioned separately. The page works with older
firmware too; the notes say which firmware a feature needs.

## Page 2.8.3, firmware 1.9.6

- **Firmware 1.9.6: no more lost bytes on the USB port.**
  - The RA4M1's USB serial port has no receive FIFO: one byte waits while the next comes in, 20 µs at 500000 baud.
  - The 4 kHz motor (PID) interrupt kept the receive interrupt waiting longer than that, so bytes arriving in a burst
    were lost. Now and then that hit one of SimHub's position commands.
  - It hit the page's commands over the USB cable often enough that the board dropped out of page mode. The page's
    events said "... bytes of unreadable data arrived on the USB port"; auto-tune couldn't read the motor settings.
  - The receive interrupts of the USB port and of the motion sensor's port now come before the PID interrupt. Nothing
    else changed.
- **The page no longer calls a 1.9.5 board "1.9.4 or older".** When the board answers over the cable but doesn't switch
  over to it, the page waits longer, then says the commands arrive damaged and that 1.9.6 fixes it.
- The page repeats "stay on the cable" (`[kU1]`) twice a second, so one damaged copy doesn't end the link.
- When the bridge opens the port it now asks for the firmware number up to 6 times. The first answer could get lost
  while Windows sets the port up.
- **"No answer from the board" and a USB chip older than 0.6.0:** the page now says to update the board's USB chip
  with Arduino's standalone updater. The IDE's Firmware Updater can't do it in that state, because it needs those
  answers itself. That update fixed a board that sent nothing back over USB. Steps:
  [INSTALL, USB cable](docs/INSTALL.md#usb-cable-for-calibration).
- Diagnostics: a finding recommends 1.9.6 for boards on 1.9.2-1.9.5.
- Simulator: `--fw195`, and `--usb-lose-ku` (every `[kU1]` arrives damaged).

Update the firmware first. Either upload `SMC3_R4_v1.9.6.ino` with the Arduino IDE, or use **Sketch, Export Compiled
Binary** and install the `.bin` in Settings, Firmware update. A build with the placeholder WiFi details uses the ones
the board remembers. Then update the page (Settings, Updates, **Update page**) and start the bridge once more.

## Page 2.8.2 (firmware 1.9.5, unchanged)

- **The Windows USB bridge gets the board's data.** On a real PC nothing came back from the board (the bridge showed
  "board -> page 0 kB" and the page gave up with "The board didn't answer on COM4 in 45 s"): the bridge waited on an
  asynchronous read of the COM port that never completed there. It now reads whatever bytes are waiting, about every
  10 ms.
- When the bridge opens the port it asks the board for its firmware number and says what it found: "The board answers
  on COM4 (firmware 1.9.x)" or "No answer from the board on COM4". The page's message says so too.
- 2.8.1's bridge re-sent the page's first bytes over and over (one line lost in a rewrite), so it never noticed the page
  going back to WiFi. Fixed.
- The bridge holds up better:
  - Pulling the USB cable no longer stops it: it waits and links again when the cable is back.
  - A click in its window no longer freezes the link (the old console's QuickEdit "Select" is off while it runs).
  - A board that stops taking bytes can't hang it any more.
  - "COM4 is in use" now shows only when the port really is in use. Otherwise it says what Windows says, e.g. that the
    port doesn't exist.
  - It lets go of the port when the switch goes back to WiFi while it is still trying to reach the page, and after a
    minute without any word from the page.
  - The page may now be quiet for 10 s (was 6 s) before the bridge drops the link.
- After a network outage during a USB link, the switch stays on USB for a minute (it went back to WiFi after 6 s), so
  the bridge links again by itself once the network is back.

Update: Settings, Updates, **Update page**, then start the bridge once more (the same Win+R line).

## Page 2.8.1 (firmware 1.9.5, unchanged)

- **A WiFi / USB switch at the top of the page.** The bridge now keeps running when you go back to WiFi: it lets go of
  the COM port, so SimHub can have it, and waits. Flip the switch to USB (SimHub closed) and it opens the port and links
  again; if the port is still in use the switch says so. With no bridge running, flipping it shows the line to start
  one. `standby` (or `SMC3_STANDBY=1`, `--standby` for the Python bridge) starts the bridge waiting, e.g. with Windows.
- **The firmware version stays right** when you switch between WiFi and the cable: it showed "1.9.2" (a guess from the
  diagnostics frame) until the new connection's own version text arrived, or for good when that answer got lost. Now it
  shows the version the board last reported meanwhile, and the page asks again if the answer doesn't come.

Update: Settings, Updates, **Update page**. Then start the bridge once more (Win+R, the same line): that loads the new
bridge.

## Page 2.8 (firmware 1.9.5)

**The USB cable for calibration.** The UNO R4 WiFi's WiFi module now and then holds the board's loop up for a second
or more, so the page's moves lag and auto-tune has to redo test moves (2.7.2). With SimHub closed, the page can now
talk to the board over its USB cable instead, and the board then makes no WiFi call at all.

- **Settings, USB cable** has one line to paste into **Win+R** on the PC the board is plugged into. It starts a small
  bridge (Windows PowerShell, nothing to install) that finds the board's COM port and links it to the page. The page
  takes over from WiFi by itself - the header says **USB cable** - and goes back to WiFi when you close the bridge
  window or press **Back to WiFi**. Then start SimHub again.
- The bridge connects out to the page's web port, so the page can stay on the NAS and the PC's firewall doesn't ask.
  It waits while auto-tune, the full check, Seat setup or a firmware update runs, links again by itself after a page
  restart, and says why if the page won't take it (WiFi only, Back to WiFi, a board without 1.9.5).
- macOS, Linux (or Windows with Python and pyserial): `smc3_scope_v2.8.py --usb-bridge PORT --page URL` is the same
  bridge.
- The header shows **WiFi** or **USB cable** next to Board Connected; the auto-tune view suggests the cable while on
  WiFi; diagnostics files say which link was used. Firmware updates from the page stay WiFi only.
- Back to WiFi (and WiFi only) wait while auto-tune, the full check or Seat setup runs. If the link changes during
  auto-tune anyway (the bridge window closed, the board restarted), auto-tune stops and puts the motor's previous
  settings back - at once, or as soon as the board is reachable again.

**Firmware 1.9.5:** USB page mode. `[kU1]` over USB (the page repeats it every second; `[kU0]` or 5 s without it
ends it) sends the telemetry, `[rdd]` and `[rdu]` over USB instead of WiFi, takes USB position commands as the page's
(not held back, not counted as SimHub's) and skips every WiFi call. Without `[kU1]` nothing changes for SimHub. See
[docs/PROTOCOL.md](docs/PROTOCOL.md#usb-page-mode-firmware-195). Also: `loop()` reads the clock again after the USB
commands, so a command parsed a millisecond after the loop's start no longer makes the comms timeout wrap (one pass
at LOW POWER).

Simulator: `--usb PATH` (the board's USB port as a pseudo-terminal), `--fw194`.

Update: flash `SMC3_R4_v1.9.5.ino` (Settings, Firmware update over WiFi, or the Arduino IDE), and the page - Settings,
Updates, Update page.

## Page 2.7.3 (firmware 1.9.4, unchanged)

- **Update page just updates**: it no longer refuses with "The motors are being moved by hand (Move motors)" when
  the page holds the seat after Return to center or a dragged knob. Just before the restart it gives the seat back to
  SimHub (every motor stays where it is) and stops a test signal. It still waits for auto-tune, the full check, Seat
  setup and a firmware update, which have the board on temporary settings until they finish. The same for Go back to
  built-in.

Update: in the page - Settings, Updates, Update page. (From 2.7.2 or older, press Resume SimHub first if the page
holds the seat: the page that is running decides.)

## Page 2.7.2 (firmware 1.9.4, unchanged)

Auto-tune and the full check cope with WiFi stalls. The UNO R4 WiFi's WiFi module now and then blocks the board's loop
for a second or more: no readings arrive, and the page's targets wait and then reach the board in a burst. On a real
rig that stopped auto-tune of the rear slide twice in a row ("Couldn't see the move in the telemetry", then
"Something else is moving motor 3 (SimHub?)") and put WiFi delays into some of the measured lags.

- **A spoiled test move is done again** (up to three times): readings missing during it, readings that stop short of
  the present, targets that reached the board bunched up (so the gradual move was really a jump), or the motor setting
  off the wrong way right after such a hiccup. The log says "WiFi hiccup during that move (...) - doing it again". Only
  a link that keeps stalling stops auto-tune, with a message that says so.
- **Its own late targets aren't SimHub**: targets the page sent in the last 15 s count as its own (2.7.1 allowed
  2 s). The same in the full check, and the watcher no longer reports "SimHub driving" right after auto-tune ends.
- **SimHub waits** while auto-tune or the full check moves a motor (firmware 1.9: `[kH]`, as in Seat setup and Move
  motors), so SimHub's motion output can't get in the way.
- Simulator: `--stalls SEC` blocks the simulated board's loop for 0.6-2.5 s about every SEC seconds. Smoke test case
  14 tunes the rear slide through such stalls.

Update: in the page - Settings, Updates, Update page.

## Page 2.7.1 (firmware 1.9.4, unchanged)

- **Help, Wiring the whole rig**: one drawing of the board, the three IBT-2 bridges (RPWM, LPWM, R_EN + L_EN, VCC,
  GND, B+ / B-, M+ / M-), the motors with their pots, the motion sensor through the MAX3232, the 5V / GND strip, the
  motor supply and USB to the PC, with the connections as a table. Also in the README
  ([docs/img/wiring.png](docs/img/wiring.png)). Help also shows the motion sensor's own drawing now.
- **Update balloon**: a newer page or firmware on GitHub shows as an **Update** balloon on the header's Settings
  button (instead of text in the status line); its tooltip says what's new, and Settings then opens on the Updates
  card.
- The header shows the page's version next to the firmware's. LOW POWER is shorter (its tooltip explains it).
- Quick Controls: Return to center, Resume SimHub, Read from board and Save to EEPROM wrap their text onto two
  lines on a narrow card (at 125 % zoom, for example) instead of running over the buttons' edges.

Update: in the page - Settings, Updates, Update page.

## Page 2.7 (firmware 1.9.4, unchanged)

**Updates from GitHub.** Settings has a new **Updates** card:

- The page asks GitHub twice a day (the first time 20 s after it starts; **Check now** asks at once) what the newest
  release has, and says when there's a newer page or firmware - in the card, in the header (**Update available**),
  and once in the event log. The tick box switches the check off.
- **Update page** downloads the new page into the data folder (`updates/`), checks it (GitHub's SHA-256 of the file,
  its version, that Python can compile it) and restarts into it; the tab reloads by itself. The board isn't touched.
  It waits while anything moves the motors or a firmware update runs.
- The file you started (in Docker, the image's) stays as it was and hands over to the update at every start while
  the update is newer: the update survives restarts and re-created containers, and a newer image wins over an older
  update. An update that doesn't get as far as serving the page, or stops three times in a row within 10 minutes of
  starting, is dropped at the next start; `SMC3_NO_UPDATE=1` skips it. **Go back to built-in** undoes it.
- A restart waits until nothing is moving the motors. On Windows the started file stays as the parent of the page,
  so a console window or service wrapper still sees one program.
- **Download firmware** downloads the release's `.ino` - firmware is never installed by itself. The card says how to
  flash it (Arduino IDE over USB, or export the `.bin` for Firmware update).
- The Docker image workflow tags two-part versions too (`v2.7` -> `2.7`; its semver rule needed three parts).
- Smoke test case 13: a stand-in for GitHub's API - check, update, hand-over at restart, going back.

Update: the page only - this once by hand (or a new image); later ones from the page.

## Page 2.6.2 (firmware 1.9.4, unchanged)

Nothing changes in what the page does. New:

- **[docs/INSTALL.md](docs/INSTALL.md): running the page step by step** - Windows (with starting at login), Mac,
  Linux and Raspberry Pi (with a systemd service), Docker, Synology Container Manager, building the image yourself,
  every option, and what to check when something doesn't work.
- **A ready-made Docker image**, `ghcr.io/winreboot/smc3-scope`, for amd64 (PCs, most NAS), arm64 (Raspberry Pi 4/5,
  ARM NAS) and arm/v7 (32-bit Raspberry Pi OS). The `docker-image` workflow builds it for every release tag (`latest`,
  the version and the minor version). [scope/compose.prebuilt.yaml](scope/compose.prebuilt.yaml) runs it without
  building anything.

## Page 2.6.1 (firmware 1.9.4, unchanged)

After the first run of 2.6 on a real rig, which settled for Kp 134 and Ki 70:

- **"Falls behind"** is judged where the goal moves at full speed, and only with the drive at PWMmax. 2.5 compared the
  motor's speed in the last quarter of the move with the goal's average speed; on 2.6's shorter moves that quarter is
  all easing-out, where the goal itself slows down, so most Kp values were flagged and the sweep fell back to the
  highest unflagged one (Kp 149 x 90%), although Kp 251 already met the lag target both ways.
- **Ki** stops once a step no longer brings the leftover error down by a count - it's at the floor the dead zone and
  the pot noise set - and the smallest Ki within a count of the best is kept. 2.6 kept the last Ki it tried: Ki 70,
  for 2 counts less error than Ki 5 and 25 points more overshoot on small steps.
- **Your answer against the weight.** Diagnostics warns, and auto-tune's log says, when your answer to the full
  check's question disagrees with how that corner sinks: you saw it go up for higher numbers, but it goes towards
  higher numbers more easily (it sank that way while switched off, and needs less drive that way).

Update: the page only.

## Page 2.6 and firmware 1.9.4

- **Auto-tune moves out and back from where the seat starts**, the way you move a knob on the dashboard: up by the
  test travel, back to the start, down, back to the start (right, back, left, back for the rear slide).
  - The seat never goes further than the test travel from where it was.
  - Each direction is measured twice, going away from the start and coming back to it.
  - 2.5 went from the bottom of the travel to the top and back, so every test seemed to start by going down.
  - The small kerb-sized steps (±30 counts from the start, as big as before) and Check work the same way.
  - The top speed is timed on two equal moves, down and back up over twice the test travel, after a gentle move up.
  - A short test travel still gets a gradual move: at least 0.3 s.
  - Each test move holds 1.3 s at its end, up to 2 s for a slow motor, so a slow motor still creeping in at the
    low Kp the sweep starts with isn't taken for a restless one.
  - The "moves away from where it is driven" stop needs it to last 150 ms longer, so a cluster of pot glitches
    can't fake it (one simulated run in sixteen with a badly glitching pot on firmware 1.9.3 stopped falsely); a
    motor wired backwards still stops within a fraction of a second.
- **The sweep charts scale to the results.** 2.5's overshoot chart ran up to the limit line, so gradual test moves
  that overshoot 0-2 % drew a flat line at the bottom. The scale now fits the values (the limit line shows when it
  fits, otherwise its value is printed), and tabs switch the second chart between overshoot, off by and
  settle time, with the peak value shown.
- **Which way a lift sinks.** The full check notes which way a lift's numbers run when its corner sinks under the
  seat's weight - drifting while its motor is switched off, or sinking at low drive - and keeps it.
  - Until you answer the full check's question, auto-tune goes by that for up and down (and says so in the log).
  - Diagnostics warns when the two front lifts seem to run opposite ways for higher numbers: one of them would move
    the wrong way in games. Besides sinking, a lift that needs clearly less drive one way counts (the weight helps
    it down). Compared in SimHub's numbers, so a motor 1 mirrored with REVERSE_MOTOR1 is allowed for. The fix names
    the drag test, SimHub's Invert, REVERSE_MOTOR1 for motor 1, or swapping both the motor wires and the pot's outer
    wires.
- **Pot glitches.**
  - **Firmware 1.9.4** holds a pot reading back when it is more than 16 counts from the last good one (no motor
    moves that far in 1/4 ms), for up to 10 ms (`POT_HOLD_MS`: 40 readings at 4 kHz, fewer with SMC3Utils' PID
    divider); a change that lasts longer is real and goes through, so a broken pot still trips the cutoff, at most
    10 ms later. The glitch counter in Diagnostics still counts every raw glitch. `POT_FILTER 0` switches it off;
    build flag bit 5 tells the page whether a build has it. Nothing else changed: protocol, EEPROM and settings as
    in 1.9.3.
  - **Pot noise filter**, in the README (Hardware) and the page's Help: a drawing, the parts and the wiring rules
    for an RC filter at the Arduino - 1 kΩ in the wiper wire, 100 nF and 1 MΩ from the A pin to GND - with shielded
    cable grounded at the Arduino end only.
  - The glitch and noise findings point to it; on 1.9.4 a glitchy pot is a warning (the motor no longer kicks), and
    on older firmware the fix also suggests 1.9.4.
- Diagnostics reports the board's own firmware version text (1.9.3 showed as 1.9.2).
- Simulator: firmware 1.9.4 with its filter (`--no-potfilter` for 1.9.3); `--sag-high M`, a lift whose weight pulls
  it towards higher numbers.
- Smoke test: auto-tune goes up first and stays within the test travel of the start; a glitchy pot on 1.9.4 gives a
  warning that names the filter; the report shows the board's own version.

Update: the page as usual. Firmware 1.9.4 is optional - flash `SMC3_R4_v1.9.4.ino` over USB, or from the page
(Settings, Firmware update) once the WiFi module runs 0.4.0 or newer.

## Page 2.5 (firmware 1.9.3, unchanged)

Auto-tune, reworked after a run on a real rig:

- **Each direction on its own.** Every test moves the motor both ways - up and down for a lift, left and right for
  the rear slide - and the log, the gauges and the result show each direction's figures. (2.4.1 also tested both
  ways but only reported the worse one.) The board keeps one set of values per motor, so the worse direction still
  decides; the minimum drive is measured each way as before, and a big difference between the two gets Ki.
- **Gradual test moves.** Auto-tune first measures the motor's top speed each way with one quick move. Every test
  after that moves the target at 60% of the slower way's top speed, easing in and out: a move the motor can follow,
  like a game's. 2.4.1 jumped the target across the whole travel, so the motor ran flat out at PWMmax for most of
  every test and Kp hardly mattered: on the run that prompted this, every Kp from 193 up rose in 257-276 ms.
  - Kp is now judged on how far the motor trails the move (lag, in ms). The sweep stops once the motor follows
    within the style's target both ways (Gentle 120 ms, Normal 80, Firm 50), or when it starts to overshoot.
  - Small sharp steps (kerb-sized) still check that it stays calm.
- **Who's in the seat while it tests.**
  - **Nobody**, with your weight entered: auto-tune allows for it.
    - The overshoot limit is about a third lower (your weight carries further).
    - The lag target is a quarter lower: a stiffer Kp, because the motor is slower lifting you.
    - Test moves run at 50% of top speed, and a little Ki is kept for the weight.
    - A note says when the empty seat already uses most of PWMmax.

    On the simulator with an 82 kg rider, values tuned with the seat empty and the allowance followed as closely
    going up as values tuned with the rider in the seat (96 ms, against 127 ms without the allowance).
  - **Me**: Start gives you 15 s to get in. Stop, Space or Esc ends it at any time. While you get in, the
    "moves away from where it is driven" watch rests, so your weight pushing a lift can't trip it.
  - Stopping auto-tune or Check holds the motor where it is, taken from the median of its last readings (one
    glitchy reading must not become a target). A refusal ("Cannot start") sends nothing at all.
- **Check** tests the motor's current values (test moves both ways, and small steps), changes nothing, and says what
  to change. Typically you run it with you in the seat after tuning with the seat empty.
- **Auto-tune live** shows each test as gauges:
  - lag (rise for small steps), overshoot, settling, how far short it stops, ringing and drive used;
  - one needle per direction, with coloured bands for what's good.

  The same card shows the phases, the values running, pot noise, the minimum drive and top speed each way, and the
  Kp sweep as two small charts: lag and overshoot for each Kp, with the figures on hover.
- **Copy** puts the whole log and the result on the clipboard. It also works over plain http from the NAS, where
  browsers switch the clipboard API off.
- Small test steps go 30 counts each way. 2.4.1 stepped 15 up and 30 down, so the shorter step up often "never got
  to 90%". "Got there" now allows for the dead zone.
- **A glitchy pot can't spoil a tune.**
  - Before overshoot, settling, ringing or a wrong-way start are judged, each pot reading is compared with the
    median of the 11 around it, and one far off is replaced. Bad readings come in clusters: on the rig that prompted
    this, about 80 a second on one pot. 2.4.1 used a 3-reading median, and one cluster could stop a tune as "moved
    the wrong way".
  - The watch for a motor running away from where it is driven compares cleaned readings too.
  - The Kp sweep stops on overshoot only when it happens twice in a row.
  - A Kp whose test is spoiled right after a calm one is tested once more. A burst of bad readings can kick the
    motor itself, because the PID reacts to it.
- The full check remembers which way each lift went when you answered its question, so auto-tune can name the
  directions up and down.
- Simulator: `--load KG` (a rider: the lifts need more drive going up and less going down, and everything carries
  more weight) and `--vmax`. Smoke test: auto-tune's test moves are gradual, and Check changes nothing.

## Page 2.4.1 (firmware 1.9.3, unchanged)

Fixes for rigs whose pots glitch (the wiper losing contact now and then: single readings 16-40 counts off, sometimes
a few in a row), found in a real diagnostics file:

- **The full check no longer reports "Motors 1 and 2 are mixed up" for a glitchy pot.** Its crosstalk test compared
  two single readings in a row against 12 counts; now it looks at the median of the last 11 readings (crossed wiring
  moves the other motor and it stays moved, a glitch doesn't), and the motor's own checks at the median of 7 - from
  the first reading of each check on.
- **Moving a motor from the page judges it on 5-reading medians.** 2.4 could switch a motor off as "moved AWAY ...
  wired backwards" because of a glitch. And it now tells the cases apart: a motor that runs away fast (more than
  120 counts/s the wrong way: wired backwards) is switched off as before; one that sinks slowly, is stuck or falls
  behind (too weak, blocked, no motor power) is held where it is - switching it off would let a seat that sinks
  sink further.
- The watcher's "kept swinging around its target" needs median-filtered swings around the target itself, so a
  glitchy pot - or a motor stuck away from its target with the motor power off - no longer counts as hunting.
- Auto-tune's travel and run-away checks use a 5-reading median too.
- **The full check calls a motor "WRONG way" only when it goes the wrong way both ways** - like auto-tune since 2.4.
  One that sinks under its load at the lowest drive while it moves the right way the other way is tried with more
  drive and reported as "sinks under its load when the drive is weak" (its gearbox can be driven backwards), not as
  wired backwards.
- While the full check measures how the other motors drift (2.4), it keeps watching the motor it is about to test: one
  that runs away from its target on its own is stopped at once and reported as wired backwards.
- "Moved away from its target while driven" no longer says to swap the motor wires when the full check found the
  motor goes the right way (then it's the load: the seat's weight moved it while the drive was too weak). "Pushed at
  full power without moving" says that every motor at once means the motor power was off.
- Old WiFi module: the page and the diagnostics give the steps - close SimHub (it keeps the USB port busy), Arduino
  IDE, Tools, Firmware Updater, then upload SMC3_R4 again (the updater erases the sketch on the board) - and name
  Arduino's stand-alone updater if the IDE's fails.
- A crossed-wiring stop in the full check records the reference, the reading, the drift and how many readings it
  saw.
- Simulator: `--pot-glitches M:N`, `--sag-pull X`. Smoke test: a full check and moves from the page on glitchy pots.

## Page 2.4 (firmware 1.9.3, unchanged)

- **Move the motors from the page** (firmware 1.9). Drag a motor card's knob and that motor follows: up to
  200 counts/s, easing in and out, never past its Clip window or travel limits (hatched on the track while you
  drag; a thin line marks where it is going). Arrow keys on a selected knob move it 5 counts, with Shift 25.
- **Quick Controls, Return to center**: every motor goes back to its neutral position - SimHub's 0, the level seat
  with the rear centred (Seat setup's level, if you made one) - gently, at up to 120 counts/s.
- While the page moves the motors it **has the seat**: the board ignores SimHub's position commands (`[kH]`, as Seat
  setup does; SimHub keeps sending) and every motor runs at full power. The status bar says so and **Resume
  SimHub** lights up; it gives the seat back. The page also lets go by itself when the board restarts, when the
  page server stops (SimHub has the seat back within 3 s), or when something else moves the motors (auto-tune, a
  test signal, Seat setup, the full check, a firmware update).
- A motor that doesn't follow the page is switched off before the firmware's cutoff would trip: within 0.1 s if it
  moves away from where it is sent (wired backwards), within 0.4 s if it falls more than 120 counts behind (blocked,
  too weak). The page keeps the seat. (2.4.1: a slow one is held where it is instead.)
- **Auto-tune works with a noisy pot, and with a seat that sinks under its weight.** 2.3.1 could stop at once with
  "moved the wrong way - the motor or the pot is wired backwards" although nothing was wrong:
  - The minimum-drive search compared two single pot readings against 6 counts. A pot that reads ±2 counts at
    rest crosses that by noise alone - on the simulator, 5 runs in 6 stopped there. Movement is now judged on the
    median of nine readings against limits that grow with the pot's noise (measured first, every run).
  - A motor is only called wired backwards when it goes the wrong way **both** ways. A seat that sinks while the
    drive is still too weak to lift it goes the wrong way one way only: that is now explained in the log and the
    search carries on with more drive.
  - The step analysis, the "holds still" limits and the ringing count allow for the pot's noise too, so a noisy pot
    no longer reads as a lively motor (2.3.1 stopped the Kp sweep at the first step).
  - It says when the pot is noisy, and what usually fixes it (a 100 nF capacitor from the wiper to GND, pot wires
    away from the motor wires).
  - A safety stop now **switches the motor off with its own settings back** (firmware 1.5+), so it shows Off and
    Enable Motors is all it takes once the cause is fixed. 2.3.1 left it on with PWMmax 0, which looked Online but
    couldn't move.
  - The page re-reads the PID table after any end of auto-tune, not just a finished one.
- The full check and the watcher use the same median-based wrong-way test.
- Full check: a switched-off motor that drifts on its own (a gearbox the seat's weight can drive backwards) no
  longer reads as crossed wiring. Its drift is measured first and allowed for.
- Simulator: `--pot-noise M:SIGMA` (a noisy pot with the odd spike), `--sag M` (a gearbox the seat's weight can
  drive backwards).

## Page 2.3.1 + firmware 1.9.3

- **Firmware 1.9.3 compiles in the Arduino IDE.** 1.9.2 stopped with "sorry, unimplemented: string literal in
  function template signature": the UNO R4 boards package compiles with GCC 7, which can't have the update file's
  name inside a template's signature. It is a parameter now. Nothing else changed.
- Page 2.3.1 names `SMC3_R4_v1.9.3.ino` where it tells you what to flash; the simulator reports 1.9.3.

## Page 2.3 + firmware 1.9.2

- **Firmware update over WiFi** (firmware 1.9.2). Settings, **Firmware update**: choose the `.bin` the Arduino IDE
  exports (Sketch, Export Compiled Binary) and press Install - no USB cable, and SimHub can stay open (with its
  motion output paused).
  - The page checks the file: a program for the UNO R4 WiFi, SMC3_R4 firmware, which version, and whether that
    version can itself be updated from the page and keep the board on the network. `.hex`, `.elf`, other sketches
    and builds that would drop the board off the WiFi are refused.
  - It converts the `.bin` into the UNO R4 WiFi's `.ota` format (LZSS + header, the same as Arduino's own tools) and
    serves it at `http://<page>:<port>/ota/<number>.ota`.
  - The board switches the motors off, and its WiFi module downloads the file, checks it and programs the board
    through its bootloader - the same route as a USB upload. The board restarts, the seat creeps back with the soft
    start, and the page confirms the new version. Progress, every step and the reason for any failure show in the
    card; if anything fails before programming, the old firmware keeps running (the matrix scrolls
    `ERROR E7 UPDATE FAILED`).
  - Refused while SimHub (or SMC3Utils) is sending over USB, during Seat setup, auto-tune, a test signal or the
    full check. Needs the WiFi module's own firmware 0.4.0 or newer (0.5.0+ shows progress).
- **WiFi memory** (firmware 1.9.2): the board remembers the WiFi name and password it last joined with (EEPROM
  112-210). A build that still has the placeholders - or a wrong password - uses the remembered ones.
- Firmware: `[uP=] [uT=] [uI=] [uJ=] [uGO] [uOK]`, `[rdu]` and the "SM3U" frame; `[rdd]` frame version 5 (the WiFi
  module's firmware version, WiFi memory, the last update); motors switched off for an update report "firmware
  update"; compiles with every UNO R4 boards package from 1.0.5 on (the progress needs 1.3.0+).
- Diagnostics: the WiFi module's firmware too old for updates (or for progress), a failed update, a board that
  joined with the remembered WiFi, firmware 1.9.1 that can't be updated from the page yet; the file has the page's
  own record of the last update.
- Page: after a board restart (or any connection that is accepted and closed at once) the page no longer reconnects
  in a tight loop.
- Simulator: firmware 1.9.2 by default, with updates from the page (it fetches, decodes and checks the file like the
  WiFi module and comes back as the version inside); `--fw191`, `--wifi-fw X.Y.Z`, `--ota-fail crc|flash`.

## Page 2.2 + firmware 1.9.1

- **Soft start** (firmware 1.9.1). After the rig has been off the seat may have ridden down a little. At power-up,
  and when you press Enable Motors, each motor now creeps from wherever it is to its position - SimHub's centre at
  power-up - instead of jumping there: 60 counts/s by default, easing in and out. Once it has caught up everything
  runs normally; position commands that arrive meanwhile are followed at that speed until it has.
  - A seat that rode down past an end stop may come back; going further out (or past the SMC3 cutoff, e.g. a broken
    pot) still switches the motor off.
  - The original SMC3's PWMrev drive (full duty back from past the Clip window) stays off while a motor creeps.
  - The tilt limit waits until every motor has caught up; the board keeps full power meanwhile.
  - Settings, Loop, PWM & smoothing: **Soft start** - off, 30, 60, 120 or 250 counts/s, saved on the board at once
    (`[k 82 '=']`, EEPROM 105-107). `[rdd]` frame version 4 reports it and which motors still creep.
- Diagnostics: soft start off, a motor still creeping, or firmware 1.9 without it.
- Simulator: `--start M:POT` (where a pot is at power-up), `--old19` (firmware 1.9 without the soft start).

## Page 2.1 (firmware 1.9, unchanged)

Traction loss: rigs where motors 1 and 2 lift the front corners and motor 3, mounted horizontally, slides the rear
left and right (the seat turning about a pivot at the front) - SimHub's 2DOF + traction loss.

- **Settings, Motor names: what motor 3 does** - slides the rear (traction loss, the default) or lifts the seat like
  the other two - and which way the rear goes for higher numbers.
- **Seat setup**: motors 1 and 2 level the seat; motor 3 is **centred** (the seat pointing straight ahead) and the
  page asks which way the rear went for +10. Motor 3 stops at the **traction loss you allow** - how far the seat may
  turn each way, measured with the sensor's yaw - and the page measures how far it turns per count. Tilt reach, the
  narrowing of combined moves and the tilt limits come from motors 1 and 2 only. The limits step shows how far the
  seat turns each way inside motor 3's travel limits, and can centre it between its ends.
- **3D view**: the rig is rebuilt for traction loss: two front lifts on a yoke with the pivot in its middle, a
  horizontal rear actuator sliding a carriage on a rail, and the seat swinging about the front pivot. The rear's
  position comes straight from motor 3's pot, so it moves exactly with the rig; an arc on the floor shows its travel
  limits, the HUD and the in-world screen show the rear's turn, and a new **Rig** camera looks at the mechanics.
- **Diagnostics**: for motor 3 the full check asks whether the rear went left or right (and remembers it); the seat
  response is measured as the seat's turn; a lift that turns the seat, or a traction-loss motor that tilts it, is
  flagged (wiring or the setting doesn't match the rig). Load map and drive-asymmetry texts no longer talk about
  lifting weight for the rear slide.
- **Auto-tune** judges motor 3 by how the seat turns (with the sensor).
- **Recommendations** add a traction-loss tip per kind of sim.
- **Simulator**: `--layout tl` (the default: front lifts + rear slide, yaw and yaw rate from the sensor) or
  `--layout lifts`; `--tl-left`.

## Page 2.0 + firmware 1.9

- **Seat setup** (new section, firmware 1.9): level first, then the limits.
  - **Level the platform**: by itself with the motion sensor (each motor moves a little to learn how it tilts the
    seat, then the page levels it at the position nearest the middle of every motor), or by hand with a spirit
    level. That becomes **SimHub's centre**: the board adds a level trim to every position command, and the seat
    powers up level.
  - **Find each motor's safe travel**, both ways: it moves slowly and stops at the tilt you allow, when it pushes
    against something, at the end of its Clip window, or when you press Stop (space bar).
  - **Save the limits** on the board: **travel limits** (no position command goes past them), **end stops** (the
    motor switches off past one: E1-E3) and **tilt limits for each direction** (every motor stops: E4). A picture
    shows where the seat can tilt inside them; combined moves can be kept within what you allow.
  - The board ignores SimHub's positions while Seat setup moves the seat, so SimHub can stay running. Cancel puts
    everything back.
- **3D view** (new section): a sci-fi cockpit seat moving live with the motion sensor (or estimated from the motor
  positions), actuators glowing with their drive, a glowing outline where the motors are being sent, the sensor's
  angles, gyro and acceleration, and an attitude indicator with the tilt limits. Cameras, orbit and full screen.
  three.js is built into the page server, so it works without internet access.
- **Motion sensor**: tilt limits for each direction (firmware 1.9); **Set up the sensor** now asks for gyro and
  acceleration too, at 100 Hz.
- Auto-tune, the test signal and the full check stay inside the travel limits, centred on the level seat.
- Diagnostics: the seat set-up is in the file, with findings for a missing or unsaved set-up, an end stop that
  tripped, and a sensor sending angles only.
- **Firmware 1.9**: `[w..]` / `[k..]` seat set-up (EEPROM 66-104, also saved by `[sav]`), `[kH n]` holds SimHub's
  position commands back, `[xe1]` sends the sensor's acceleration and gyro ("SM3X"), `[rdd]` frame version 3. SimHub
  and SMC3Utils see no difference, apart from the level trim and the limits doing their job.
- **Simulator**: `--seat-tilt`, `--stops`, `--seat-setup`, acceleration and gyro from the seat's movement.

## Page 1.9.1 + firmware 1.8.1

The first published 1.9 / 1.8. (Firmware 1.8 stopped in the Arduino IDE with "'GlyphDef' does not name a
type"; 1.8.1 fixes that, and the automatic test now compiles the firmware with Arduino's own tools on every push.)


- **LED matrix redone** (firmware 1.8). The status dots are gone. After the IP address the matrix shows live
  **motor bars**, with a dot blinking where a lagging motor should be. With the motion sensor it can show a
  **seat horizon** instead, or stay **quiet**. While SimHub is idle it runs a slow **oscilloscope trace**.
- **Problems scroll in words**: `ERROR E3 MOTOR 3 CUTOFF`, `ERROR E9 NO WIFI` and so on, again and again while they
  last. A motor switched off on purpose is no longer shown as an error. Help lists every code with what to do.
- **WT901C motion sensor**, set up in Settings, Motion sensor, with no recompiling:
  - switch it on, set the mounting (swap, invert), **Level now**;
  - a **tilt limit** that stops every motor if the seat tilts past it (`ERROR E4 TILT LIMIT`);
  - **Set sensor to 100 Hz** (the factory setting is 10 Hz). The board finds the sensor's baud rate by itself.
- **Seat angle chart** in Live Graphs: roll and pitch in degrees, the range reached, the fastest movement, and a
  small horizon.
- **Diagnostics with the sensor:**
  - the full check measures the seat's own response to each step: degrees per count, **bounce** (flex) and
    **play** in the linkage;
  - findings for a silent, slow, garbled or unlevelled sensor;
  - tilt-limit trips and a lost sensor are recorded as problems.
- **Load map**: while SimHub drives, the page records where each motor falls behind or runs at full power, each
  way. It tells you, for example, that motor 1 struggles lifting above pot 600, which is what a heavy driver does.
  It works from the pots alone, and adds the seat's angle with the sensor.
- **Auto-tune watches the seat** when the sensor sends: the seat's own overshoot and ringing count, so it won't pick
  a Kp that keeps the motor calm but makes the frame bounce.
- **Simulator:** a seat with a sensor (`--imu`), plus `--flex`, `--slop`, `--heavy` and `--imu-lost-at`.

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

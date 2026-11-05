# SMC3 R4

<img src="docs/img/icon-180.png" width="110" align="right" alt="">

**A web dashboard, tuner, auto-tuner and diagnostics tool for DIY SMC3 motion seats, plus SMC3-compatible
firmware for the Arduino UNO R4 WiFi.**

The page lets you:

- watch every motor live;
- tune while you drive;
- let each motor tune itself;
- keep backups and per-game profiles;
- see the seat's real angles with an optional motion sensor, and the seat itself moving in a **3D view**;
- **level the platform** and make that SimHub's centre, then let each motor find how far it may go and save
  **travel limits, end stops and tilt limits** on the board;
- **move each motor by dragging its slider**, or send the whole seat back to center with one button;
- find out why when something goes wrong, and where the rig struggles under load;
- **update the firmware over WiFi**: pick the `.bin` from the Arduino IDE and press Install;
- **calibrate over the USB cable** instead of WiFi, with SimHub closed: flip the WiFi / USB switch at the top of the
  page (Chrome, Edge, Brave, Firefox 151+; or a bridge started with one line in Win+R), and the page talks to the
  board without WiFi's hiccups;
- **run test signals on several motors at once** (twist, heave, corner, wave) and test every motor moving together -
  each one fades in and out, no kick at the start;
- see **the drive the board's output stage sends** (calculated: 0 while braked) next to the PID output, and run a
  **sensor check** that tells the sensor's own noise, the motor supply's and the drive's apart;
- let the LED matrix **play a little game while the rig is idle** (ping pong, brick breaker, missile defense, snake,
  meteor dodge);
- **restart the board** from the page - never while SimHub is driving;
- **park the seat** with the **(P)** in Quick Controls: it goes down evenly to a park position you set - past the
  travel limits if it has to - and the motors switch off there, so no corner sinks on its own; it comes back up level
  when you switch the motors on ([Park](#park));
- **watch motion presets in the 3D view** - wave, twist, heave, corner, an airliner landing and a lap of a circuit -
  while the real seat moves ([Motion presets](#motion-presets));
- **centre the pots**: with the seat level, the page tells you which way to move each pot's belt, and how many
  teeth, so every motor has the same travel up and down ([Centre the pots](#centre-the-pots));
- **check the motion sensor's link** second by second, and **start Seat setup over** in one click.

It knows two kinds of rig: **two front lifts plus a traction-loss motor** (motors 1 and 2 lift the front corners,
motor 3 is mounted horizontally and swings the rear left and right about a pivot at the front - the default), and
**three lifts**. Pick yours under Settings, Motor names.

SimHub keeps driving the rig over USB exactly as before. The page talks to the board over WiFi, so both run at
the same time. You can open it from any browser, including your phone. For calibration with SimHub closed, the page
can use the USB cable instead (page 2.8, firmware 1.9.5; 2.0 recommended - it gives the board its own USB port): see
[USB cable for calibration](docs/INSTALL.md#usb-cable-for-calibration).

![Dashboard: three motors moving under SimHub, motor 2 selected](docs/img/dashboard.png)

---

## What's in here

| Folder | What it is |
|---|---|
| [`firmware/SMC3_R4`](firmware/SMC3_R4) | Firmware for the **Arduino UNO R4 WiFi**. It speaks the SMC3 v0.70 protocol, so SimHub, SimTools, FlyPT Mover and SMC3Utils work unchanged. It adds a 4 kHz control loop, 14-bit pot readings, quiet 25 kHz PWM, target smoothing, a fixed PID, WiFi telemetry at 100 samples a second, an LED matrix that shows live motor bars and problems in words, a level trim with travel limits and end stops for each motor, a soft start, firmware updates over WiFi from the page, its own USB port (2.0: no WiFi chip in between), games on the LED matrix while idle (2.1), park mode (2.1.1), and an optional WT901C motion sensor with tilt limits for each direction. |
| [`scope/`](scope) | `smc3_scope.py`: the page server. It's one file using only the Python standard library, with a Dockerfile and compose files for a NAS or any PC. A ready-made Docker image is on `ghcr.io/winreboot/smc3-scope`; step by step for Windows, Mac, Linux, Raspberry Pi, Docker and Synology: [docs/INSTALL.md](docs/INSTALL.md). |
| [`tools/simboard.py`](tools/simboard.py) | A simulated rig. You can try everything without hardware, and inject faults (reversed motor, swapped wiring, glitchy pot and more). |
| [`firmware/SMC3_smooth`](firmware/SMC3_smooth) | The original SMC3 for the **UNO R3**, with target smoothing and the PID fixes, for anyone staying on an R3. |
| [`docs/`](docs) | [Install step by step](docs/INSTALL.md), screenshots, the [protocol](docs/PROTOCOL.md), the [diagnostics](docs/DIAGNOSTICS.md) file format, an [example diagnostics file](docs/examples/diagnostics-full-check.json) and the [lessons learned](docs/LESSONS-LEARNED.md) on the way. |

> ⚠️ **A motion rig can hurt you.** It moves a person with motors strong enough to lift them. Test with the seat
> empty, keep hands and feet clear, and keep a hand near the motor power switch whenever you test, tune or flash
> new firmware. The safety cutoff in the firmware is a last resort, not a guarantee.

---

## A tour

### Dashboard

Each motor has a card showing where it is being sent and where it really is (the green tick on the bar), its
error and its PID output. The live graphs show, per motor:

- **Target**: what SimHub asks for.
- **Goal**: the target after the firmware's smoothing.
- **Feedback**: where the pot says the motor is.
- **PID output**: the drive sent to the motor.

Good tuning means the green line sits on the blue one. Click a motor, or choose one in a menu, and it is
highlighted everywhere: card, graph, PID column, test signal and auto-tune.

**Move a motor by hand (firmware 1.9): drag its knob.** The motor follows at up to 200 counts/s, easing in and out,
and never goes past its Clip window or travel limits (hatched on the track). A thin line marks where it is going.
Arrow keys on a selected knob move it 5 counts, with Shift 25. **Quick Controls, Return to center** sends every
motor back to its neutral position (SimHub's 0: the level seat with the rear centred), gently.

While the page moves the motors, it **has the seat**: the board ignores SimHub's position commands, and the status
bar says so. **Resume SimHub** gives the seat back. The page also lets go when the board restarts, when the page
server stops, or when auto-tune, a test signal, Seat setup or the full check starts. A motor that runs away from
where it is sent (wired backwards) is switched off at once; one that sinks, is stuck or falls behind is held where it
is.

![Return to center: each motor heading for its neutral position, the page holding the seat](docs/img/manual-control.png)

**Day, Dark and Auto** (which follows your computer):

![Day theme](docs/img/dashboard-day.png)

**Every control explains itself.** Hover over any setting, button or status item to see what it does on the rig,
what too low and too high feel like, and where to start.

<img src="docs/img/tooltip.png" width="720" alt="Tooltip on Kd">

### Motors and live graphs

The expand button opens the graphs full width; pressing it again returns to the page you came from.

![Motors view](docs/img/motors.png)

![Live graphs, 10 second window](docs/img/live-graphs.png)

**Drive (calc.)** (page 2.9.1, named so since 2.9.2): next to the orange PID output, a thin dotted line shows the
PWM the board's output stage sends by its own rule - PWMmin added, PWMmax as the limit, and **0 while it is braked**
inside the dead zone, where the bridge shorts the motor whatever the PID says. The orange line can jitter with
sensor noise while the motor gets nothing; the dotted one shows when it doesn't. It is calculated by the page from
the telemetry and the settings, not read back from the board: it can't tell whether the motor supply is on. The
motor cards and the readouts next to the graphs show it too (*braked*, *+45*, *off*). Click a legend entry to hide
or show its line.

![Live graphs: a sine fading in on motor 1, the dotted Drive (calc.) line next to the PID output](docs/img/drive-line_v2.9.2.png)

### Seat angle (motion sensor)

With a WitMotion **WT901C** on the seat, wired to the board (see [Hardware](#hardware)), Live Graphs gets a **Seat
angle** chart: roll and pitch in real degrees, the range the seat has reached, its fastest movement, and a small
horizon. A smooth slope while the seat moves is good; steps, wobbles, or a slope that slows down in one direction
show load, play or flex.

![Seat angle chart](docs/img/seat-chart.png)

### 3D view

The seat in a sci-fi cockpit, moving live. With the motion sensor it shows the seat's real attitude; without one,
an estimate from the motor positions. On a traction-loss rig the two front actuators lift the front corners through a
yoke, and the horizontal rear actuator slides the rear left and right, so the seat swings about the pivot at the
front; how far it turns comes straight from motor 3's pot, so it moves exactly with the rig, and an arc on the floor
shows the rear's travel limits. (On a three-lift rig the actuators sit where Seat setup found each motor acts.) The
actuators glow brighter the harder they drive; a glowing outline shows where the motors are being sent whenever the seat lags behind.
Around it: the angles and the rear's turn, the sensor's gyro and acceleration, the actuators' positions, the sensor's
state and an attitude indicator with the tilt limits. Drag to turn it, scroll to zoom, or pick a camera (**Rig**
looks at the mechanics from low down at the back); it also goes full screen. The 3D engine (three.js) is built into the page server, so it works without internet access.

![3D view](docs/img/view3d.png)

| Rig | Side |
|---|---|
| ![](docs/img/view3d-rig.png) | ![](docs/img/view3d-side.png) |

#### Motion presets

Page 2.9.3. A panel at the top of the 3D view (at the bottom on a phone) moves the real seat by itself while you watch
it in 3D:

| Preset | What the seat does | How long |
|---|---|---|
| **Wave** | every motor in turn, a third of a cycle apart | until Stop |
| **Twist** | the front lifts opposite each other: the seat rolls left and right | until Stop |
| **Heave** | the lifts together, up and down | until Stop |
| **Corner** | the seat rolls and the rear slides out with it: a left-hander, then a right-hander | until Stop |
| **Landing** | an airliner: approach in light turbulence, a crosswind (banking and crabbing), the flare, touchdown with a rebound, the nose wheel, rollout rumble, braking, taxi | 30 s |
| **Driving** | a lap: launch and gear shifts, braking, a left hairpin over a kerb, esses, a bumpy straight, a right sweeper where the rear steps out and is caught | 36 s |

![Motion presets: Corner at 70% in a left-hander - the seat rolls 5° and the rear slides out](docs/img/motion-presets_v2.9.3.jpg)

To try them:

1. Seat empty, open **3D View**.
2. Set **Intensity** to 20-30% the first time, then press a preset - **Heave** is the gentlest start. Confirm once: the
   page takes the seat from SimHub, as Return to center does.
3. Raise **Intensity** while it runs (up to 100%: how much of each motor's room around level a preset may use - the
   room is the distance to the nearer end of its Clip window or travel limits). **Speed** (0.5-2×) sets how fast Wave,
   Twist, Heave and Corner go; Landing and Driving play at their own pace. The line under the buttons says what is
   happening (*Left hairpin*, *Flare*, *Touchdown* ...).
4. **Stop** holds the seat where it is. **Return to level** glides it back to level. **Resume SimHub** gives the
   seat back to SimHub.

Every preset starts from where the seat is and eases in over about a second, no motor moves faster than 1500 counts/s,
and each motor moves around its level - SimHub's centre, so your Seat setup's level. Landing and Driving glide back to
level when they end. A motor that switches off stops the preset and the others hold; auto-tune, the full check, Seat
setup and a page update stop it too, and Seat setup, the test signal and the USB cable wait while one runs.

### Seat setup

**Level first, then the limits.** Seat setup (firmware 1.9) does in a couple of minutes what used to take an
evening with SMC3Utils:

1. **Level the platform.** With the motion sensor, press **Level automatically**: each motor moves a little to learn
   how it tilts the seat, then the page moves them until the sensor reads level, choosing the level position nearest
   the middle of every motor. Without the sensor, nudge the motors with a spirit level on the seat. Whatever you
   accept becomes **SimHub's centre**: the board adds a level trim to every position SimHub sends, and the seat
   powers up level. On a traction-loss rig only motors 1 and 2 level the seat: you **centre** motor 3, the rear
   slide, so the seat points straight ahead, and tell the page which way the rear went for +10.
2. **Find how far each motor may go.** Say how far the seat may tilt each way (and, with traction loss, how far it
   may turn). Each motor moves slowly away from level, both ways, and stops by itself at that tilt - the rear slide
   at that turn, measured by the sensor - when it **pushes against something**, at the end of its Clip window, or
   when you press **Stop** (or the space bar).
3. **Check and save the limits.** From what was found the page suggests **travel limits** (the board never sends a
   motor past them, whatever SimHub asks), **end stops** (a motor that gets past one anyway is switched off) and
   **tilt limits** for each direction (every motor stops). A picture shows where the seat can tilt inside them;
   front up and down usually need several motors at once, so the travel can be narrowed until no combination goes
   further than you allow. For the rear slide it shows how far the seat turns each way inside its travel limits.

While Seat setup is open the board ignores SimHub's positions, so SimHub can stay running; **Cancel** puts
everything back. Auto-tune, the test signal and the full check stay inside the travel limits afterwards.

| Level | Limits | Saved |
|---|---|---|
| ![](docs/img/seat-setup-level.png) | ![](docs/img/seat-setup-limits.png) | ![](docs/img/seat-setup-done.png) |

**Start over** (page 2.9.3) sits at the top right of the Seat setup card, always there. One click and one
confirmation: it takes the set-up off the board (level trim, travel limits and end stops; the tilt limits stay),
switches every motor on - each creeps back to its position - and opens Seat setup at its first step once every motor
is back. Next to Start seat setup, **Centre the pots** starts the pot centring below.

![Seat setup: Start over at the top right, Centre the pots next to Start seat setup](docs/img/seat-start_v2.9.3.png)

**End stops past where the seat rests** (page 2.9.3). With the motors off, the seat's weight may take a corner further
than it ever goes while driving. The page notes where each motor rests with the motors off, and the limits step puts
the end stop on that side 25 counts past it (or at the SMC3 cutoff, if that is nearer) and says so under the table,
so that corner can always be switched on again.

**Seat setup is open** (page 2.9.3): whenever it is, Quick Controls says so - SimHub's positions are ignored, and
Return to center, the USB cable and the full check wait - with **Open Seat setup** and **Cancel Seat setup** right
there. Left open for 15 minutes with nothing done, it gives the seat back to SimHub by itself.

<img src="docs/img/seat-open-quick_v2.9.3.png" width="300" alt="Quick Controls: Seat setup is open, with Open Seat setup and Cancel Seat setup">

#### Centre the pots

Page 2.9.3, with the motion sensor. With the seat level, every motor should read **512** - the middle of its pot - so
SimHub has the same travel up and down from level. A level trim moves SimHub's centre but not the Clip window: a motor
that is level at 587 has 75 counts less travel one way than the other. The cure is mechanical - move the pot's belt a
tooth - and the page tells you which way and how many teeth.

Step by step:

1. Open **Seat Setup**. Under *Seat now*, click the **bubble** (it says *Click the bubble to centre the pots* when a
   motor is off centre), or press **Centre the pots**.

<img src="docs/img/seat-bubble_v2.9.3.png" width="320" alt="Seat now: Click the bubble to centre the pots">

2. The page levels the seat with the sensor (about 15 s) and shows a row per motor: a **dial** with how far its level
   reading is from 512 (green: close enough), its travel from level each way, and what to do.

![Centre the pots: a dial per motor with how far its level reading is from 512](docs/img/pots-centre_v2.9.3.png)

3. Pick a motor whose dial isn't green and press **Switch motor N off**.
4. Move that pot's belt **one tooth** on the pot's pulley - either way. Keep your hands clear of the rest of the rig.
5. Press **Switch motor N on**. The motor creeps back to its old reading, so the seat now sits a little off level.
   From the sensor the page works out the motor's new level reading and says **Right way** (with how many teeth are
   left) or **Wrong way** (go the other way). From your first tooth it learns how many counts a tooth is, and uses it
   for the other motors too.

![After one tooth on the Left motor's belt: Right way, one tooth more the same way](docs/img/pots-after_v2.9.3.png)

6. Repeat, one tooth at a time, until the dial is green. **Level again** re-levels the seat whenever you like.
7. The rear slide (traction loss) has no level, only a centre: the seat pointing straight ahead. The page starts from
   where SimHub's centre had it - check by eye, and if the seat isn't straight, centre it with **−10 −1 +1 +10** and
   press **Centre is here**. After a tooth on its belt, the page works out the new centre from the sensor's yaw.
8. Press **Finish**. If a belt moved, the old set-up no longer fits: it comes off the board and Seat setup carries on
   from levelling - run **Level automatically**, then find the travel and save. If nothing moved, the old set-up comes
   back. Nothing is saved before you finish.

### Park

Page 2.9.3, firmware 1.9 or newer; firmware 2.1.1 lets the park position go past the travel limits. With the motors
off, a heavy corner can sink while friction holds the other, and the seat sits twisted until the next start. **Park**
lowers the seat evenly first - every motor arriving at its park position at the same time - and switches the motors
off there, where nothing sinks any further.

<img src="docs/img/park-quick_v2.9.3.png" width="560" alt="Quick Controls: the (P) at the top right, and lit when parked">

**The (P) in Quick Controls** (top right of the card):

- **Press it to park.** It asks first, then the seat goes down evenly at 60 counts/s (for the motor with the furthest to
  go) and the motors switch off. SimHub waits meanwhile.
- **Lit** means parked.
- **Press it again, or Enable Motors, to unpark.** The seat rises evenly back to level, the same way it went down, and
  then SimHub has it. If a motor can't get back up, the page says which one and where it stopped, and switches the
  motors off again.
- **While it pulses** (parking or unparking), pressing it stops: the motors switch off where the seat is.

**After every park the page checks** that each motor got to its park position (or rests against its stop short of
it), that nothing moved after the motors switched off, and - with the motion sensor - that the seat sits at the same
angles as when the park position was saved (if not, a pot or its belt may have moved). The message shows in the
**Park position** card and as a pop-up.

#### Calibrating the park position

The **Park position** card is in **Seat Setup** and in **Settings**. Until you set one, the park position is the low
end of each lift's travel window, with the rear slide centred. To set your own:

1. Seat empty, motors on. Press **Set the park position**. The page takes the seat from SimHub, and each motor's row
   gets arrows, the **Park at** position and where it is **Now**.
2. Press **Lower the lifts 10** (or **Lower 2** for small steps) until the seat sits where it rests by itself when the
   power is off: on its lowest point, its stops, or wherever it settles. "Down" is the way each corner sinks - from
   the full check's answer, or else from where it rests with the motors off. **Raise 10** goes back up; the
   **▼ ▲** arrows on a row move one motor on its own (the rear slide gets −10 −1 +1 +10).
3. With the motion sensor, press **Level side to side**: it evens the seat out using how each lift tilts it (Seat
   setup measured that). Press it again until the roll is close to 0.

![Park position card while setting it: arrows per motor, Lower the lifts, Level side to side, Save](docs/img/park-setting_v2.9.3.png)

4. Press **Save this as the park position**. The positions are kept, and with the sensor the seat's angles there.
5. Press **Park here now** to try it, or **Back up (cancel)** to glide back to where the seat was and give it to
   SimHub.
6. Read the message after parking. *Nothing moved when the motors switched off* is what you want. If it says a corner
   still moved, set the park position lower on that corner (its ▼ arrow) and save again.

![Park position card after parking: the saved position, and nothing moved when the motors switched off](docs/img/park-parked_v2.9.3.png)

You can also type the positions into the **Park at** boxes and press **Save typed values**, change the **Speed**
(10-300 counts/s), or press **Default** to forget your park position.

**How far down:** with firmware 2.1.1 the board has a **park mode**: while the page parks the seat or you set the park
position, those motors may go past their travel limits, Clip window and end stops - anywhere inside the SMC3 cutoff
(Max limit). SMC3's PWMrev push back into the Clip window is left out and the tilt limits wait. Park mode lasts only
while the page refreshes it every second, and lapses 3 s after the last refresh. With older firmware the park position
stays inside the travel window, and unpark is Enable Motors: each motor creeps back up by itself.

### Tuning

Every SMC3Utils setting is here, per motor: Kp, Ki, Kd, Ks, PWMmin, PWMmax, PWMrev, dead zone, the safety
cutoff ("Max limit") and Clip. You also get the loop divider, the PWM frequency, and, with firmware 1.6 or newer,
**target smoothing** and a **max speed** cap.

**Apply** changes a motor live. Nothing is stored on the board until you press **Save to EEPROM**.

![Tuning view](docs/img/tuning.png)

### Test signals

Drive one motor from the page with a sine, square, triangle or step, or hold it at a position. Use this to see
overshoot and lag before you touch a game. Stop SimHub's motion output first.

**Several motors at once** (page 2.9): the **Together** list and the Presets run one signal on several motors, each
around its own centre - **Twist** moves the two front lifts opposite each other (one goes down while the other goes
up: the most twist), **Heave** the lifts together, **Corner** the front lifts opposite each other with the rear slide,
**Wave** every motor in turn, a third of a cycle apart.

Every signal **starts and stops gently** (page 2.9.1): each motor fades in from where it is over half a period, and
**Stop** eases it back to its centre. A new amplitude or offset slides in while it runs, and a new period carries on
from where the cycle is. Only the **Step** preset steps.

![Test Signal with the Together list, and the Twist, Heave, Corner and Wave presets](docs/img/presets-together_v2.9.png)

![Twist running: Left and Right move opposite each other and the seat rolls](docs/img/twist-dashboard_v2.9.png)

![Square test on motor 1](docs/img/test-tools.png)

### Auto-tune

Auto-tune works on one motor at a time and tests it **both ways**: up and down for a lift, left and right for the
rear slide. Every test goes **out and back from where the seat starts**, the way you'd move a knob on the
dashboard: up, back to the start, down, back to the start (right, back, left, back for the rear slide). So the seat
never goes further than the test travel from where it was, and each direction is measured going away from the start
and coming back to it.

1. It measures the pot noise (dead zone), and the smallest drive that moves the motor each way (PWMmin).
2. It measures the motor's top speed each way with one quick move. Every test after that is a **gradual move** at
   60% of the slower way's top speed, easing in and out, so the motor can follow it as it follows a game.
3. It raises Kp step by step until the motor follows within the style's lag target both ways, or starts to
   overshoot.
4. It checks that small, sharp, kerb-sized steps (±30 counts from the start) stay calm.
5. It adds Ki and Kd only if they clearly help.

Each direction is judged on its own, and the log and the result show both. The board has one set of values per
motor, so the worse direction decides.

**Who's in the seat while it tests:**

- **Nobody**: tune with the seat empty and enter your weight, and auto-tune allows for it. It sets a stricter
  overshoot limit and a stiffer Kp, runs slower test moves, and keeps a little Ki. Then press **Check** with you in
  the seat: it tests the current values both ways, changes nothing, and says what to change.
- **Me**: Start gives you 15 s to get in. Stop, Space or Esc ends it at once.

**Auto-tune live** shows each test as gauges:

- how far the motor trails the move (lag), overshoot, settling, how far short it stops, ringing, and how much of
  PWMmax it takes;
- one needle per direction, with a coloured band for what's good.

The same card shows pot noise, the minimum drive and top speed each way, and the Kp sweep as two small charts
that scale to the results; the tabs switch the second one between overshoot, off by and settle time.
**Copy** puts the whole log and the result on the clipboard.

With the motion sensor, auto-tune also watches the **seat**: the seat's own overshoot and ringing count, so it
won't pick a Kp that keeps the motor calm but makes a flexing frame bounce.

There are three styles: **Gentle**, **Normal** and **Firm**. It stops by itself if SimHub moves the rig or
anything looks wrong, such as the motor moving the wrong way or running away. Nothing is saved until you press
Save to EEPROM, and **Revert** undoes it.

It measures the pot's noise first, and every limit that judges "it moved" or "it holds still" allows for it, so a
noisy pot doesn't look like a motor going the wrong way or ringing. A motor is only called wired backwards when it
goes the wrong way **both** ways: a seat that sinks under its weight while the drive is still too weak goes the
wrong way one way only. If it does stop for safety, that motor is switched off with its previous settings back.

**Test together** (page 2.9) runs every motor in use with the values it has now: each alone, then all together,
then twisted. It shows what moving together costs each one - more lag or running out of drive points to a shared
supply that sags, more overshoot or ringing to a frame that flexes - and changes nothing.

![Test together: each motor alone, all together, then twisted](docs/img/test-together_v2.9.png)

A crackling pot (bursts of readings a few counts off, whatever the Kp) doesn't end the Kp sweep early (page 2.9): the
hold is judged over most of a second with a median spread, against what the pot does at the gentlest Kps.

**A lift that sinks under its own weight** (page 2.9.1) - a gearbox that the seat can drive backwards, or a heavy
corner - goes down with no drive at all, so "the easier way" there is gravity, not the motor's friction. Auto-tune
sees it in the minimum-drive step and sets PWMmin from what lifting takes (80% of it), holds the seat with a Kp that
can, and doesn't take the seat sinking under a weak drive for a motor wired backwards. If the seat sinks towards what
your full-check answer calls "up", it says the answer looks the wrong way round and uses the other way for the run.
The quick move for the top speed eases in instead of jumping, and the noise at rest is measured three times.

| Checklist first | Running | Result |
|---|---|---|
| ![](docs/img/autotune-checklist.png) | ![](docs/img/autotune-running.png) | ![](docs/img/autotune-result.png) |

### Recommended starting points (profiles per kind of sim)

What feels right for circuit racing is wrong for a flight simulator. Each recommendation starts from **your own**
settings (ideally after auto-tune) and adjusts how firm, damped, fast and smooth the rig feels for that kind of
sim. It also lists the **SimHub settings** that matter most for it. You see a before-and-after table of every
change before anything is applied. Apply it, save it as a profile, or both.

| | Kp | Ki | Kd | PWMmax | Dead zone | Smoothing | Max speed |
|---|---|---|---|---|---|---|---|
| **Circuit racing** | as tuned | as tuned | as tuned | as tuned | as tuned | 8 ms | off |
| **Rally & off-road** | ×1.15 | as tuned | at least 10 (Ks 4) | +15 | as tuned | 4 ms | off |
| **Flight** | ×0.75 | at least 15 | as tuned | −15 | +1 | 32 ms | 1500/s |
| **Space & sci-fi** | ×0.85 | at least 10 | as tuned | −10 | +1 | 16 ms | 2000/s |
| **Trucks & cruising** | ×0.8 | at least 5 | as tuned | −10 | as tuned | 32 ms | 1500/s |

<table>
<tr><td><b>Circuit racing</b>: GT, formula, touring. Crisp and detailed, your tuning as it is.<br><img src="docs/img/recommend-circuit.png"></td>
<td><b>Rally &amp; off-road</b>: gravel, jumps, rough roads. Sharper, with damping.<br><img src="docs/img/recommend-rally.png"></td></tr>
<tr><td><b>Flight</b>: planes and helicopters. Smooth, held attitudes, with washout in SimHub.<br><img src="docs/img/recommend-flight.png"></td>
<td><b>Space &amp; sci-fi</b>: Elite, Star Citizen. Floaty but responsive.<br><img src="docs/img/recommend-space.png"></td></tr>
<tr><td><b>Trucks &amp; cruising</b>: ETS2, ATS, relaxed driving. Gentle and heavy.<br><img src="docs/img/recommend-truck.png"></td>
<td><b>Profiles</b>: save every board setting under a name, one per car or game, and load it in a click.<br><img src="docs/img/profiles.png"></td></tr>
</table>

### EEPROM and backups

Every Save to EEPROM, and every bigger change (loading a profile or recommendation, starting auto-tune,
switching backups), keeps a copy of the settings on the NAS. **Switch to** puts a backup on the board live;
**Switch + save** also stores it in EEPROM. The backup that matches the board right now is marked.

![EEPROM and backups](docs/img/eeprom-backups.png)

### Settings and motor names

In Settings you set:

- the board's address, plus a **Find board on network** scan and auto-find if the address changes;
- the theme;
- **how many motors you use** (1 to 3). Unused motors can also be switched off on the board.

Under **Motor names** you can call each motor by where it sits: Front left, Front right, Traction loss, or anything
you like. The names show everywhere in the page and in diagnostics, always next to the number, because the wiring,
SimHub's outputs and the LED matrix codes go by number.

The same card says what **motor 3** does:

- **Slides the rear left / right (traction loss)**, the default: motors 1 and 2 lift the front corners, and motor 3,
  mounted horizontally, swings the rear about a pivot at the front. Also which way the rear goes for higher numbers
  (Seat setup and the full check ask you).
- **Lifts the seat, like motors 1 and 2**: three lifts.

Seat setup, the full check, auto-tune, the recommendations and the 3D view follow it.

**Position sensors** (page 2.9.1): potentiometers, or contactless Hall-effect sensors such as the TT Electronics
6127. Diagnostics and auto-tune give the advice that fits: a pot's wiper can wear and lose contact, a contactless
sensor can't - its jumps come from the wiring, its 5 V and GND, or the motor supply.

<img src="docs/img/position-sensors_v2.9.1.png" width="520" alt="Motor names card with Position sensors: Contactless Hall sensors">

![Settings with motor names](docs/img/settings.png)

**LED matrix** and **Motion sensor** (firmware 1.8) are set here too, and saved on the board at once:

- what the matrix shows;
- switching the sensor on;
- how it is mounted;
- **Level now**;
- the **tilt limits** for each direction (firmware 1.9; one for all with 1.8), which stop every motor if the seat
  tilts further than it should;
- **Set up the sensor**: 100 readings a second with gyro and acceleration (it leaves the factory at 10).

The status line says whether the sensor is found, at what baud rate, how fast it sends and whether gyro and
acceleration arrive.

![LED matrix and motion sensor settings](docs/img/settings-board.png)

Since page 2.9.3 the status is **one steady line**: *Receiving 100 angles/s at 115200 baud + gyro + acceleration*
(amber, with what to do, if it dropped out in the last 2 minutes), or *Looking for the sensor: trying 57600 baud…*
with when the angles last came - red only after 25 s with nothing. **Set up the sensor** goes out only while the board
receives the sensor, then checks what took and says so (*Done: 100 angles/s at 115200 baud, with gyro and
acceleration*, or which part didn't take). With firmware 2.1.1 the board also keeps the sensor all the time while
**Use the sensor** is ticked.

**Motion sensor check** (page 2.9.3): the **Check (20 s)** button, also in Diagnostics. Nothing moves. It records
20 s of the link between the board and the sensor and shows it as a strip - green while angles arrive, amber while
the board looks for the sensor, labelled with the baud rate it tries - with a report and a CSV download:

- whether angles arrive all the time, and every gap: how soon the board switched rate after the last angles, and
  where it found the sensor again;
- packets and bad checksums per second, and whether the sensor sends angles only or gyro and acceleration too;
- the noise and drift of its angles, acceleration and gyro at rest.

It tells a wiring problem (nothing at all arrives) from a sensor at a baud rate the board doesn't try, and from a
link that keeps dropping out. With firmware 2.1.1 the board adds its own counts (`[rdj]`): bytes received at any
rate, times it lost the sensor, baud changes and the packet types - so even a silent sensor and garbage on D0 are
told apart.

<img src="docs/img/sensor-card_v2.9.3.png" width="620" alt="Motion sensor card: receiving 100 angles/s at 115200 baud with gyro and acceleration, and the Check (20 s) button">

<img src="docs/img/sensor-link-check_v2.9.3.png" width="740" alt="Motion sensor check: received all the time, the link strip green for the whole 20 s, Download recording (CSV)">

**Firmware update** (firmware 1.9.2): install new firmware over WiFi - no USB cable, and SimHub can stay open.

1. In the Arduino IDE, open the new `SMC3_R4.ino` and choose **Sketch, Export Compiled Binary**. The WiFi name and
   password may stay as placeholders: the board remembers the ones it last joined with.
2. In the page, choose that `.bin` (in the sketch's `build/arduino.renesas_uno.unor4wifi` folder) and press
   **Install**, with the seat empty and SimHub's motion output paused.

The page checks the file first (an SMC3_R4 build for the UNO R4 WiFi, which version) and converts it to the board's
update format. The board switches the motors off, its WiFi module downloads the file, checks it and programs the
board - the same way a USB upload does - and the board restarts; the seat creeps back with the soft start and the
page confirms the new version. If anything fails before programming, the old firmware keeps running and the card
says why. The first install of it (1.9.3 or later; 1.9.2 didn't compile) goes over USB, and the WiFi module needs its
own firmware 0.4.0 or newer (Arduino IDE, Tools, Firmware Updater).

![Firmware update: SMC3_R4 1.9.3 installed over WiFi](docs/img/firmware-update.png)

**Restart** (page 2.9, firmware 2.0): the button next to Settings restarts the board as its RESET button does - the
motors switch off first, and the page reconnects by itself. It is refused while SimHub sends positions, and while
auto-tune, Seat setup or the full check runs.

![The Restart button next to Settings, and what it asks first](docs/img/restart_v2.9.png)

**Updates** (page 2.7): the page asks GitHub twice a day what the newest release has. When there's a newer page or
firmware, a balloon on the header's **Settings** button says **Update**; it leads straight here. The header shows the
page's version next to the board's firmware version.

- **Update page** downloads the new page, checks it (GitHub's SHA-256, its version, that Python can read it) and
  restarts into it - the board isn't touched and settings stay. It's kept in the data folder, so it survives
  restarts and re-created Docker containers; **Go back to built-in** undoes it, and an update that doesn't start is
  dropped by itself.
- **Download firmware** downloads the new `.ino` only - flash it with the Arduino IDE, or export the `.bin` and use
  Firmware update above.

![Updates: a newer page and firmware on GitHub (a test release)](docs/img/updates.png)

### LED matrix

The board's 12x8 matrix shows one thing at a time, readable from across the room. These previews are rendered from
the firmware's own drawing code:

| Power-up: the address | Motor bars | Seat horizon (sensor) | Idle | A problem |
|---|---|---|---|---|
| <img src="docs/img/matrix-boot.gif" width="160"> | <img src="docs/img/matrix-bars.gif" width="160"> | <img src="docs/img/matrix-horizon.gif" width="160"> | <img src="docs/img/matrix-idle.gif" width="160"> | <img src="docs/img/matrix-error-cutoff.gif" width="160"> |

- **Motor bars:** each motor's position, live. A dot blinks where a motor should be when it falls behind. A dash
  at the bottom means it's switched off on purpose.
- **Problems** scroll in words, again and again while they last: `ERROR E3 MOTOR 3 CUTOFF`, `ERROR E4 TILT LIMIT`,
  `ERROR E9 NO WIFI` and so on. The page's Help explains every code and what to do.
- **While idle** (firmware 2.1): after 20 s without positions the matrix can play one of five little games that play
  themselves, or a different one each time - instead of the slow trace. The motor bars are back the moment a position
  arrives. Settings, LED matrix, **While idle** has a live preview (the firmware's own game code, in the browser), and
  **Apply** has the board play it for 15 s at once.

| Ping pong | Brick breaker | Missile defense | Snake | Meteor dodge |
|---|---|---|---|---|
| <img src="docs/img/matrix-ping-pong_v2.1.gif" width="160"> | <img src="docs/img/matrix-brick-breaker_v2.1.gif" width="160"> | <img src="docs/img/matrix-missile-defense_v2.1.gif" width="160"> | <img src="docs/img/matrix-snake_v2.1.gif" width="160"> | <img src="docs/img/matrix-meteor-dodge_v2.1.gif" width="160"> |

<img src="docs/img/led-idle_v2.9.1.png" width="560" alt="Settings, LED matrix: While idle - Snake, with the live preview">

![LED matrix error codes in Help](docs/img/help-errors.png)

### Diagnostics

For when something isn't right: a motor going the wrong way, sudden jerks, a motor that won't move, the rig
restarting. Describe the problem, run a check, and download **one self-describing file** to give to whoever
helps you. You can also upload it to an AI assistant such as Claude.

**Quick check:** nothing moves, and it's safe while SimHub runs. It reads every setting and compares it with
what's saved in EEPROM, reads the board's own counters, listens to the motors for 3 seconds, and adds everything
the page noticed while you drove.

**Full check:** moves each motor a little, one at a time, with the others switched off meanwhile. For each motor
it finds:

- which way the motor really moves;
- how much drive it needs to start moving each way;
- whether another motor moves instead, which means swapped motor or pot wiring;
- how it handles a small step with its own settings.

It also **asks you what you saw**: which way each corner went, and where it is on the seat - for the traction-loss
motor, whether the rear went left or right. That's the one thing sensors can't tell, for example whether the motion
will be upside down in games. A motor that moves the wrong
way is stopped at once and left off.

It also notes which way each lift's numbers run when its corner **sinks** under the seat's weight (drifting while
its motor is off, or sinking at low drive). Until you answer the question, auto-tune goes by that for up and down,
and the results warn when the two front lifts seem to run opposite ways (also when one needs clearly less drive one
way - the weight helps it down) - one of them would then move the wrong way in games. It also says when your answer
disagrees with how a corner sinks, so you can look again.

| Before it starts | "Watch now", then the question |
|---|---|
| ![](docs/img/diag-confirm.png) | ![](docs/img/diag-watch.png)<br>![](docs/img/diag-question.png) |

The results come in plain words, each with **what to do**, naming the wire and pin where it can:

![Diagnostics results: reversed motor and glitchy pot found](docs/img/diag-results.png)

**A watcher that never sleeps.** While you drive, the page notices problems by itself:

- pot readings that jump faster than any motor can move (the classic cause of violent jerks);
- a motor moving away from its target;
- a motor pushed hard that doesn't move;
- hunting at rest;
- motors switching themselves off, including the tilt limit;
- board restarts;
- gaps in the data;
- the motion sensor going quiet.

**Where the rig struggles.** While SimHub drives, the page also keeps a **load map** for each motor: in eight
bands across its travel, and in each direction, how far it runs behind and how often it's at full power. Lifting a
heavy driver shows up as one direction falling behind at full power while the other keeps up. The quick check names
the band and, with the sensor, the seat angles there:

![Quick check: motor 1 struggles lifting](docs/img/diag-load.png)

With the motion sensor, the full check also measures what the **seat** does on each step: how many degrees each
motor tilts it (the traction-loss motor: how far it turns it), whether it **bounces** after the motor stops (flex),
and whether there is **play** in a linkage. A lift that turns the seat, or a traction-loss motor that tilts it, is
flagged: the wiring or the setting doesn't match the rig.

Each one keeps a few seconds of data from before and after. A badge and a System Status line tell you about it,
and the **event log** shows what led up to it.

<img src="docs/img/noticed-badge.png" height="560" alt="Badge and System Status"> <img src="docs/img/diag-side.png" height="560" alt="Problems noticed, saved reports and event log">

**Sensor check** (page 2.9.1): nothing moves. It records 10 s of every motor under one condition you set up -
**A** motor power switched off, **B** motor power on but the motors switched off (Disable Motors), **C** holding -
and reports the feedback's noise (quietest and worst second), drift, impulses and bursts (their lengths from the
samples' timestamps), a summary of the board's own raw readings over the same time (lowest, highest, biggest step
and jumps, at 4 kHz ÷ Fpid divider), an estimate of how much of the PID output's jitter is the P term and how much
the rest (D and I, with the board's rounding), and how often the motor was braked or driven. With two or three
conditions recorded, it puts them side by side and reports all three parts: the sensor side on its own (A), what the
motor power adds (B over A) and what holding adds (C over B) - they add up, so a rig can have some of each. Each
check is saved, the recording also as CSV.

![Sensor check: motor 1 quiet with the motor power off, noisy with it on - the A, B and C checks side by side, with what each condition adds](docs/img/sensor-check_v2.9.2.png)

**Motion sensor check** (page 2.9.3): 20 s of the sensor link, nothing moves - see
[Settings and motor names](#settings-and-motor-names). The quick and full checks also say (page 2.9.3) when a motor
rests past its end stop or past its safety cutoff with the motors off, and when a contactless sensor reaches the end of
its output.

See [docs/DIAGNOSTICS.md](docs/DIAGNOSTICS.md) for every check and the file format, and
[docs/examples/diagnostics-full-check.json](docs/examples/diagnostics-full-check.json) for a real file from the
simulated rig with a reversed motor, a glitchy pot and play in a linkage.

### On a phone

<img src="docs/img/phone-dashboard.png" width="300" alt="Phone dashboard"> <img src="docs/img/phone-diagnostics.png" width="300" alt="Phone diagnostics">

The Help section in the page sums everything up, including every LED matrix code.

![Help](docs/img/help.png)

---

## Hardware

- **Arduino UNO R4 WiFi**. The firmware is written for it: WiFi, LED matrix and the RA4M1's timers.
- Motor bridges: **IBT-2 / BTS7960** (`MODE2`, the default), or a bridge with PWM plus two direction pins
  (`MODE1`).
- Up to three DC motors with feedback pots, such as worm-gear wiper motors on crank arms or linear actuators, and
  a motor supply that suits them. Typical layouts: two lifts at the front plus a horizontal **traction-loss** motor
  sliding the rear (SimHub's 2DOF + traction loss), or three lifts.
- Optional: a WitMotion **WT901C** motion sensor on the seat. The -232 model connects through a MAX3232 module
  (RS-232 to TTL) to D0 and D1 - **never straight to the pins**: RS-232 levels damage the board. It gives seat
  angles, automatic levelling, tilt limits, the 3D view, and seat measurements in auto-tune and diagnostics. It is
  set up from the page.

![WT901C-232 wiring: DB9 plug with screw terminals, MAX3232 module, UNO R4 WiFi](docs/img/sensor-wiring.png)

Wiring is the same as the original SMC3 on an UNO, so existing rigs need no changes. The whole rig - board, three
IBT-2 bridges, motors with their pots, the motion sensor and the motor supply - in one drawing (also in the page's
Help):

![Wiring the whole rig: UNO R4 WiFi, three IBT-2 bridges on D2-D11, pots on A0-A2, MAX3232 and WT901C on D0/D1](docs/img/wiring.png)


| | PWM | Bridge pins (ENA / ENB) | Feedback pot (wiper) |
|---|---|---|---|
| Motor 1 | D9 | D2 / D3 | A0 |
| Motor 2 | D10 | D4 / D5 | A1 |
| Motor 3 | D11 | D6 / D7 | A2 |

On an IBT-2 (`MODE2`): **PWM** goes to its LPWM, **ENA** to RPWM, and **ENB** to R_EN and L_EN joined; its VCC and GND
to the board's 5V and GND; R_IS and L_IS stay free; B+ / B- to the motor supply and M+ / M- to the motor. The pots'
outer pins go to 5 V and GND.

### Pot noise filter

Diagnostics counts **pot glitches**: readings that jump further in 1/4 ms than any motor can move. Most come from the
motor PWM getting into the pot wires, and each one makes the PID kick the motor for a moment. Firmware 1.9.4 holds
a glitch back in software (the board still counts them), and three small parts per pot at the Arduino stop most of
them reaching the board at all:

![Pot noise filter: 1 kΩ in the wiper wire, 100 nF and 1 MΩ from the A pin to GND, shielded cable](docs/img/pot-filter.png)

| Part (for three pots) | Where | What it does |
|---|---|---|
| 3 × resistor 1 kΩ, ¼ W (R1) | in the wiper wire, at the Arduino | with C1: a low-pass filter (0.1 ms) - the 25 kHz PWM spikes go, the motor's own movement passes untouched |
| 3 × ceramic capacitor 100 nF, 50 V, X7R, marked 104 (C1) | A pin to GND | |
| 3 × resistor 1 MΩ, ¼ W (R2) | A pin to GND | a broken wiper wire reads 0, so the cutoff still trips (otherwise C1 would hold the last position) |
| perfboard, or a screw-terminal proto shield | next to the Arduino | keeps the parts close to the pins |
| 3-core shielded cable, about 0.25 mm² (22-24 AWG), foil or braid with a drain wire | each pot run | the shield goes to GND **at the Arduino end only** |
| optional: clip-on ferrites | motor cables, near the IBT-2 boards | less noise from the motor wires |

Keep C1 at 1 µF or less: a bigger one delays the reading and the motors start to hunt. Wiring rules that cost
nothing: the pot GND goes straight to an Arduino GND pin (not to the IBT-2 or the motor supply), pot cables stay
5 cm or more from motor cables and cross them at right angles, and each motor's two wires are twisted together. A
pot that still glitches is losing contact (wear) - no filter fixes that: fit a conductive-plastic pot (for example
Bourns 6639S-1-103, 10 kΩ) or a contactless Hall-effect sensor with a 0-5 V output (for example TT Electronics
6127V1A180L.5FS), keeping its travel away from the ends of its range. The same drawing is in the page's Help.

**Contactless sensors** have no wiper, so when one jumps it's the wiring: its 5 V and GND (straight to the Arduino,
never through a motor bridge's ground), a crimp or solder joint, or the motor PWM getting into the signal wire. The
same filter fits them (a 6127 drives a load of 1 kΩ or more), plus 100 nF across the sensor's 5 V and GND right at
the sensor. A 6127 with output code V1 puts out 0.2-4.8 V over its electrical angle (180° for an A180): keep the
motor's travel inside it. Choose them under Settings, Motor names, Position sensors, and use Diagnostics, **Sensor
check** with the motor power off and on to see whether the noise is the sensor's or the motor supply's.

## Getting started

### 1. Flash the firmware

1. In the Arduino IDE, install the board package **Arduino UNO R4 Boards** and select **Arduino UNO R4 WiFi**.
2. Open `firmware/SMC3_R4/SMC3_R4.ino` and put your **WiFi name and password** into `WIFI_SSID` / `WIFI_PASS`
   near the top. The board only does 2.4 GHz. Check `MODE2` / `MODE1` and the other options listed in
   [firmware/README.md](firmware/README.md).
3. Upload. At power-up the board's IP address scrolls across the LED matrix. Reserve it in your router.

Firmware 2.0 then takes the board's USB connector for the main chip: Windows plays its unplug and plug sounds and a
new COM port appears - that is the one for SimHub. To upload again with the IDE, first press **WiFi
chip until the next reset** in the page's Settings, USB cable (or press RESET twice, about a second apart); see
[firmware/README.md](firmware/README.md#flashing).

From firmware 1.9.2 on, later versions can be installed from the page instead (Settings, Firmware update).

Your SMC3Utils settings stay in EEPROM in the same layout. If the board had none, the defaults are written.

### 2. Run the page

Step by step for Windows, Mac, Linux, Raspberry Pi, Docker and Synology: **[docs/INSTALL.md](docs/INSTALL.md)**.
In short:

**Docker, ready-made image** (a NAS or any PC; amd64, arm64 and arm/v7):

```sh
docker run -d --name smc3-scope --restart unless-stopped -p 8198:8198 -e SMC3_WEB_PORT=8198 -v smc3-scope-data:/data ghcr.io/winreboot/smc3-scope:latest
```

Or with Compose, from [scope/compose.prebuilt.yaml](scope/compose.prebuilt.yaml): `docker compose -f
compose.prebuilt.yaml up -d`. To build the image yourself instead: `git clone
https://github.com/winreboot/smc3-r4.git`, `cd smc3-r4/scope`, `docker compose up -d --build`.

Then open `http://<that machine's IP>:8198`, go to **Settings** and enter the board's address (or press **Find
board on network**).

**Without Docker** (Python 3.8 or newer, nothing to install): download `smc3_scope_v<version>.py` from the
[latest release](https://github.com/winreboot/smc3-r4/releases/latest) and run

```sh
python3 smc3_scope_v2.9.3.py --host 192.168.1.50 --bind 0.0.0.0 --web 8198
```

(`py` instead of `python3` on Windows). `--bind 0.0.0.0` makes the page reachable from other devices; without it,
the page is on this machine only - and the board can't fetch firmware updates from it.

From then on the page updates itself: Settings, **Updates**, **Update page** (page 2.7 or newer).

### 3. SimHub

In SimHub, set the motion output to an **SMC3** device on the board's COM port (with firmware 2.0 the new one
that appeared after its first start), exactly as with an UNO R3. Only
one program can use the COM port, so close SMC3Utils and the Arduino serial monitor. The page never needs the COM
port while SimHub drives. With a traction-loss rig, SimHub's traction-loss (yaw / slip) output goes to motor 3; if the rear swings the
wrong way in a slide (in a right-hand slide it should swing out to the left), tick Invert on that output.

### 4. Seat setup

Open **Seat Setup**: level the platform (by itself with the motion sensor), centre the rear slide if you have
traction loss, let each motor find its safe travel, and save. SimHub's centre is then the level seat, and the limits protect the rig whatever drives it.

With the motion sensor, run [Centre the pots](#centre-the-pots) first: then every motor has the same travel up and
down from level. Afterwards set the [park position](#calibrating-the-park-position), and park the seat with the
**(P)** before you switch the rig off.

### 5. Tune

Run **auto-tune** on each motor: seat empty first, with your weight entered. Then either **Check** with you in the
seat, or run auto-tune again with you in it. Try it in a game, and **Save to EEPROM**. Over WiFi a hiccup now and then
makes auto-tune redo a test move; for a steadier link close SimHub and use the **USB cable**: flip the switch at the
top of the page to USB (page 2.9; firmware 2.0 recommended).

**Soft start** (firmware 1.9.1): after the rig has been off the seat may have ridden down a little. At power-up, and
when you press Enable Motors, each motor creeps back to its position instead of jumping there, then runs normally.
The speed (60 counts/s by default, or off) is under Settings, Loop, PWM & smoothing. With firmware 2.1.1 a motor that
rests past its end stop creeps back too (at 60 counts/s even with the soft start off), with up to PWMrev's drive while it
lags behind. Then
try the **recommendations** for the sims you play and save them as profiles.

## Try it without a rig

```sh
python3 tools/simboard.py 4210 --tuned &
python3 scope/smc3_scope.py --host 127.0.0.1 --port 4210
# open http://localhost:8080
```

The simulator speaks the same protocol as the firmware, using a worm-gear motor model with friction, lag and pot
noise. To see what the diagnostics find, add a fault:

| Option | What it simulates |
|---|---|
| `--reverse 2` | motor 2's wires swapped |
| `--swap-motors 1,2` / `--swap-pots 2,3` | channels mixed up |
| `--glitch 3` / `--noisy 2` | worn or noisy pot |
| `--dead 3` / `--slip 1` / `--oneway 1` | no motor power / pot slipped on its shaft / half a bridge dead |
| `--hunt 1` / `--pwmmax0 2` | unstable settings / PWMmax 0 |
| `--simhub` | a fake SimHub driving all three motors |
| `--layout lifts` / `--tl-left` | three lifts instead of front lifts + a traction-loss rear slide (the default) / the rear slide goes left for higher numbers |
| `--reboot-at 30` | the board restarting |
| `--fw 16` | older firmware behaviour |
| `--imu 100` (or `10`) | a WT901C on the seat, sending 100 (or the factory 10) angles per second; `--sensor-on` switches it on |
| `--flex` / `--slop 2:24` | a bouncy seat frame / 24 counts of play in motor 2's linkage |
| `--heavy 1` | a heavy load on motor 1: lifting needs much more drive, worse near the top |
| `--pot-noise 2:2.3` | motor 2's pot reads ±2.3 counts at rest, with the odd spike |
| `--sag 2` (`--sag-pull 0.2`) | motor 2's gearbox can be driven backwards: the seat sinks while the drive is weak (harder) |
| `--pot-glitches 2:80` | motor 2's pot wiper loses contact about 80 times a second (readings 16-40 counts off); `--no-potfilter` = firmware 1.9.3, whose PID sees every glitch |
| `--sag-high 1` | like `--sag`, but the weight pulls motor 1 towards higher numbers (a lift whose numbers run the other way) |
| `--load 82` | an 82 kg rider: the lifts need more drive going up and less going down, and every motor carries more weight |
| `--vmax 1000` | slower motors (counts/s at full drive; default 2200) |
| `--imu-lost-at 30` | the sensor goes quiet after 30 s |
| `--seat-tilt 1.6:-2.2` | the seat isn't level with every motor at 512 (for Seat setup) |
| `--stops 3:180:860` | motor 3 hits a mechanical stop at pot 180 and 860 |
| `--seat-setup` | start with a saved seat set-up (level trim, travel limits, end stops, tilt limits) |
| `--usb PATH` | the board's USB port as a pseudo-terminal at PATH, for the bridge (`--usb-bridge PATH`) |
| `--fw20` / `--fw20-esp` | firmware 2.0 with its own USB port / set to classic (the WiFi chip has the connector) |
| `--usb-stall 30` | the WiFi chip's USB stall: 30 s after the cable link starts nothing comes back until the port re-enumerates |
| `--pot-crackle 2:3` | motor 2's pot crackles about 3 times a second (bursts the glitch filter can't catch) |
| `--supply-sag 2.5` | a weak shared motor supply: every motor gets less drive when they all pull at once |
| `--fw211` | firmware 2.1.1: park mode, coming back up from past an end stop, the board's sensor link report |
| `--pot-offset 1:-60` | motor 1's pot belt sits 60 counts round (for Centre the pots); `--tooth 37`: counts per belt tooth |
| `--lift-hard 2:140:0.35` | lifting motor 2 needs more drive past pot 140 (the sunk end of a `--sag` lift: a corner PWMmax alone can't lift) |
| `--imu-baud 9600` / `--imu-angles-only` | the sensor's baud rate at start / it sends angles only |
| `--imu-race` | firmware 2.1's sensor timing: the board loses the sensor every few seconds while it is switched on |

## Compatibility

- The board answers like SMC3 v0.70 (`[ver]` returns 70), and the EEPROM layout is unchanged. SimHub, SimTools,
  FlyPT Mover and SMC3Utils see an ordinary SMC3.
- The page works with every firmware from 1.3 on. Newer firmware unlocks more:
  - 1.4: auto-tune, and Check;
  - 1.5: Disable Motors;
  - 1.6: smoothing settings;
  - 1.7: full diagnostics;
  - 1.8: the LED matrix in words, and the motion sensor;
  - 1.9: Seat setup (level trim, travel limits, end stops, tilt limits for each direction), gyro and acceleration
    for the 3D view, moving the motors from the page (page 2.4);
  - 1.9.1: soft start;
  - 1.9.2: firmware updates from the page, and the board remembers its WiFi;
  - 1.9.3: the same, and it compiles in the Arduino IDE (1.9.2 didn't);
  - 1.9.4: a pot glitch filter - a reading too far from the last one to be the motor is held back for up to 10 ms;
  - 1.9.5: USB page mode - the page can use the USB cable instead of WiFi (calibration with SimHub closed);
  - 1.9.6: no more lost bytes on the USB port (the serial receive interrupts come before the PID interrupt).
  - 2.0: its own USB port - the main chip has the USB connector, so the WiFi chip's USB stall and lost bytes are
    gone; the page's Restart button; nothing restarts the board or its chips while SimHub sends positions.
  - 2.1: games on the LED matrix while idle.
  - 2.1.1: park mode - the page may take the seat past the travel limits to park it; a motor that rests past its end
    stop comes back up when the motors are switched on (also with the soft start off), with up to PWMrev's drive while
    it lags; the board keeps the motion sensor while Use the sensor is ticked, and reports its link (`[rdj]`).

  See the [CHANGELOG](CHANGELOG.md).
- New commands are documented in [docs/PROTOCOL.md](docs/PROTOCOL.md). They go over WiFi; USB stays plain SMC3
  unless the page asks for USB page mode (`[kU1]`, 1.9.5).

## Credits and licence

- **SMC3** by RufusDufus and the [xsimulator.net](https://www.xsimulator.net) community. The firmware here is a
  port and extension of SMC3 v0.70. Thank you.
- The **Michroma** font, SIL Open Font License 1.1 ([licence](docs/licenses/Michroma-OFL-1.1.txt)), is embedded
  in the page for the SMC3 R4 wordmark.
- **three.js** r160 (MIT, [licence](docs/licenses/three.js-LICENSE.txt)) is embedded in the page server for the 3D
  view.
- The seat logo, rig picture and motor icon come from the dashboard mock-up this project started from.
- Built with help from Claude (Anthropic).

Code in this repository: [MIT](LICENSE). The original SMC3 was shared freely without a stated licence; see the
note in [LICENSE](LICENSE).

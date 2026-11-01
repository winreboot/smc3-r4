# Lessons learned

Notes from getting the USB cable link and auto-tune to work on a real rig: page 2.8 to 2.9, firmware 1.9.5 to 2.0.
What broke, why, how it was fixed, and what to do differently next time. The changes themselves are in the
[CHANGELOG](../CHANGELOG.md).

## What was fixed

| What the rig showed | Cause | Fixed in |
|---|---|---|
| The page's commands over the cable arrived damaged; "456 bytes of unreadable data"; the link dropped out | The main chip's serial port has no receive FIFO; the 4 kHz motor interrupt kept the receive interrupt waiting longer than one byte takes at 500000 baud | firmware 1.9.6 |
| The Windows bridge got nothing back ("board -> page 0 kB") | PowerShell's asynchronous serial read never completed on a real PC | page 2.8.2 |
| Nothing ever came back over USB, even after a replug | The board's USB chip ran firmware older than 0.6.0 | USB chip update (standalone updater) |
| The cable went silent after some minutes of streaming; only a replug helped | The UNO R4 WiFi's USB chip stalls its output (a known board problem); the PC's commands still arrive | firmware 1.9.7: the page has the board restart the chip |
| After that restart the board stayed off WiFi, the LED matrix crawled, nothing answered until RESET | The main chip kept its old link to the restarted chip and never redid the start-up handshake, so every WiFi attempt blocked for 15 s and failed | firmware 1.9.8 |
| The board got stuck again after a chip restart, even with 1.9.8 | The chip sometimes doesn't talk to the main chip again after it restarts | firmware 2.0: the USB connector belongs to the main chip, no chip restarts |
| Auto-tune's Kps far too low (52 where it had been 252) on clean moves | A crackling pot read as a restless hold; the limits came from one short noise reading | page 2.9 |
| (found in the release audit) A 2.0 board would never have been offered 2.0.1, and the finding for 1.9.7 boards never showed | The betas' version key added the beta mark after dropping trailing zeros; one check compared it with a hand-written tuple | page 2.9 |

## The UNO R4 WiFi's USB port

**The connector isn't the main chip's.** On the UNO R4 WiFi the USB connector goes to the WiFi chip (an ESP32-S3),
which relays the bytes to the main chip (the RA4M1) over a serial link at the host's baud rate. Every problem on the
cable lived in that relay or in the link behind it: the stall, the lost bytes, the boot garbage, the chip restarts.

**Look for a way around a flaky part before building on it.** Three rounds of recovery logic (the chip restart in
1.9.7, its fix in 1.9.8, the page's stall detection in 2.9b2 and 2.9b3) became unnecessary, on the default setting,
with one switch. The board has a USB multiplexer that the main chip controls (the Arduino core flips it for its
keyboard and mouse libraries). Firmware 2.0 flips it at boot and runs the core's own USB serial port: full USB speed,
no baud rate, nothing in between. It should have been the first thing tried once the stall turned out to be the
chip's.

**Restarting a chip you don't control is fragile.** After its `+RESET` the WiFi chip boots for about 5 s and expects
the start-up handshake (`+SOFTRESETWIFI`) again. The WiFiS3 library kept its "already begun" state and the serial
link open, so it never redid the handshake: every `WiFi.begin()` blocked for its 15 s timeout and failed, and the
loop crawled (the matrix scrolled at a tenth of its speed). 1.9.8 closes the link (`modem.end()`), clears the flag,
waits for the chip and redoes the handshake first. Even then the chip sometimes didn't come back. A recovery that
depends on a part behaving after a restart is not a recovery.

**No receive FIFO means every interrupt is a deadline.** The RA4M1's serial ports hold one byte while the next comes
in: 20 µs at 500000 baud. The 4 kHz PID interrupt (three analog reads, up to about 100 µs) ran above the serial
receive interrupt, so bytes in a burst were lost. On a chip like this, any interrupt that can run longer than one
byte time must sit below the UART's (1.9.6: receive interrupts at priority 4).

**A USB write must never wait for the PC.** The core's `SerialUSB.write()` spins until its buffer has room - for
ever when nothing reads the port (SimHub closed, a program holding the port without reading). 2.0 writes only what
fits, waits at most 10 ms, and doesn't wait again until room has been seen.

**Moving the connector moves the COM port, and uploads.** The main chip's port is a new COM port on the PC, and the
Arduino IDE uploads through the WiFi chip. So the user needs a way to hand the connector back without the page:
2.0 keeps a mark in the battery-backed registers (they survive a reset but not a power-up), and two RESET presses
within 3 s give the connector to the WiFi chip until the next reset. The page's "WiFi chip until the next reset" does
the same.

**Check the USB chip's own firmware early.** A board whose USB chip ran firmware older than 0.6.0 passed nothing
back at all. The IDE's Firmware Updater needs those answers itself, so it can't fix that; Arduino's standalone
updater can.

## Never surprise a running rig

**A restart during a game is a hazard, so the firmware refuses it.** The rule ("the board must never restart by
itself while SimHub drives") is enforced on the board, where the motors are: the chip restart (`[kE1]`) and the
board restart (`[kZ R]`) are refused while SimHub's last position command is less than 10 s old, and a firmware update
while anything came over USB in the last 3 s. The page checks too, but the page can be any version on any computer;
the firmware is the last word.

**Keep the list of reset paths short and known.** The firmware has one software reset (`[kZ R]`, from the page only)
and no watchdog. When the board does restart, the diagnostics frame carries the reset-status registers and the
page's events say what kind of start it was: a power-up or supply dip, the reset button, or a software reset (an
upload, or the page's Restart button).

## Windows, browsers and serial ports

- **PowerShell's serial port:** asynchronous reads never completed on a real PC, and a click in the console window
  (QuickEdit) froze the script. Poll `BytesToRead` every 10 ms instead, and switch QuickEdit off while running.
- **The first answer after opening a port can be lost** while Windows sets the port up. Ask several times (the
  bridges ask for the firmware number up to 6 times).
- **Web Serial works only on a secure page:** https, or `localhost`. A page served from a NAS at `http://<address>`
  isn't one. The one-time fixes are a browser flag for that address, or a port proxy so the page is reached as
  `localhost`. Check a browser feature's secure-context rule before planning it for a page on the local network.
- **One program per COM port.** SimHub has to be closed for the cable link; the switch at the top of the page hands
  the port back when it goes to WiFi.

## Auto-tune on real pots

- **One short noise reading is not a baseline.** The pot's noise at rest, measured for 3 s, was 1.3 on one run and
  2.7 an hour before, and the limits taken from it moved with it. Take the median of several readings made during
  the run itself (2.9: the first three, gentlest Kps of the sweep).
- **Use a window and a statistic that a burst can't fill.** A spread over the last 0.3 s was dominated by one crackle
  burst. Over 0.9 s, with slow creep removed, as a median spread (1.4826 × the median absolute deviation), a burst
  counts little and a motor that really hunts counts in full.
- **Ask whether more tuning can change what you see.** A crackling pot is noise the PID can't fix; a hunting motor is
  something it can. The test is cheap: if the hold is as restless at a gentle Kp, it's the pot.
- **A correction that doesn't improve the measurement isn't one.** The small-step check cut Kp 103 -> 82 -> 65 -> 52
  for a restless hold that didn't calm down (2.22, 2.07, 2.41). 2.9 goes back to the Kp before when a cut doesn't make
  the hold at least 15% calmer.
- **Never loosen a safety limit from a re-test.** 2.9b5 could raise the buzz limit when the lower Kp happened to buzz
  during a burst too. That limit is fixed now.
- **Reproduce the field result in the simulator before fixing it.** `--pot-crackle`, `--fric` and `--stick`
  reproduced the early stops; with the fix the same simulations reached Kp 108-193 where the betas had stopped at
  40-68.

## Version numbers

- **Decide how betas sort before the first one ships:** 2.9b1 < 2.9b7 < 2.9 < 2.9.1.
- **Pad before adding a mark.** The betas' key dropped trailing zeros and then added the beta/release mark, so
  2.0 became (2, 1) and sorted above 2.0.1 (2, 0, 1, 1). A 2.0 board would never have been offered 2.0.1. 2.9 pads
  every version to four numbers first.
- **Compare against a parsed version, not a hand-written tuple.** `vkey(v) == (1, 9, 7)` stopped matching the moment
  the key grew a release mark, and the finding for 1.9.7 boards silently never showed. `vkey(v) == vkey("1.9.7")`
  can't drift.
- **Every place that reads a version must accept the new form:** the update check, the header, the firmware check of
  an uploaded `.bin` (its version pattern dropped the beta mark until 2.9b7).
- **Test the ordering as a list, in each language that compares versions** (here Python and the page's JavaScript).

## Testing and releasing

- **A simulator flag for every failure seen on the rig:** `--usb-stall`, `--usb-stall-now`, `--usb-lose-ku`,
  `--fw197-stuck`, `--pot-crackle`, `--fric`, `--stick`, `--supply-sag`, `--usb-unplug`. Most became regression
  tests; the rest drive the auto-tune comparisons.
- **Run the firmware on the PC.** The sketch compiles against a stub of the board's core (serial ports, the USB port,
  the WiFi chip's link, the battery-backed registers, the reset) and is driven byte by byte; a separate check catches
  what GCC 7, the Arduino IDE's compiler, rejects (1.9.2 didn't compile there).
- **Let the board report what happened.** The diagnostics frame (version 6) carries the board's own account of its
  last chip restart: when the chip answered again, how many WiFi tries, the last result. With the page's log
  (`docker logs`) that told what the chip did after a restart, which the simulator never could.
- **The rig has the last word.** The simulator can't reproduce the USB chip; each fix of the cable link went to the
  rig as a beta before it was called done.
- **Betas stay off GitHub.** Each beta was a package for the rig with every file name versioned (`smc3_scope_v2.9b7.py`,
  `SMC3_R4_v2.0b2.ino`), never changed once delivered. The release is the last beta with its version numbers and
  texts promoted, plus the fixes an audit of every version reference turned up.

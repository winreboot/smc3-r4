# Protocol

## Commands (USB and WiFi)

Every command is 5 bytes, `[`, three bytes, `]`, the same as SMC3 v0.70. `xx` is a 16-bit big-endian value; `xy`
are two separate bytes. The same commands work over USB (SimHub, SMC3Utils) and over the WiFi TCP connection
(smc3_scope).

| Command | Meaning |
|---|---|
| `[Axx]` `[Bxx]` `[Cxx]` | Target position of motor 1/2/3 (0–1023) |
| `[Dxx]` `[Exx]` `[Fxx]` | Kp ×100 for motor 1/2/3 |
| `[Gxx]` `[Hxx]` `[Ixx]` | Ki ×100 |
| `[Jxx]` `[Kxx]` `[Lxx]` | Kd ×100 |
| `[Mxx]` `[Nxx]` `[Oxx]` | Ks (derivative sample spacing) |
| `[Pxy]` `[Qxy]` `[Rxy]` | PWMmin x, PWMmax y (0–255) |
| `[Sxy]` `[Txy]` `[Uxy]` | Cutoff limit x and input clip y (the max side is 1023 − value) |
| `[Vxy]` `[Wxy]` `[Xxy]` | Dead zone x and PWM reverse duty y |
| `[Zx?]` | PID loop divider (Fpid/x) |
| `[~xy]` | PWM kHz for motors 1+2 (x) and motor 3 (y) |
| `[rdx]` | Read a value: `A`–`C` position, `a`–`c` status + PWM, `D`–`X` parameters, `Y` status, `Z` loop count, `~` PWM frequencies |
| `[mo0]`–`[mo3]` | Monitor mode (SMC3Utils) |
| `[ena]` `[en1]`–`[en3]` | Enable motors |
| `[sav]` | Save to EEPROM |
| `[ver]` | Version (answers 70) |

### Added by SMC3_R4

| Command | Since | Meaning |
|---|---|---|
| `[rd$]` | 1.3 | Worst control-interrupt time in µs since the last read |
| `[rd!]` | 1.4 | Build flags (bit0 REVERSE_MOTOR1; since 1.7 also bit1 MODE2, bit2 IMU, bit3 LED matrix, bit4 static IP; since 1.9.4 bit5 pot glitch filter) and firmware number (17 = 1.7) |
| `[dis]` `[di1]`–`[di3]` | 1.5 | Disable all motors or one (brake, stay off until enabled) |
| `[fab]` / `[rdf]` | 1.6 | Smoothing shift a (0–8, time constant 2^a / 4 ms at Fpid/1) and max speed b × 100 counts/s (0 = off) / read them |
| `[rdd]` | 1.7 | Diagnostics frame (below). WiFi only. Version 2 since 1.8. |
| `[lvf]` / `[rdl]` | 1.8 | LED matrix view v (0 motor bars, 1 seat horizon, 2 quiet) and flags f (bit0 idle trace, bit1 address every minute) / read them. Saved at once. |
| `[ift]` / `[rdi]` | 1.8 | Motion sensor flags f (bit0 on, bit1 swap roll/pitch, bit2 invert roll, bit3 invert pitch) and tilt limit t degrees (0 = off), saved at once / read: state (bits 0-3 as set, bit4 angles arriving, bit5 sensor found, bit6 setting up) and tilt limit. |
| `[iL?]` `[iC?]` | 1.8 | Level now (the current angles count as 0°/0°) / clear the level. |
| `[iSr]` | 1.8 | Set the sensor up through D1: angles only, rate code r (0 = 100 Hz), 115200 baud, saved in the sensor. 1.9: r = WitMotion rate code in bits 0-3 (0x09 = 100 Hz), bit6 set = angles + gyro + acceleration (0x49). |
| `[iR?]` | 1.8 | Look for the sensor again (try the baud rates for 30 s). |
| `[ift]` | 1.9 | As in 1.8, but t = 255 leaves the tilt limits alone (t ≤ 60 sets all four directions to t). `[rdi]` answers the tightest of the four, whole degrees; state bit7 = gyro + acceleration arriving. |
| `[whl]` | 1.9 | Value register = h × 256 + l, signed, for the next `[k f =]`. |
| `[kf=]` | 1.9 | Seat set-up field f = value register. f = 16 × motor (0-2) + 0 level trim (counts, −300…300), 1 travel low, 2 travel high, 3 end stop low, 4 end stop high (pot counts 0-1023); f = 64 + d: tilt limit × 10 for d = 0 roll + (right side down), 1 roll −, 2 pitch + (front up), 3 pitch − (0 = none); f = 80 / 81: the sensor's level roll / pitch (raw × 100, saved at once); f = 82 (1.9.1): soft-start speed in counts/s, 0 = off (saved at once in EEPROM 105-107). |
| `[kSV]` `[kRL]` `[kCL]` | 1.9 | Save the seat set-up (EEPROM 66-104; `[sav]` saves it too) / reload the saved one / clear it (no trim, no travel limits or end stops; tilt limits stay) until saved or restarted. |
| `[kHn]` | 1.9 | The page holds the seat for n × 100 ms: position commands from USB (SimHub) are counted but ignored meanwhile. n = 0 gives the seat back at once. Seat setup refreshes it every second, and so does page 2.4 while you move the motors from the page (a motor card's knob, Return to center). |
| `[xe1]` `[xe0]` | 1.9 | WiFi only: send an "SM3X" frame (the sensor's acceleration and gyro) after every telemetry burst / stop. Off for every new connection. |
| `[uP=]` `[uT=]` `[uI=]` `[uJ=]` | 1.9.2 | WiFi only. Firmware update: the value register is the page's web port / the file's number / the page's address bytes 1-2 / 3-4 (0 = the address the page connects from). The board fetches `http://<address>:<port>/ota/<number as 4 hex digits>.ota`. |
| `[uGO]` `[uOK]` | 1.9.2 | WiFi only. Start the firmware update (below) / clear a finished or failed one. |
| `[rdu]` | 1.9.2 | WiFi only. One "SM3U" frame (below): the update's state and this firmware's version text. |

### Seat set-up (firmware 1.9)

* **Level trim:** every position command (`[A]` `[B]` `[C]`, from USB or WiFi) gets the motor's trim added before
  anything else, so SimHub's centre (512) is the level seat. At power-up the target is 512 + trim, so the seat goes
  level when the motors are enabled. The trim moves where the target lands, not the SMC3 scale: SMC3Utils, the
  cutoff and the clip still work in pot counts.
* **Travel limits** (per motor, low and high separately): position commands are clipped to them, together with the
  SMC3 Clip (the tighter wins). PWMrev still works at the SMC3 Clip only.
* **End stops** (per motor, low and high): the motor switches off (reason "safety cutoff", E1-E3 on the matrix) if
  its pot gets past one - together with the SMC3 Max limit, the tighter wins.
* **Tilt limits** (per direction): every motor stops (E4) if the seat tilts further.

A value that makes no sense (a window narrower than 8 counts) falls back to SMC3's own. The telemetry's target is
the target in force (after trim and limits), in pot counts.

Over WiFi, position commands count as "host alive" (since 1.4), so tests from the page run at full power. Reading
telemetry alone doesn't.

## Telemetry (WiFi, TCP port 4210)

The board pushes frames to the connected client. Everything is little-endian.

```
'S' 'M' '3' 'T'  uint8 version (1)  uint8 sampleCount   then sampleCount × 38-byte samples:
  uint32 millis
  3 × { int16 target (0-1023), int16 goal (Q4 = 1/16 count), int16 feedback (Q4), int16 pid (Q4, signed, before PWMmin) }
  uint8 status (Fpid divider × 16 + bit per disabled motor)   uint8 powerScale (7 normal, 9 low power)
  int16 roll×100  int16 pitch×100  int16 yaw×100   uint16 worst ISR µs
```

Roll and pitch are the seat's angles from the motion sensor, levelled and turned the way the sensor is mounted
(firmware 1.8). -32768 means no sensor data; firmware 1.7 and older send 0.

After `[xe1]` (firmware 1.9) every burst is followed, in the same write, by:

```
'S' 'M' '3' 'X'  uint8 version (1)  uint8 length (18)
  int16 acceleration x, y, z (raw: / 32768 × 16 g)   int16 gyro x, y, z (raw: / 32768 × 2000 °/s)
  int16 temperature ×100   uint16 biggest |acceleration| − 1 g since the last burst (milli-g)
  uint8 flags (bit0 acceleration fresh, bit1 gyro fresh)   uint8 acceleration packets since the last burst
```

Samples are taken every 10 ms and sent in bursts of five every 50 ms. Replies to `[rd?]` come back on the same
connection as ordinary 5-byte SMC3 replies.

Firmware 1.9.2 sends, after `[rdu]`, whenever the update's state changes and four times a second while it runs:

```
'S' 'M' '3' 'U'  uint8 version (1)  uint8 length (23)
  uint8 state (0 idle, 1 stopping the motors, 2 starting, 3 downloading, 4 checking, 5 installing, 6 failed)
  int16 code (0, or why it stopped: -100 SimHub is sending over USB, -101 WiFi module firmware older than 0.4.0,
              -102 no address for the file, -103 the download stopped, -104 programming failed;
              otherwise the WiFi module's own code: -6 can't connect, -10 CRC, -11 wrong board, -12 download broke off ...)
  uint32 bytes downloaded   uint32 file size
  uint8 flags (bit0 the download reports progress, bit1 WiFi details remembered)
  uint8 WiFi module firmware major, minor, patch   char[8] this firmware's version ("1.9.2", zero-padded)
```

## Diagnostics frame (`[rdd]`, firmware 1.7)

```
'S' 'M' '3' 'D'  uint8 version (1)  uint8 payloadLength   payload:
  uint32 millis
  uint8 RSTSR0, RSTSR1, RSTSR2        reset-status registers as found at boot
  uint8 flags                          bit0: cold/warm-start flag usable; bits1-2: SMC3Utils monitor mode
  uint32 USB commands ok, USB position commands, USB bytes thrown away, WiFi commands ok
  uint16 ms since the last USB command, ms since the last USB position command   (65535 = longer/never)
  uint32 longest loop() pass in µs since the last [rdd]
  int8   WiFi RSSI dBm
  uint8  build flags, uint8 firmware number
  3 × { uint8 why off (0 on, 1 safety cutoff, 2 command, 3 not enabled since boot),
        uint16 cutoff trips, int16 pot at the last trip (-1 none),
        uint16 pot glitches (> 16 counts in one 1/4 ms cycle),
        uint16 biggest one-cycle pot change (Q4) since the last [rdd],
        uint16 lowest and highest pot reading (Q4) since the last [rdd] }
  uint8 n, then EEPROM bytes 0..n-1 (SMC3 layout, + 53-55 smoothing, + 56-65 sensor/matrix in 1.8; n = 72)
  version 2 (1.8) continues:
  uint8 sensor state ([rdi] byte 1)   uint8 tilt limit °   uint8 baud index   uint32 baud
  uint16 angle packets in the last second   uint32 good packets   uint16 bad checksums
  uint16 ms since the last angle packet (65535 = never/long)
  int16 raw roll, raw pitch, yaw (×100)   int16 level roll, level pitch (×100)   uint16 tilt-limit trips
  uint8 matrix view   uint8 matrix flags   uint8 error on the matrix (0-9, 255 = none)
  version 3 (1.9) continues:
  3 × { int16 level trim, travel low, travel high, end stop low, end stop high }
  int16 tilt limits ×10 (roll +, roll −, pitch +, pitch −)
  uint8 flags: bit0 the seat set-up running is the saved one, bit1 one is saved, bit2 "SM3X" frames on, bit3 the page holds the seat
  uint32 SimHub position commands ignored while held   uint16 acceleration packets, gyro packets in the last second
  int16 raw acceleration x, y, z   int16 raw gyro x, y, z   int16 temperature ×100
  version 4 (1.9.1) continues:
  uint16 soft-start speed (counts/s, 0 = off)   uint8 motors still creeping to their position (bit n = motor n+1)
  version 5 (1.9.2) continues:
  uint8 WiFi module firmware major, minor, patch (0.0.0 = unknown)
  uint8 WiFi flags: bit0 a WiFi name/password is remembered, bit1 joined with the remembered ones, bit2 this build has
        the placeholders   uint8 update state (as in "SM3U")   int16 update code
```

Why off: 4 = tilt limit (1.8), 5 = switched off for a firmware update (1.9.2).

### EEPROM 56-65 (firmware 1.8)

| Address | Content |
|---|---|
| 56 | 0xA6 = this block is valid |
| 57, 58 | Matrix view, matrix flags |
| 59, 60 | Sensor flags, tilt limit (degrees) |
| 61-62, 63-64 | Level offsets for roll and pitch, int16 ×100, big-endian |
| 65 | Baud-rate index that last worked (9600, 115200, 230400, 57600, 38400, 19200, 4800) |

They are written when the setting changes, not by `[sav]`, so the SMC3 tuning stays untouched.

### EEPROM 66-104 (firmware 1.9): seat set-up

| Address | Content |
|---|---|
| 66 | 0xB9 = this block is valid |
| 67 + 10 × motor | int16 level trim, travel low, travel high, end stop low, end stop high (big-endian) |
| 97-104 | int16 tilt limits ×10: roll +, roll −, pitch +, pitch − |

Written by `[kSV]` and `[sav]`. Without the block, the tilt limits come from 1.8's single limit (byte 60).

### EEPROM 105-107 (firmware 1.9.1): soft start

| Address | Content |
|---|---|
| 105 | 0xB5 = valid (otherwise the default, 60 counts/s) |
| 106-107 | int16 soft-start speed in counts/s, big-endian (0 = off) |

Written as soon as `[k 82 '=']` changes it.

### EEPROM 112-210 (firmware 1.9.2): WiFi memory

| Address | Content |
|---|---|
| 112 | 0xC7 = valid |
| 113 | Length of the network name (1-32) |
| 114-145 | Network name |
| 146 | Length of the password (0-64) |
| 147-210 | Password |

Written after the board has joined the network with the name and password compiled into the sketch, if they differ
from what is stored. A build that has the placeholders (`YourNetworkName`), or whose details don't work, joins with
the stored ones. The diagnostics frame copies EEPROM 0-71 only, so the password never leaves the board.

### Firmware update (firmware 1.9.2)

1. The page converts the `.bin` exported by the Arduino IDE into the UNO R4 WiFi's `.ota` format: uint32 length,
   uint32 CRC-32 (both little-endian, over the rest), uint32 magic 0x23411002, 8 version bytes (`00 … 00 40` =
   compressed), then the `.bin` compressed with LZSS (Okumura, EI 11, EJ 4) - as Arduino's `lzss.py` + `bin2ota.py`.
   It serves it with a Content-Length, in one write (the WiFi module reads the 20-byte header from the first piece).
2. `[uP=]`, `[uT=]` (and optionally `[uI=]` `[uJ=]`), then `[uGO]`. The board refuses (state 6) when the page gave
   no port, when anything came over USB in the last 3 s, or when its WiFi module runs firmware older than 0.4.0.
3. The board switches every motor off (why off: 5), waits 300 ms, then asks its WiFi module (`OTAUpdate`):
   `begin("/update.bin")`, `startDownload(url, "/update.bin")` and `downloadProgress()` every 200 ms (WiFi module
   0.5.0+, boards package 1.3.0+), or the blocking `download(url, "/update.bin")` otherwise. No progress for 20 s =
   -103.
4. `verify()` checks the magic number and the CRC. If USB is still quiet, state 5 for 400 ms, then
   `update("/update.bin")`: the WiFi module resets the board into its bootloader, programs it (as with a USB upload)
   and starts the new firmware. That call only comes back if programming couldn't start (-104).
5. The new firmware starts with the soft start. The page waits for the connection to come back and reads the version
   from `[rdu]`.

`[ena]` is ignored from state 1 to 5. While it runs, the matrix shows `UP` and a progress bar (blinking while the
WiFi module checks and installs); after a failure it scrolls `ERROR E7 UPDATE FAILED` until `[uOK]` or the next
update.

### Soft start (firmware 1.9.1)

When a motor is switched on - at power-up, and by `[ena]` / `[enN]` after it was off - its smoothed goal starts
exactly at the pot reading and moves to the target at no more than the soft-start speed, easing in over 0.6 s and out
over the last ~15 counts. Position commands still set the target meanwhile; the goal follows them at that speed until
it has caught up (within 2 counts), then everything is normal. It gives up after 40 s. While a motor creeps:
PWMrev (the original's full-duty drive back from past the Clip window) is not used; a motor that starts past one of
its 1.9 end stops may come back (it is switched off only if it goes 16 counts further out than it has been, or past
the SMC3 cutoff, until it is 16 counts inside); the tilt limit waits until no motor creeps any more, plus 1.5 s; the
board keeps full power without position commands.

RSTSR2 bit0 (CWSF) reads 0 after a real power-on, or after a supply dip deep enough to restart the chip. The
firmware sets it to 1 at boot, so a later reset by the button, an upload or the WiFi/USB chip reads 1. The first
start after flashing 1.7 always reads as a cold start.

# Firmware

## SMC3_R4: Arduino UNO R4 WiFi

`SMC3_R4/SMC3_R4.ino` is a port and extension of SMC3 v0.70 (RufusDufus, xsimulator.net). It keeps the same
protocol, EEPROM layout and pins, so SimHub, SimTools, FlyPT Mover and SMC3Utils see an ordinary SMC3. `[ver]`
still answers 70.

What the R4 hardware adds:

- **4 kHz PID loop in a timer interrupt**, so it keeps running while WiFi or USB is busy.
- **14-bit pot readings**. Positions carry 4 extra fractional bits on the usual 0–1023 scale, so every SMC3Utils
  number keeps its meaning.
- **25 kHz PWM at full timer resolution** (about 1900 steps instead of 256). It's quiet and gives fine control
  near zero.
- **Target smoothing and max speed**: a low-pass filter and an optional speed limit on SimHub's steps. Both are
  settings since 1.6.
- **A fixed PID**: a real integrator with anti-windup, and a filtered derivative taken on the measured position.
- **LED matrix**: the IP address at power-up, then live motor bars (or a seat horizon with the motion sensor), an
  oscilloscope trace while idle, and problems scrolled in words: `ERROR E3 MOTOR 3 CUTOFF`.
- **Motion sensor (optional)**: a WitMotion WT901C on Serial1 gives the seat's real angles, tilt limits for each
  direction, gyro and acceleration for the page's 3D view, and the seat's own movement for auto-tune and
  diagnostics. It is switched on and set up from the page, with no recompiling.
- **Seat set-up** (1.9, made by the page's Seat setup): a level trim per motor, so SimHub's centre is the level seat
  (and the seat goes level at power-up), travel limits and end stops for each motor and each direction, and the
  tilt limits. Saved on the board, so they work whatever drives it.
- **WiFi telemetry** at 100 samples a second to `smc3_scope`, plus live tuning and diagnostics. See
  [docs/PROTOCOL.md](../docs/PROTOCOL.md).

### Flashing

1. In the Arduino IDE, use Boards Manager to install **Arduino UNO R4 Boards**, then select **Arduino UNO R4 WiFi**.
2. Open `SMC3_R4/SMC3_R4.ino` and set **`WIFI_SSID`** and **`WIFI_PASS`** (2.4 GHz only).
3. Upload. The address scrolls across the matrix at power-up.

### Options (top of the sketch)

| Option | Default | What it does |
|---|---|---|
| `MODE2` / `MODE1` | `MODE2` | `MODE2` = IBT-2 / BTS7960 bridges. `MODE1` = a bridge with PWM and two direction inputs. |
| `REVERSE_MOTOR1` | off | Mirrors motor 1, as in the original. |
| `DEFAULT_TIMER1_KHZ`, `DEFAULT_TIMER2_KHZ` | 25 | PWM frequency for motors 1+2 and for motor 3. SMC3Utils and the page can change it. |
| `TARGET_SMOOTH_SHIFT` | 6 (16 ms) | Default target smoothing: 4 = 4 ms … 8 = 64 ms, 0 = off. It's a setting since 1.6. |
| `TARGET_MAX_RATE` | 0 (off) | Default max speed in counts per second. It's a setting since 1.6. |
| `D_FILTER_SHIFT` | 3 | Low-pass filter on the derivative term. |
| `LED_MATRIX`, `LED_FPS` | 1, 20 | Use the matrix; refresh rate. |
| `LED_DEFAULT_VIEW`, `LED_DEFAULT_FLAGS` | 0, 3 | What a board that was never set shows: motor bars, idle trace on, address every minute. The page's Settings change it. |
| `LED_IDLE_AFTER_MS`, `LED_HORIZON_DEG` | 20000, 10 | When the idle trace starts; how many degrees move the seat horizon to the edge. |
| `IMU_PORT` | 1 | Listen for a WT901C on Serial1 (D0 RX, D1 TX). 0 leaves D0/D1 free. |
| `ENABLE_WIFI` | 1 | 0 = no networking: a plain, faster SMC3. |
| `WIFI_SSID`, `WIFI_PASS`, `WIFI_HOSTNAME` | placeholders, `smc3-r4` | Your network. |
| `TELEMETRY_PORT` | 4210 | TCP port the page connects to. |
| `WIFI_STATIC_IP` + `WIFI_IP`… | 0 | Use a fixed address instead of DHCP. |

### LED matrix

One thing at a time, readable from across the room. The previews are rendered from the firmware's own drawing
code.

| | |
|---|---|
| <img src="../docs/img/matrix-boot.gif" width="250" alt="Boot"> | **Power-up:** "R4" while WiFi connects, then the IP address scrolls. It repeats every minute while the page isn't connected (Settings can turn that off). |
| <img src="../docs/img/matrix-bars.gif" width="250" alt="Motor bars"> | **Motor bars** (default): each motor's position, live. The two columns of a bar differ by half a pixel between steps. A dot **blinks where a motor should be** when it falls well behind - here motor 2 gets stuck for a moment. |
| <img src="../docs/img/matrix-horizon.gif" width="250" alt="Seat horizon"> | **Seat horizon** (motion sensor): the edge moves up with the seat's front and tilts with its roll. Without sensor data it shows the motor bars. |
| <img src="../docs/img/matrix-idle.gif" width="250" alt="Idle trace"> | **Idle:** after 20 s without position commands, the motor-bars view becomes a slow oscilloscope trace. |
| <img src="../docs/img/matrix-motor-off.gif" width="250" alt="Motor off"> | **A motor switched off on purpose** (Disable Motors, or not in use) is a dash at the bottom - not an error. |
| <img src="../docs/img/matrix-error-cutoff.gif" width="250" alt="Cutoff error"> | **Problems scroll in words**, again and again while they last, with a moment of the view in between. A motor stopped by its cutoff blinks full height. |
| <img src="../docs/img/matrix-error-tilt.gif" width="250" alt="Tilt limit"> | **Tilt limit** (motion sensor): the seat pitched past the limit, every motor stopped, `ERROR E4 TILT LIMIT`. |
| <img src="../docs/img/matrix-error-wifi.gif" width="250" alt="No WiFi"> | **No WiFi**: `ERROR E9 NO WIFI` every few seconds; the motor bars carry on in between, because SimHub over USB still works. |

| Code | Scrolls | Meaning |
|---|---|---|
| E1, E2, E3 | `ERROR E1 MOTOR 1 CUTOFF` | That motor's pot went past its safety cutoff or one of its end stops (1.9); the motor is off until Enable Motors. |
| E4 | `ERROR E4 TILT LIMIT` | The motion sensor saw the seat tilt past a tilt limit; every motor stopped. |
| E5 | `ERROR E5 NO SENSOR` | The sensor is switched on, but nothing arrives from it. |
| E9 | `ERROR E9 NO WIFI` | Can't join the network, or WiFi dropped (retries every 30 s while SimHub is idle). |
| E8 | `ERROR E8 NO IP ADDRESS` | Joined, but the router hasn't given an address yet. |
| E0 | `ERROR E0 WIFI MODULE` | The WiFi chip doesn't answer. |

The page's Help section says what to do for each. E0, E8 and E9 only affect the page: SimHub over USB keeps working.

### Motion sensor (WT901C)

<img src="../docs/img/sensor-wiring.png" width="900" alt="Sensor wiring: WT901C-232, DB9 plug with screw terminals, MAX3232 module, UNO R4 WiFi">

The **-232** version of the WT901C talks at RS-232 levels (its TX idles at about −5 to −9 V): it goes through a
MAX3232 module, never straight to D0/D1. Sensor TX (yellow) → the module's RS-232 RX, its TTL TX → **D0**; **D1** →
the module's TTL RX, its RS-232 TX → sensor RX (green); sensor and module from **5V** / **GND**. With a DB9 module,
a DB9 plug with screw terminals takes the sensor's wires: yellow to pin 3, green to pin 2, black to pin 5. D0/D1 are
not the USB port on the R4, so SimHub and uploading are unaffected.

The board tries the usual baud rates until the sensor answers, levels and turns the angles the way the page's
settings say, and sends them with every telemetry sample. With **Use the sensor** ticked in the page, a silent
sensor scrolls `ERROR E5 NO SENSOR`, and the **tilt limits** work: three readings in a row past one stop every
motor. The check pauses for 1.5 s after Enable Motors, so the seat can come back first.

**Set up the sensor** in the page sends the WitMotion set-up through D1: unlock, output angles + gyro +
acceleration (1.9; angles only with 1.8), 100 Hz, 115200 baud, save. It leaves the factory at 10 Hz, 9600 baud. If
SimHub also reads the sensor, wire only TX and GND to the MAX3232 and set it up in WitMotion's PC software instead.

The sensor and matrix settings live on the board (EEPROM 56-65) and are saved as soon as they change, separately
from the SMC3 tuning. The seat set-up (EEPROM 66-104) is saved by Seat setup, and by `[sav]`.

### Seat set-up (1.9)

Made by the page's **Seat setup** and stored on the board:

- **Level trim** per motor: added to every position command, so SimHub's 512 is the level seat. At power-up the
  seat goes there when the motors are enabled.
- **Travel limits** per motor, low and high separately: no position command goes past them (with the SMC3 Clip,
  the tighter wins).
- **End stops** per motor: the motor switches off (E1-E3) if its pot gets past one anyway (with the SMC3 Max limit,
  the tighter wins).
- **Tilt limits** per direction (motion sensor): roll right, roll left, front up, front down.

While Seat setup moves the seat, the board ignores SimHub's position commands (`[kH n]`, refreshed by the page; it
lapses by itself after 3 s if the page goes away). The commands are in [docs/PROTOCOL.md](../docs/PROTOCOL.md).

### Versions

| Version | Change |
|---|---|
| 1.9 | Seat set-up: level trim, travel limits and end stops per motor and direction, tilt limits per direction (`[w..]`, `[k..]`, EEPROM 66-104). The page can hold SimHub off while it moves the seat (`[kH n]`). The sensor's gyro and acceleration are read and sent to the page (`[xe1]`, "SM3X"); `[iS..]` can ask for them. `[rdd]` frame version 3. |
| 1.8.1 | Compiles in the Arduino IDE (1.8 didn't). Same features. |
| 1.8 | LED matrix redone (motor bars, seat horizon, idle trace, problems in words). WT901C sensor at run time: `[i..]` on/off, mounting, level, tilt limit, set to 100 Hz; `[l..]` matrix view. `[rdd]` frame version 2 adds the sensor and matrix state. |
| 1.7 | `[rdd]` diagnostics frame (reset cause, USB counters, glitch counts, why motors are off, EEPROM copy). `[rd!]` reports the build options. |
| 1.6 | Smoothing and max speed are settings (`[f a b]`, `[rdf]`) saved to EEPROM. |
| 1.5 | `[dis]` and `[di1]`–`[di3]` software stop. |
| 1.4 | Position commands over WiFi count as "alive", so tests from the page run at full power. `[rd!]` reports the version. |
| 1.3 | First R4 release. |

## SMC3_smooth: Arduino UNO R3

`SMC3_smooth/SMC3_smooth.ino` is the original SMC3 v0.70 for the UNO R3 with:

- target smoothing;
- a working I and D term;
- a few bug fixes.

It's for anyone staying on an R3, and doesn't talk to the page. The changes are listed at the top of the file.

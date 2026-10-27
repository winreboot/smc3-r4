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
- **LED matrix**: a tracking view, drive bars, fault codes, the IP address, and an optional IMU bubble level.
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
| `LED_MATRIX`, `LED_VIEW`, `LED_FPS` | 1, 0, 20 | Use the matrix, choose the tracking view (0) or IMU bubble (1), set the refresh rate. |
| `IMU_SERIAL1`, `IMU_BAUD` | 0, 115200 | Read a WitMotion WT901C on Serial1 (D0), through a MAX3232 for the RS-232 version. |
| `ENABLE_WIFI` | 1 | 0 = no networking: a plain, faster SMC3. |
| `WIFI_SSID`, `WIFI_PASS`, `WIFI_HOSTNAME` | placeholders, `smc3-r4` | Your network. |
| `TELEMETRY_PORT` | 4210 | TCP port the page connects to. |
| `WIFI_STATIC_IP` + `WIFI_IP`… | 0 | Use a fixed address instead of DHCP. |

### LED matrix

- **Boot:** "R4", then the IP address scrolls. It repeats every minute while no page is connected.
- **WiFi errors:**
  - **E0**: the WiFi module isn't answering.
  - **E9**: the board can't join the network (wrong name or password, or a 5 GHz-only or WPA3-only network).
  - **E8**: the board joined, but the router hasn't given it an address yet.
- **E1 / E2 / E3**: that motor is off, either because its safety cutoff tripped or because Disable Motors was
  pressed. **Enable Motors** re-arms it.
- **Tracking view:** for each motor, a row showing where it's being sent above a row showing where it is, then
  the drive bars.
- **Bottom row**, left to right:
  1. commands arriving;
  2. comms timeout (low power);
  3. motors on (three dots);
  4. loop overrun;
  5. IMU;
  6. WiFi (solid = page connected, blinking = WiFi up);
  7. heartbeat.

### Versions

| Version | Change |
|---|---|
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

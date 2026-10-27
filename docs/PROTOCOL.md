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
| `[rd!]` | 1.4 | Build flags (bit0 REVERSE_MOTOR1; since 1.7 also bit1 MODE2, bit2 IMU, bit3 LED matrix, bit4 static IP) and firmware number (17 = 1.7) |
| `[dis]` `[di1]`–`[di3]` | 1.5 | Disable all motors or one (brake, stay off until enabled) |
| `[fab]` / `[rdf]` | 1.6 | Smoothing shift a (0–8, time constant 2^a / 4 ms at Fpid/1) and max speed b × 100 counts/s (0 = off) / read them |
| `[rdd]` | 1.7 | Diagnostics frame (below). WiFi only. |

Over WiFi, position commands count as "host alive" (since 1.4), so tests from the page run at full power. Reading
telemetry alone doesn't.

## Telemetry (WiFi, TCP port 4210)

The board pushes frames to the connected client. Everything is little-endian.

```
'S' 'M' '3' 'T'  uint8 version (1)  uint8 sampleCount   then sampleCount × 38-byte samples:
  uint32 millis
  3 × { int16 target (0-1023), int16 goal (Q4 = 1/16 count), int16 feedback (Q4), int16 pid (Q4, signed, before PWMmin) }
  uint8 status (Fpid divider × 16 + bit per disabled motor)   uint8 powerScale (7 normal, 9 low power)
  int16 roll×100  int16 pitch×100  int16 yaw×100  (IMU, 0 without one)   uint16 worst ISR µs
```

Samples are taken every 10 ms and sent in bursts of five every 50 ms. Replies to `[rd?]` come back on the same
connection as ordinary 5-byte SMC3 replies.

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
  uint8 n, then EEPROM bytes 0..n-1 (SMC3 layout, + 53-55 smoothing)
```

RSTSR2 bit0 (CWSF) reads 0 after a real power-on, or after a supply dip deep enough to restart the chip. The
firmware sets it to 1 at boot, so a later reset by the button, an upload or the WiFi/USB chip reads 1. The first
start after flashing 1.7 always reads as a cold start.

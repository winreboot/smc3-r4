//****************************************************************************************************************
// SMC3_R4.ino - SMC3-compatible 3-motor PID driver for the ARDUINO UNO R4 WiFi (Renesas RA4M1)
//
// A port of SMC3 v0.70 (RufusDufus, xsimulator.net) plus the smoothing / PID fixes from SMC3_smooth.ino,
// rebuilt for the R4 hardware:
//
//   * PID loop runs in a hardware-timer interrupt at 4 kHz (FspTimer), not a busy-wait in loop().
//   * 25 kHz PWM on D9/D10/D11 through the GPT timers (PwmOut) at full timer resolution (~1920 steps
//     instead of 256).  The SMC3Utils "PWM freq" setting still works: Timer1 value -> motors 1+2,
//     Timer2 value -> motor 3.
//   * Feedback pots read at 14 bits.  Internally positions carry 4 extra fractional bits ("Q4") on top
//     of the SMC3 10-bit scale, so all SMC3Utils numbers (clip, limits, dead zone, 0-1023 targets) keep
//     their meaning, and the PID output keeps 4 fractional bits of PWM as well.
//   * Target smoothing (low-pass + optional slew limit), real integrator with anti-windup, filtered
//     derivative-on-measurement - same design as SMC3_smooth.ino.
//   * 12x8 LED matrix: scrolls the IP address, then live motor bars (or a seat horizon with the motion sensor)
//     and an oscilloscope trace while idle; problems scroll as words ("ERROR E3 MOTOR 3 CUTOFF").
//   * Optional WitMotion WT901C motion sensor on Serial1 (D0/D1, through a MAX3232 for the -232 model): seat
//     angles in the telemetry, tilt limits for each direction, acceleration and gyro for the page's 3D view.
//     Switched on and set up from smc3_scope.
//   * Seat set-up (from smc3_scope's Seat setup): a level trim so SimHub's centre is the level seat, travel limits
//     and end stops for each motor and each direction.
//   * WiFi telemetry + live tuning: the board runs a small TCP server.  A PC tool (smc3_scope.py) connects,
//     receives 100 Hz telemetry (target / goal / feedback / PID output per motor, status, IMU angles,
//     loop timing) and sends ordinary SMC3 commands back - so you can tune Kp/Ki/Kd/PWMmin... live while
//     SimHub keeps the USB port.  Every WiFiS3 call is a blocking round-trip to the ESP32 module over a
//     115200-baud UART, so the network code is rate-limited and kept to one round-trip per telemetry burst.
//
//   SERIAL PROTOCOL, COMMAND SET AND EEPROM LAYOUT ARE UNCHANGED from SMC3 v0.70.
//   SimHub (SMC3 output) and SMC3Utils v1.x talk to this board exactly as to the original.
//   (USB CDC ignores the baud rate, so SimHub's 500000 setting is fine.)
//
//   Not carried over from the UNO sketch: the A5 "motion scaler pot" options and the broken SoftwareSerial
//   second port (Serial1 is a real UART here and is used for the IMU instead).
//
// WIRING (same as the UNO):   D2/D3 M1 ENA/ENB,  D4/D5 M2 ENA/ENB,  D6/D7 M3 ENA/ENB,
//                             D9/D10/D11 M1/M2/M3 PWM,  A0/A1/A2 M1/M2/M3 feedback pots.
//                             D0 (RX) <- MAX3232 <- WT901C TX (yellow);  D1 (TX) -> MAX3232 -> WT901C RX (green,
//                             optional: only needed to set the sensor up from the page).  Sensor VCC 5 V, GND.
//
// Pots: wiper must stay within 0-5 V (the R4 is a 5 V board, so existing wiring is fine).
//****************************************************************************************************************

#define FW_VERSION     "1.9.6"         // shown in the USB boot banner; [ver] still answers 70 for SMC3Utils
#define FW_VERSION_NUM 19              // reported by [rd!] so smc3_scope can check what it is talking to (1.9.x = 19)
#define FW_CAPS        "SMC3CAPS:ota1,wifimem1,usbpage1"   // 1.9.2: what this build can do - smc3_scope looks for it in a .bin

// v1.9.6 changes: no more lost bytes on the USB port.  On the RA4M1 the USB port (Serial, SCI9) and the motion
//                sensor's port (Serial1, SCI2) have no receive FIFO: one byte waits in the receive register while
//                the next comes in - 20 us at 500000 baud.  The 4 kHz PID interrupt (priority 6, its three
//                analogRead()s take up to ~100 us) kept the core's receive interrupt (priority 12) waiting longer
//                than that, so bytes in a burst were lost: now and then one of SimHub's position frames, and often
//                enough the page's commands over the USB cable that its page mode dropped out ("456 bytes of
//                unreadable data").  Their receive and receive-error interrupts now come before the PID interrupt
//                (SERIAL_RX_IRQ_PRIORITY 4); each takes a few microseconds.  Nothing else changed.

// v1.9.5 changes: the page can talk over the USB cable instead of WiFi, for calibration with SimHub closed.  [kU1]
//                over USB switches the USB port to "page mode" for USB_PAGE_TIMEOUT_MS (5 s); the page repeats it
//                every second, and [kU0] (or 5 s without it) ends it.  While it is on, the telemetry ("SM3T"/"SM3X")
//                and the diagnostics frame go out over USB, USB position commands are the page's (the seat hold
//                doesn't drop them, and they don't count as SimHub's), and the board makes no WiFi call at all - the
//                WiFi module's round-trips are what held up the loop now and then.  The WiFi link picks up again
//                when page mode ends.  SimHub on USB is unchanged: without [kU1] nothing is different.  The
//                diagnostics flags byte gets bit 4 (USB page mode on); [rdu] answers over USB too (the version
//                text - firmware updates themselves stay WiFi only); FW_CAPS says "usbpage1".
//                Also: loop() reads the clock again after the USB commands - one parsed a millisecond after loop() took
//                "now" made "now - LastPacketMillis" wrap, so the board ran a pass at LOW POWER (and page mode would
//                have ended at once).

// v1.9.4 changes: pot glitch filter.  A pot reading that is more than POT_JUMP_COUNTS (16) away from the last good one
//                can't be the motor moving (that would be 64000 counts/s) - the wiper lost contact for a moment.  The
//                last good reading now stands in for it, for up to POT_HOLD_MS (10 ms: 40 readings at 4 kHz, fewer with
//                a PID divider) in a row, so the PID no longer kicks the motor at every glitch.  A change that lasts longer is real (a slipped pot, a broken
//                wire) and goes through - the cutoff still trips on it, at most 10 ms later.  The glitch counter
//                (diagnostics) still counts every raw glitch, so a bad pot still shows.  POT_FILTER 0 switches it off;
//                the build flags ([rd!] and the diagnostics frame, bit 5) say whether a build has it.
//                Nothing else changed: protocol, EEPROM and settings as in 1.9.3.

// v1.9.3 changes: compiles in the Arduino IDE.  1.9.2 stopped with "sorry, unimplemented: string literal in function
//                template signature": the UNO R4 boards package compiles with GCC 7, which can't have the update
//                file's name ("/update.bin") inside a template's signature; it is a parameter now.  Nothing else
//                changed.

// v1.9.2 changes: firmware update over WiFi, from smc3_scope's Settings.  The page converts the .bin the Arduino IDE
//                exports (Sketch, Export Compiled Binary) into the UNO R4 WiFi's .ota format and serves it; the board's
//                WiFi module downloads it, checks it and flashes the board - the same way it does a USB upload - and the
//                new firmware starts with the soft start.  The motors are switched off first; it is refused while
//                SimHub is sending over USB (the WiFi module flashes the board over that same link).  Needs the WiFi
//                module's firmware 0.4.0 or newer (0.5.0 or newer for a progress bar).
//                [w hi lo] + [uP=] / [uT=] set the page's web port and the file's number, [uI=] [uJ=] the page's
//                address (optional: by default the address the page connects from), [uGO] starts, [uOK] clears a
//                finished or failed update.  "SM3U" frames report how it is going.
//                The board remembers the WiFi name and password it last joined with (EEPROM 112-210): a build that
//                still has the placeholders (or a wrong password) uses the remembered ones.
//                [rdd] frame version 5 adds the WiFi module's firmware version, the WiFi memory and the last update.

// v1.9.1 changes: soft start.  At power-up, and when a motor is switched back on (Enable Motors), each motor creeps
//                from wherever it is to where it should be - SimHub's centre at power-up - at the soft-start speed
//                (default 60 counts/s, easing in and out), instead of jumping there.  Once it has caught up it runs
//                normally.  Meanwhile:
//                * a seat that rode down past an end stop while the rig was off may come back (it is switched off
//                  only if it goes 16 counts further out, or past the SMC3 cutoff - a broken pot still trips at once);
//                * the PWMrev "drive back at full reverse duty" of the original stays off, so nothing slams;
//                * the tilt limit waits until every motor has caught up (and 1.5 s more);
//                * the board stays at full power even without position commands.
//                [k 82 '='] sets the speed (counts/s, 0 = off), saved at once in EEPROM 105-107.
//                [rdd] frame version 4 adds the soft-start speed and which motors are still creeping.

// v1.9 changes:  seat set-up, made by smc3_scope's Seat setup:
//                * level trim per motor: added to every position command, so SimHub's centre (512) is the level seat
//                  (and the seat goes level at power-up);
//                * travel limits per motor, low and high separately: position commands never go past them;
//                * end stops per motor: the motor switches off (E1-E3) if its pot gets past one anyway;
//                * tilt limits for each direction - roll right/left, front up/down - instead of one for all.
//                [w hi lo] loads a value, [k f '='] stores it in field f, [kSV] saves the set-up (so does [sav]),
//                [kRL] reloads the saved one, [kCL] clears it (until saved or restarted), [kH n] lets the page hold
//                the seat for n x 100 ms: SimHub's position commands are ignored meanwhile (the page refreshes it).
//                Motion sensor: acceleration and gyro packets are read too; [iS b] with bit6 set asks the sensor for
//                angles + gyro + acceleration.  [xe1] sends them to the page after every telemetry burst ("SM3X").
//                [rdd] frame version 3 adds the seat set-up and the sensor's acceleration, gyro and temperature.
//                SimHub and SMC3Utils see no difference, apart from the level trim and the limits doing their job.

// v1.8.1 changes: compiles in the Arduino IDE.  1.8 stopped with "'GlyphDef' does not name a type": the IDE copies
//                every function's declaration to the top of the sketch, and FindGlyph() returned a GlyphDef pointer.
//                It returns an index now.  Nothing else changed.

// v1.8 changes:  LED matrix redone: no more status dots.  After the IP address it shows live motor bars (a dot
//                blinks where a lagging motor should be), a seat horizon with the motion sensor, or nothing; a
//                slow oscilloscope trace while SimHub is idle.  Problems scroll as words, e.g. "ERROR E3 MOTOR 3
//                CUTOFF" - a motor switched off on purpose is no longer shown as an error.  Codes: E0 WiFi module,
//                E1-E3 motor cutoff, E4 tilt limit, E5 no sensor, E8 no IP address, E9 no WiFi.
//                WT901C motion sensor at run time (no recompiling): the board finds the sensor's baud rate by
//                itself, [i f t] switches it on, sets the mounting and the tilt limit, [iL?] levels it, [iS?]
//                sets the sensor to 100 Hz / 115200 baud.  Tilt limit: all motors stop (E4) if the seat tilts
//                past it.  [l v f] picks the matrix view.  These settings are saved at once (EEPROM 56-65).
//                [rdd] frame version 2 adds the sensor and matrix state.  Nothing changes for SimHub/SMC3Utils.

// v1.7 changes:  diagnostics for smc3_scope's Diagnostics page.  New read code [rdd] (WiFi only) returns one
//                "SM3D" frame: cause of the last reset, SimHub/USB command counters, longest loop() pass,
//                WiFi signal, and per motor why it is off, safety-cutoff trips (and where), pot glitches seen
//                at the full 4 kHz loop rate, pot range seen - plus a copy of EEPROM 0-63 so the page can
//                tell running settings from saved ones.  [rd!] flags gain bit1 MODE2, bit2 IMU_SERIAL1 (IMU_PORT in 1.8),
//                bit3 LED_MATRIX, bit4 WIFI_STATIC_IP.  Nothing changes for SimHub or SMC3Utils.

// v1.6 changes:  target smoothing and max speed are now settings instead of compile-time constants:
//                [f a b] sets smoothing shift a (0-8) and max speed b*100 counts/s (0 = off), [rdf] reads them,
//                [sav] stores them (EEPROM 53-55; boards without them start with the defaults below).

// v1.5 changes:  new commands [dis] (disable all motors) and [di1]/[di2]/[di3] (one motor) - a software stop
//                for the "Disable Motors" button.  The motors brake and stay off until [ena] (Enable motors).

// v1.4 changes:  position commands ([A],[B],[C]) arriving over WiFi now count as "host alive", so test signals
//                and auto-tune from smc3_scope run at full power instead of the 15 s low-power fallback;
//                WiFi commands are polled every 20 ms while the page is driving a motor (100 ms otherwise);
//                new read code [rd!] -> flags (bit0 = REVERSE_MOTOR1) and firmware version.

//----------------------------------------------------------------------------------------------------------------
//  HARDWARE / PROTOCOL OPTIONS
//----------------------------------------------------------------------------------------------------------------

// MODE1: H-bridge with PWM + two direction inputs.   MODE2: IBT-2 / BTS7960 ("43A Chinese" bridge).
#define MODE2

// Uncomment to reverse the direction of Motor 1 (same meaning as the original).
//#define REVERSE_MOTOR1

// Default PWM frequencies in kHz (SMC3Utils can change these; they are stored in EEPROM).
#define DEFAULT_TIMER1_KHZ 25      // motors 1 and 2
#define DEFAULT_TIMER2_KHZ 25      // motor 3 (on the R4 it can be anything, not just 4/31)

//----------------------------------------------------------------------------------------------------------------
//  SMOOTHING / PID OPTIONS  (same meaning as SMC3_smooth.ino)
//----------------------------------------------------------------------------------------------------------------

// DEFAULTS - since 1.6 both can be changed at run time from smc3_scope ([f] command) and saved to EEPROM.
// Low-pass on the commanded position. 0 = off. Time constant ~ 2^shift PID cycles:
//   at 4 kHz (Fpid/1):  4 -> 4 ms   5 -> 8 ms   6 -> 16 ms   7 -> 32 ms   8 -> 64 ms
#define TARGET_SMOOTH_SHIFT 6

// Hard limit on how fast the commanded position may move, in SMC3 counts (0-1023) per second. 0 = off.
// 2000 = full travel in about half a second. Also gives a gentle move to centre at power-up. Steps of 100.
#define TARGET_MAX_RATE 0

// Soft start (1.9.1): after power-up and Enable Motors each motor moves to its position at this speed, in counts per
// second (0 = off: straight there, as before). smc3_scope can change it; it is saved on the board.
#define SOFT_START_RATE 60

// Extra low-pass on the derivative term (0 = off, 2-4 sensible).
#define D_FILTER_SHIFT 3

// Fixed-point scaling of the I and D terms (matches original SMC3 motor 3 / v1.x). Leave alone.
#define I_SHIFT 6
#define D_SHIFT 5

//----------------------------------------------------------------------------------------------------------------
//  LED MATRIX / IMU OPTIONS
//----------------------------------------------------------------------------------------------------------------

#define LED_MATRIX 1               // 1 = use the 12x8 matrix, 0 = off
#define LED_FPS    20              // display refresh rate from loop()
// The view (motor bars / seat horizon / quiet) and the idle trace are chosen in smc3_scope's Settings and saved
// on the board; these are the values for a board that never had them set:
#define LED_DEFAULT_VIEW  0        // 0 = motor bars, 1 = seat horizon (needs the sensor), 2 = quiet
#define LED_DEFAULT_FLAGS 3        // bit0 = oscilloscope trace while idle, bit1 = repeat the IP address every minute
#define LED_IDLE_AFTER_MS 20000    // idle = no position commands (SimHub or page) for this long
#define LED_HORIZON_DEG   10.0f    // seat horizon: degrees that move/tilt the horizon to the edge of the matrix

#define IMU_PORT 1                 // 1 = listen for a WitMotion WT901C on Serial1 (D0 RX / D1 TX), 0 = leave D0/D1 alone

//----------------------------------------------------------------------------------------------------------------
//  WIFI / TELEMETRY OPTIONS   <<< put your network name and password here >>>
//----------------------------------------------------------------------------------------------------------------

#define ENABLE_WIFI    1                   // 0 = no networking at all
#define WIFI_SSID      "YourNetworkName"   // 2.4 GHz network (the ESP32-S3 module has no 5 GHz)
#define WIFI_PASS      "YourPassword"
// 1.9.2: the board remembers the name and password it last joined with. Leave the placeholders above and it uses the
// remembered ones (handy for builds you install from the page); put new ones in to change network.
#define WIFI_HOSTNAME  "smc3-r4"           // shows up in your router's client list
#define TELEMETRY_PORT 4210                // TCP port smc3_scope.py connects to

#define WIFI_STATIC_IP 0                   // 1 = use the fixed address below instead of DHCP
#define WIFI_IP        192, 168, 1, 50
#define WIFI_GATEWAY   192, 168, 1, 1
#define WIFI_SUBNET    255, 255, 255, 0
#define WIFI_DNS       192, 168, 1, 1

#define TELEMETRY_SAMPLE_MS 10             // one telemetry sample every 10 ms (100 Hz)
#define TELEMETRY_SEND_MS   50             // sent as a burst of 5 samples every 50 ms (one TCP write)
#define WIFI_CONNECT_TIMEOUT_MS 15000      // how long setup() waits for the network before giving up
#define WIFI_DHCP_TIMEOUT_MS    10000      // after joining, how long to wait for the router to assign an address
#define WIFI_RETRY_MS  30000               // retry a failed/lost connection this often (only when SimHub is idle)

// The full IP address scrolls across the matrix after boot, and again every IP_SCROLL_REPEAT_MS while
// WiFi is up but no scope is connected (0 = boot only).  "E0" = WiFi module not responding, "E9" = could
// not join the network (wrong name/password, 5 GHz-only or WPA3-only network), "E8" = joined but the router
// has not assigned an address yet (keeps checking).  The same text is printed once on the USB serial
// monitor at boot if a terminal has the port open.
#define IP_SCROLL_REPEAT_MS 60000
#define MARQUEE_STEP_MS     110            // ms per column; ~8 s for a full address
#define ERROR_STEP_MS       90             // ms per column for "ERROR ..." text (1.8)

//----------------------------------------------------------------------------------------------------------------
//  INTERNALS - no need to touch
//----------------------------------------------------------------------------------------------------------------

#define PID_RATE_HZ      4000      // PROCESS_PERIOD_uS 250 in the original
#define PID_IRQ_PRIORITY 6         // lower number = higher priority; matrix refresh uses the default 12
#define SERIAL_RX_IRQ_PRIORITY 4   // 1.9.6: the serial ports' receive interrupts come first (no FIFO - see v1.9.6)
#define ADC_BITS         14
#define POS_SHIFT        (ADC_BITS - 10)   // 4 fractional bits on top of the SMC3 0-1023 scale ("Q4")
#define PWM_FRAC         POS_SHIFT         // PID output also carries 4 fractional bits
#define PWM_FULL_Q       (255 << PWM_FRAC) // = 4080, the "255" of the original in Q4
#define NUM_MOTORS       3
#define START_BYTE       '['
#define END_BYTE         ']'
#define COMMS_TIMEOUT_MS 15000     // same 15 s as the original before dropping to low power
#define POT_JUMP_COUNTS  16        // a pot reading that moves more than this in one PID cycle (1/4 ms) is a glitch:
                                   // no motor can do that (it would be 64000 counts/s), the wiper lost contact
#define POT_FILTER       1         // 1.9.4: hold the last good pot reading through a glitch (0 = off, as 1.9.3)
#define POT_HOLD_MS      10        // ... for at most this long (10 ms = 40 readings at 4 kHz; fewer with SMC3Utils'
                                   // PID divider, so it stays 10 ms)

#if TARGET_MAX_RATE > 30000
#error "TARGET_MAX_RATE must be 30000 or less"
#endif
#if POT_FILTER && (POT_HOLD_MS < 1 || POT_HOLD_MS > 60)
#error "POT_HOLD_MS must be 1-60"
#endif

//    COMMAND SET (unchanged):
//
//    [Axx],[Bxx],[Cxx]         Position for Motor 1,2,3, xx = 16-bit big-endian, 0-1023
//    [Dxx],[Exx],[Fxx]         Kp x100 for motor 1,2,3
//    [Gxx],[Hxx],[Ixx]         Ki x100
//    [Jxx],[Kxx],[Lxx]         Kd x100
//    [Mxx],[Nxx],[Oxx]         Ks (derivative sample divider)
//    [Pxy],[Qxy],[Rxy]         PWMmin (x) and PWMmax (y), 0-255
//    [Sxy],[Txy],[Uxy]         Cutoff limit (x) and input clip (y); max = 1023 - value
//    [Vxy],[Wxy],[Xxy]         Feedback dead zone (x) and PWM reverse duty (y)
//    [Zx?]                     PID process divider (Fpid/x)
//    [~xy]                     PWM freq kHz for timer1 (motors 1,2) and timer2 (motor 3)
//    [rdx]                     Read a value; x = A/B/C position, a/b/c status+PWM, D..X parameters,
//                              Y status, Z PID loop count, ~ PWM freqs.  New: $ = worst ISR time in us.
//    [mo1]/[mo2]/[mo3]/[mo0]   Monitor motor n (auto-send position + status every 20 ms) / off
//    [ena]/[en1]/[en2]/[en3]   Enable motors
//    [dis]/[di1]/[di2]/[di3]   Disable motors (brake, stay off until enabled) - new in 1.5
//    [fab]                     Smoothing shift a (0-8) and max speed b*100 counts/s - new in 1.6 ([rdf] reads)
//    [rdd]                     Diagnostics frame "SM3D" (the page only: WiFi, or USB in page mode) - new in 1.7
//    [lvf]                     Matrix view v (0 bars, 1 horizon, 2 quiet) and flags f - new in 1.8 ([rdl] reads)
//    [ift]                     Sensor flags f (bit0 on, bit1 swap roll/pitch, bit2 invert roll, bit3 invert
//                              pitch) and tilt limit t degrees (0 = off) - new in 1.8 ([rdi] reads)
//    [iL?] [iC?] [iS?] [iR?]   Sensor: level now, clear level, set to 100 Hz/115200, search baud again - 1.8
//                              [iSb]: b = WitMotion rate code, bit6 set = angles + gyro + acceleration (1.9)
//    [whl]                     Value register = h*256+l (signed) - 1.9
//    [kf=]                     Seat set-up field f = value register: f = 16*motor(0-2) + 0 trim, 1 travel low,
//                              2 travel high, 3 end stop low, 4 end stop high (pot counts); f = 64 + d: tilt
//                              limit in 0.1 degree for d = 0 roll +, 1 roll -, 2 pitch +, 3 pitch - (0 = none);
//                              f = 80 / 81: the sensor's level roll / pitch x100 (saved at once) - 1.9;
//                              f = 82: soft-start speed, counts/s, 0 = off (saved at once) - 1.9.1
//    [kSV] [kRL] [kCL] [kHn]   Seat set-up: save, reload saved, clear, page holds the seat n x 100 ms - 1.9
//    [kU1] / [kU0]             USB only: the page is on the USB cable for the next 5 s (repeat every second) / no
//                              longer - telemetry and [rdd] come over USB, no WiFi calls meanwhile - 1.9.5
//    [xe1] / [xe0]             Sensor acceleration + gyro after every telemetry burst on / off - 1.9
//    [uP=] [uT=] [uI=] [uJ=]   Firmware update (WiFi client only): value register = the page's web port, the
//                              file's number (http://<page>:<port>/ota/<number, 4 hex digits>.ota), the page's
//                              address bytes 1-2 / 3-4 (0 = the address the page connects from) - 1.9.2
//    [uGO] / [uOK]             Start the update / clear a finished or failed one - 1.9.2
//    [sav]                     Save parameters to EEPROM (and the seat set-up, 1.9)
//    [ver]                     Software version (replies 70 so SMC3Utils is happy)

#include <EEPROM.h>
#include "FspTimer.h"
#include "pwm.h"
#if LED_MATRIX
#include "Arduino_LED_Matrix.h"
#endif
#if ENABLE_WIFI
#include <WiFiS3.h>
#include <OTAUpdate.h>                         // 1.9.2: comes with the UNO R4 boards package (no install needed)
#endif

//----------------------------------------------------------------------------------------------------------------
//  Pins
//----------------------------------------------------------------------------------------------------------------

const uint8_t ENApin[NUM_MOTORS]      = {2, 4, 6};
const uint8_t ENBpin[NUM_MOTORS]      = {3, 5, 7};
const uint8_t PWMpin[NUM_MOTORS]      = {9, 10, 11};
const uint8_t FeedbackPin[NUM_MOTORS] = {A0, A1, A2};

PwmOut Pwm0(9);
PwmOut Pwm1(10);
PwmOut Pwm2(11);
PwmOut *Pwm[NUM_MOTORS] = {&Pwm0, &Pwm1, &Pwm2};

FspTimer PidTimer;

#if LED_MATRIX
ArduinoLEDMatrix Matrix;
uint8_t Frame[8][12];
#endif

//----------------------------------------------------------------------------------------------------------------
//  Parameters (set over serial / EEPROM, read by the ISR).  All in SMC3 units.
//----------------------------------------------------------------------------------------------------------------

volatile int  Target[NUM_MOTORS]         = {512, 512, 512};   // raw commanded position, 0-1023
volatile long Kp_x100[NUM_MOTORS]        = {100, 420, 420};
volatile long Ki_x100[NUM_MOTORS]        = {40, 40, 40};
volatile long Kd_x100[NUM_MOTORS]        = {40, 40, 40};
volatile int  Ks[NUM_MOTORS]             = {1, 1, 1};
volatile int  PWMoffset[NUM_MOTORS]      = {50, 50, 50};      // PWMmin
volatile int  PWMmax[NUM_MOTORS]         = {100, 100, 100};
volatile int  PWMrev[NUM_MOTORS]         = {200, 200, 200};
volatile int  DeadZone[NUM_MOTORS]       = {0, 0, 0};
volatile int  CutoffLimitMin[NUM_MOTORS] = {23, 23, 23};
volatile int  CutoffLimitMax[NUM_MOTORS] = {1000, 1000, 1000};
volatile int  InputClipMin[NUM_MOTORS]   = {100, 100, 100};
volatile int  InputClipMax[NUM_MOTORS]   = {923, 923, 923};
volatile int  Disable[NUM_MOTORS]        = {1, 1, 1};
const int     LiftFactor[NUM_MOTORS]     = {0, 0, 0};         // "was 10" in the original; kept at 0

volatile int  PIDProcessDivider = 1;
volatile byte PowerScale = 7;                                 // 7 normally, 9 after comms timeout
unsigned int  Timer1FreqkHz = DEFAULT_TIMER1_KHZ;
unsigned int  Timer2FreqkHz = DEFAULT_TIMER2_KHZ;

//----------------------------------------------------------------------------------------------------------------
//  Control-loop state (owned by the ISR, read by loop())
//----------------------------------------------------------------------------------------------------------------

volatile int  Feedback_q[NUM_MOTORS] = {512 << POS_SHIFT, 512 << POS_SHIFT, 512 << POS_SHIFT};  // pot, Q4
volatile int  Goal_q[NUM_MOTORS]     = {512 << POS_SHIFT, 512 << POS_SHIFT, 512 << POS_SHIFT};  // smoothed target, Q4
volatile int  PWMout_q[NUM_MOTORS]   = {0, 0, 0};              // drive magnitude incl. PWMmin, Q4 PWM units (0..4080)
volatile int  PWMpid_q[NUM_MOTORS]   = {0, 0, 0};              // raw signed PID output before PWMmin (telemetry only)
long          GoalAcc[NUM_MOTORS]    = {0, 0, 0};              // smoothing accumulators, Q4 << 8 (= Q12)
long          SlewStepQ12 = 0;                                 // max goal change per PID cycle, Q12 (0 = none)
volatile uint8_t  SmoothShift = TARGET_SMOOTH_SHIFT;           // run-time smoothing (0-8)
unsigned int      MaxRate = TARGET_MAX_RATE;                   // run-time max speed, counts/s (0 = off)

// Soft start (1.9.1): a motor that has just been switched on creeps to its target, then runs normally
unsigned int      SoftRate = SOFT_START_RATE;                  // counts/s (0 = off)
long              SoftStepQ12 = 0;                             // max goal change per PID cycle while creeping, Q12
uint16_t          SoftEaseCycles = 2400;                       // PID cycles to reach full creeping speed (0.6 s)
volatile uint8_t  SoftStart[NUM_MOTORS] = {0, 0, 0};           // 1 = still creeping towards its target
volatile uint16_t SoftTicks[NUM_MOTORS] = {0, 0, 0};           // PID cycles since it started creeping
volatile int      SoftOutQ[NUM_MOTORS]  = {0, 0, 0};           // how far past an end stop it is, Q4 (0 = inside)
unsigned long     SoftSince[NUM_MOTORS] = {0, 0, 0};           // millis() when it started (gives up after 40 s)
#define SOFT_MAX_MS  40000UL
#define SOFT_EE      105                                       // EEPROM 105 = 0xB5 "valid", 106-107 = speed

long          PidCumError[NUM_MOTORS]      = {0, 0, 0};
long          PidIntegralLimit[NUM_MOTORS] = {0, 0, 0};
int           PidLastPosition[NUM_MOTORS]  = {0, 0, 0};
long          PidDTerm_x100[NUM_MOTORS]    = {0, 0, 0};
int           PidKdFilterCount[NUM_MOTORS] = {0, 0, 0};

volatile uint32_t PwmPeriodTicks[NUM_MOTORS] = {1, 1, 1};     // timer ticks per PWM period (resolution)
volatile bool     PwmHold = false;                             // set while a PWM timer is being reconfigured
int               PIDProcessCounter = 0;
volatile unsigned long LoopCount = 0;                          // PID updates done (for [rdZ])
unsigned long     LastCount = 0;
volatile uint32_t IsrCyclesMax = 0;                            // worst-case ISR length in CPU cycles

//----------------------------------------------------------------------------------------------------------------
//  Diagnostics (1.7) - read by smc3_scope with [rdd]
//----------------------------------------------------------------------------------------------------------------

#define DIS_ON      0                                          // DisableReason values
#define DIS_CUTOFF  1                                          // pot went past the safety cutoff (Max limit)
#define DIS_COMMAND 2                                          // [dis]/[di1-3] - Disable Motors or unused motor
#define DIS_BOOT    3                                          // not enabled yet after power-up
#define DIS_TILT    4                                          // 1.8: the seat tilted past the tilt limit
#define DIS_UPDATE  5                                          // 1.9.2: switched off for a firmware update

volatile uint8_t  DisableReason[NUM_MOTORS] = {DIS_BOOT, DIS_BOOT, DIS_BOOT};
volatile uint16_t CutoffTrips[NUM_MOTORS]   = {0, 0, 0};       // since power-up
volatile int16_t  CutoffAtFb[NUM_MOTORS]    = {-1, -1, -1};    // pot reading (0-1023) when the cutoff last tripped
volatile uint16_t PotJumps[NUM_MOTORS]      = {0, 0, 0};       // glitches (> POT_JUMP_COUNTS in one cycle) since power-up
volatile uint16_t PotJumpMax_q[NUM_MOTORS]  = {0, 0, 0};       // biggest cycle-to-cycle change since the last [rdd], Q4
volatile int      FbMin_q[NUM_MOTORS]       = {16383, 16383, 16383};   // pot range seen since the last [rdd], Q4
volatile int      FbRaw_q[NUM_MOTORS]       = {512 << POS_SHIFT, 512 << POS_SHIFT, 512 << POS_SHIFT};  // last raw read
volatile uint8_t  FbHeld[NUM_MOTORS]        = {0, 0, 0};       // 1.9.4: readings in a row held back as glitches
volatile int      FbMax_q[NUM_MOTORS]       = {0, 0, 0};

uint8_t       ResetFlags[3] = {0, 0, 0};                       // RSTSR0, RSTSR1, RSTSR2 as found at boot
uint8_t       ResetCwsfOk = 0;                                 // 1 = the cold/warm-start flag could be set
uint32_t      UsbFramesOk = 0;                                 // complete [xyz] commands from USB (SimHub, SMC3Utils)
uint32_t      UsbTargetFrames = 0;                             // ... of which position commands [A]/[B]/[C]
uint32_t      UsbBadBytes = 0;                                 // bytes thrown away while looking for a valid command
uint32_t      TcpFramesOk = 0;                                 // commands from smc3_scope over WiFi
unsigned long LastUsbPacketMillis = 0;
unsigned long LastUsbTargetMillis = 0;
uint32_t      LoopMaxUs = 0;                                   // longest loop() pass since the last [rdd]

//----------------------------------------------------------------------------------------------------------------
//  Serial / monitor state (loop() only)
//----------------------------------------------------------------------------------------------------------------

unsigned int RxByte = 0;
int          BufferEnd = -1;
unsigned int RxBuffer[5] = {0};
byte         errorcount = 0;
int          SerialFeedbackEnabled = 0;
unsigned long LastPacketMillis = 0;
unsigned long LastMonitorMillis = 0;
uint8_t      ReplyTarget = 0;                 // 0 = reply over USB, 1 = reply to the TCP client
bool         UsbPage = false;                 // 1.9.5: the page is on the USB cable ([kU1]): telemetry there, no WiFi
unsigned long UsbPageMillis = 0;              // ... last [kU1]
uint32_t     UsbPageStarts = 0;               // ... how often it was switched on (test / diagnostics)
#define USB_PAGE_TIMEOUT_MS 5000              // page mode ends this long after the last [kU1]

//----------------------------------------------------------------------------------------------------------------
//  Network state (loop() only)
//----------------------------------------------------------------------------------------------------------------

#if ENABLE_WIFI
WiFiServer    TcpServer(TELEMETRY_PORT);
WiFiClient    TcpClient;
bool          WifiUp = false;
bool          ClientUp = false;
char          WifiText[20] = "E9";             // IP address as text, or an E-code, for the display and the banner
unsigned long WifiLastAttempt = 0;
unsigned long LastIpScroll = 0;
bool          PendingIpScroll = false;         // scroll the address once (it arrived late, or WiFi came back)
unsigned long NetLastSample = 0, NetLastSend = 0, NetLastPoll = 0, NetLastCheck = 0, NetLastAccept = 0;
int           TcpBufferEnd = -1;
unsigned int  TcpRxBuffer[5] = {0};

#define TEL_SAMPLE_BYTES   38
#define TEL_MAX_SAMPLES    (TELEMETRY_SEND_MS / TELEMETRY_SAMPLE_MS + 1)
#define TEL_EXTRA_BYTES    18                  // 1.9: "SM3X" payload (sensor acceleration + gyro)
uint8_t       TelBuf[6 + TEL_MAX_SAMPLES * TEL_SAMPLE_BYTES + 6 + TEL_EXTRA_BYTES];
uint8_t       TelCount = 0;
bool          TelExtras = false;               // [xe1]: the page wants "SM3X" frames (off for every new client)

// 1.9.2: WiFi memory - the name and password the board last joined with.  EEPROM 112 = 0xC7 "valid", 113 = length of
// the name, 114-145 the name, 146 = length of the password, 147-210 the password.
#define WIFI_EE        112
#define WIFI_EE_PASS   146
char          WifiMemSsid[33] = "";
char          WifiMemPass[65] = "";
bool          WifiMem = false;                 // EEPROM holds a name and password
uint8_t       WifiFrom = 0;                    // joined with 0 = the sketch's name and password, 1 = the remembered ones
uint32_t      WifiFw = 0;                      // the WiFi module's firmware, 0xMMmmpp (0 = unknown)

// 1.9.2: firmware update over WiFi
#define OTA_IDLE         0
#define OTA_PREP         1                     // motors braking
#define OTA_BEGIN        2                     // asking the WiFi module to fetch the file
#define OTA_DOWNLOAD     3
#define OTA_VERIFY       4                     // the WiFi module checks the file
#define OTA_FLASH        5                     // the WiFi module programs the board (this firmware stops here)
#define OTA_FAILED       6
#define OTA_FILE         "/update.bin"         // on the WiFi module's own storage
#define OTA_ERR_SIMHUB   -100                  // SimHub (or SMC3Utils) is sending over USB
#define OTA_ERR_OLDWIFI  -101                  // WiFi module firmware older than 0.4.0
#define OTA_ERR_NOURL    -102                  // the page didn't say where the file is
#define OTA_ERR_STALLED  -103                  // the download stopped moving
#define OTA_ERR_FLASH    -104                  // the WiFi module could not program the board
#define OTA_NO_PROGRESS  -1000                 // this boards package has no non-blocking download (before 1.3.0)
OTAUpdate     Ota;
uint8_t       OtaState = OTA_IDLE;
int16_t       OtaCode = 0;                     // 0 = fine, < 0 = why it stopped (OTA_ERR_... or the WiFi module's code)
uint32_t      OtaDone = 0, OtaTotal = 0;       // bytes downloaded / size of the file
bool          OtaProgress = false;             // the download reports its progress
uint16_t      OtaPort = 0, OtaToken = 0;       // [uP=] [uT=]
uint16_t      OtaIpHi = 0, OtaIpLo = 0;        // [uI=] [uJ=] (0 = the address the page connects from)
char          OtaUrl[64] = "";
unsigned long OtaSince = 0, OtaLastPoll = 0, OtaLastMove = 0, OtaLastFrame = 0;
bool          OtaFrameDue = false;             // send an "SM3U" frame with the next network service
#endif

static inline bool UpdateRunning()             // 1.9.2: a firmware update is under way (the motors stay off)
{
#if ENABLE_WIFI
    return OtaState >= OTA_PREP && OtaState <= OTA_FLASH;
#else
    return false;
#endif
}

int8_t UpdateShow()                            // 1.9.2, for the matrix: -1 no update, 0-100 downloaded %, 101 installing
{
#if ENABLE_WIFI
    if (!UpdateRunning()) { return -1; }
    if (OtaState >= OTA_VERIFY) { return 101; }
    if (OtaState == OTA_DOWNLOAD && OtaTotal > 0 && OtaProgress)
    {
        uint32_t pct = (OtaDone >= OtaTotal) ? 100UL : (OtaDone * 100UL) / OtaTotal;
        return (int8_t)pct;
    }
    return 0;
#else
    return -1;
#endif
}

static inline bool UpdateFailed()              // 1.9.2: the last update failed (the matrix says so until it is cleared)
{
#if ENABLE_WIFI
    return OtaState == OTA_FAILED;
#else
    return false;
#endif
}

//----------------------------------------------------------------------------------------------------------------
//  Motion sensor (WT901C) and LED matrix settings - 1.8.  Saved at once in EEPROM 56-65 (56 = 0xA6 "valid").
//----------------------------------------------------------------------------------------------------------------

uint8_t       SensorFlags  = 0;               // bit0 sensor fitted (on), bit1 swap roll/pitch, bit2 invert roll, bit3 invert pitch
uint8_t       TiltLimitDeg = 0;               // 1.8's single tilt limit, still answered by [rdi]: the tightest of the four below
int16_t       LevelRoll_x100 = 0, LevelPitch_x100 = 0;   // "Level now": raw sensor angles that count as level
uint8_t       LedView  = LED_DEFAULT_VIEW;
uint8_t       LedFlags = LED_DEFAULT_FLAGS;

float         ImuRawRoll = 0, ImuRawPitch = 0, ImuYaw = 0;    // as the sensor sends them, degrees
float         SeatRoll = 0, SeatPitch = 0;                     // levelled and turned the way the sensor is mounted
unsigned long ImuLastMillis = 0;              // last angle packet
unsigned long ImuLastValid  = 0;              // last packet of any kind with a good checksum
uint32_t      ImuPackets = 0;                 // good packets since power-up
uint16_t      ImuBadSums = 0;                 // packets with a bad checksum since power-up
uint16_t      ImuAngleCount = 0, ImuRateHz = 0;   // angle packets this second / last second
unsigned long ImuRateSince = 0;
uint8_t       ImuBaudIdx = 0;                 // index into ImuBauds[]
unsigned long ImuBaudSince = 0;               // when that baud rate was set
unsigned long ImuScanUntil = 0;               // keep looking for the sensor until then even if it is not switched on
uint8_t       TiltHits = 0;
uint16_t      TiltTrips = 0;
unsigned long TiltGraceUntil = 0;             // no tilt check for a moment after the motors are enabled
unsigned long LastPosMillis = 0;              // last position command from SimHub or the page (idle trace)

// 1.9: acceleration and gyro from the sensor (raw: acceleration / 32768 * 16 g, gyro / 32768 * 2000 deg/s)
int16_t       ImuAcc[3]  = {0, 0, 0};
int16_t       ImuGyro[3] = {0, 0, 0};
int16_t       ImuTemp_x100 = 0;
unsigned long ImuAccMillis = 0, ImuGyroMillis = 0;
uint16_t      ImuAccCount = 0, ImuGyroCount = 0, ImuAccHz = 0, ImuGyroHz = 0;
uint16_t      AccPeak_mg = 0;                 // biggest |acceleration| - 1 g since the last telemetry burst
uint8_t       AccBurstN = 0;                  // acceleration packets since the last telemetry burst

//----------------------------------------------------------------------------------------------------------------
//  Seat set-up - 1.9.  Made by smc3_scope's Seat setup; saved with [kSV] or [sav] in EEPROM 66-104 (66 = 0xB9).
//----------------------------------------------------------------------------------------------------------------

int16_t       Trim[NUM_MOTORS]     = {0, 0, 0};          // level trim, added to every position command (counts)
int16_t       TravelLo[NUM_MOTORS] = {0, 0, 0};          // position commands never go below / above (pot counts)
int16_t       TravelHi[NUM_MOTORS] = {1023, 1023, 1023};
int16_t       EndLo[NUM_MOTORS]    = {0, 0, 0};          // the motor switches off if its pot gets past these
int16_t       EndHi[NUM_MOTORS]    = {1023, 1023, 1023};
int16_t       TiltLim_x10[4]       = {0, 0, 0, 0};       // 0.1 deg: roll + (right side down), roll - (left side down),
                                                         // pitch + (front up), pitch - (front down); 0 = no limit
volatile int  TargetLo[NUM_MOTORS] = {100, 100, 100};    // in force: the SMC3 clip and the travel limits together
volatile int  TargetHi[NUM_MOTORS] = {923, 923, 923};
volatile int  CutLo[NUM_MOTORS]    = {23, 23, 23};       // in force: the SMC3 cutoff and the end stops together (ISR)
volatile int  CutHi[NUM_MOTORS]    = {1000, 1000, 1000};
int16_t       ValueReg = 0;                              // [w hi lo]
bool          PageHold = false;                          // [kH n]: SimHub's position commands wait ...
unsigned long PageHoldUntil = 0;                         // ... until then
uint32_t      UsbHeldFrames = 0;                         // ... and how many were ignored

#if IMU_PORT
const uint32_t ImuBauds[] = {9600, 115200, 230400, 57600, 38400, 19200, 4800};
#define IMU_NBAUDS 7
#define IMU_FAST_BAUD_IDX 1                   // 115200
uint8_t       ImuCmd[10][5];                  // sensor set-up commands waiting to be sent, 120 ms apart
uint8_t       ImuCmdN = 0, ImuCmdPos = 0;
int8_t        ImuCmdBaudSwitchAt = -1;        // switch our own baud rate before sending this command
unsigned long ImuCmdLast = 0;
#endif

static inline bool SensorFresh() { return ImuLastMillis != 0 && (millis() - ImuLastMillis) < 500; }

//****************************************************************************************************************
//    Small helpers
//****************************************************************************************************************

static inline int StatusByte()
{
    return PIDProcessDivider * 16 + Disable[0] + (Disable[1] * 2) + (Disable[2] * 4);
}

// How this firmware was built - reported by [rd!] (bit0 since 1.4, the rest since 1.7)
static inline int BuildFlags()
{
    int flags = 0;
#ifdef REVERSE_MOTOR1
    flags |= 1;
#endif
#ifdef MODE2
    flags |= 2;                                // IBT-2 / BTS7960 wiring (MODE1 = PWM + two direction pins)
#endif
#if IMU_PORT
    flags |= 4;                                // listens for the motion sensor on Serial1
#endif
#if LED_MATRIX
    flags |= 8;
#endif
#if ENABLE_WIFI && WIFI_STATIC_IP
    flags |= 16;
#endif
#if POT_FILTER
    flags |= 32;                               // 1.9.4: pot glitch filter
#endif
    return flags;
}

static inline uint32_t CycleCount()
{
#ifdef DWT
    return DWT->CYCCNT;
#else
    return 0;
#endif
}

//****************************************************************************************************************
//    PWM output.  q is in Q4 PWM units (0..4080).  Scaled to the timer period so every timer tick is usable.
//****************************************************************************************************************

static inline void SetPwmQ(uint8_t m, int q)
{
    if (PwmHold) { return; }
    if (q < 0) { q = 0; }
    else if (q > PWM_FULL_Q) { q = PWM_FULL_Q; }
    uint32_t ticks = ((uint32_t)q * PwmPeriodTicks[m]) / PWM_FULL_Q;
    Pwm[m]->pulseWidth_raw((int)ticks);
}

void ConfigurePwm(uint8_t m, float freqHz)
{
    PwmHold = true;
    Pwm[m]->end();
    Pwm[m]->begin(freqHz, 0.0f);
    PwmPeriodTicks[m] = Pwm[m]->get_timer()->get_period_raw();
    if (PwmPeriodTicks[m] < 1) { PwmPeriodTicks[m] = 1; }
    PwmHold = false;
}

void ApplyPwmFrequencies()
{
    ConfigurePwm(0, (float)Timer1FreqkHz * 1000.0f);
    ConfigurePwm(1, (float)Timer1FreqkHz * 1000.0f);
    ConfigurePwm(2, (float)Timer2FreqkHz * 1000.0f);
}

//****************************************************************************************************************
//    Derived constants (called after EEPROM read and whenever Ki or the divider change over serial)
//****************************************************************************************************************

void UpdateSmoothingRate()
{
    // counts/s -> Q12 counts per PID cycle: rate * 4096 / PID_RATE_HZ * divider
    long q = ((long)MaxRate * 4096L / PID_RATE_HZ) * PIDProcessDivider;
    if (MaxRate > 0 && q < 1) { q = 1; }
    SlewStepQ12 = q;
    q = ((long)SoftRate * 4096L / PID_RATE_HZ) * PIDProcessDivider;    // 1.9.1: soft start
    if (SoftRate > 0 && q < 1) { q = 1; }
    SoftStepQ12 = q;
    long e = (long)PID_RATE_HZ * 6 / 10 / (PIDProcessDivider > 0 ? PIDProcessDivider : 1);
    SoftEaseCycles = (uint16_t)(e < 1 ? 1 : (e > 60000 ? 60000 : e));
}

bool AnySoftStart()                             // 1.9.1: is any switched-on motor still creeping to its target?
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++) { if (SoftStart[m] && Disable[m] == 0) { return true; } }
    return false;
}

void UpdatePIDLimits()
{
    // Clamp the integral so the I term alone can never exceed full PWM:  Ki * CumError >> I_SHIFT >> 7 <= PWM_FULL_Q
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        PidIntegralLimit[m] = (Ki_x100[m] > 0) ? (((long)PWM_FULL_Q) << (I_SHIFT + 7)) / Ki_x100[m] : 0;
    }
}

//****************************************************************************************************************
//    Motor enable / disable
//****************************************************************************************************************

void DisableMotor(uint8_t m)          // brakes on; may be called from the ISR
{
    Disable[m] = 1;
    digitalWrite(ENApin[m], LOW);
    digitalWrite(ENBpin[m], LOW);
    SetPwmQ(m, 0);
}

void SoftStartFrom(uint8_t m)        // 1.9.1: creep from where the motor is now to its target (loop() only)
{
    // No interrupt lock needed: once SoftStart is set the ISR only lets the goal creep, so whatever it does between
    // these lines is overwritten (GoalAcc is one 32-bit store) or is a step of a fraction of a count.
    int fb = Feedback_q[m];
    int lo = CutLo[m] << POS_SHIFT, hi = CutHi[m] << POS_SHIFT;
    SoftOutQ[m] = (fb < lo) ? (lo - fb) : ((fb > hi) ? (fb - hi) : 0);
    SoftTicks[m] = 0;
    SoftStart[m] = 1;
    GoalAcc[m] = (long)fb << 8;                  // Q4 -> Q12: the goal starts exactly where the motor is
    Goal_q[m] = fb;
    PidCumError[m] = 0;                          // nothing wound up while it was off
    PidLastPosition[m] = fb;
    PidDTerm_x100[m] = 0;
    SoftSince[m] = millis();
}

void EnableMotor(uint8_t m)           // loop() only
{
    if (UpdateRunning()) { return; }   // 1.9.2: not during a firmware update
    TiltGraceUntil = millis() + 1500;  // let the seat come back inside the tilt limit first
    if (Disable[m] != 0 && SoftRate > 0) { SoftStartFrom(m); }   // 1.9.1: no jump - it creeps to its target
    DisableReason[m] = DIS_ON;
    Disable[m] = 0;
#ifdef MODE2
    // IBT-2: pulse the enable low then leave it high to clear any over-temperature shutdown
    digitalWrite(ENBpin[m], LOW);
    digitalWrite(ENBpin[m], HIGH);
#endif
}

//****************************************************************************************************************
//    Target smoothing - identical to SMC3_smooth.ino but in Q12 (count x 4096)
//****************************************************************************************************************

static inline int SmoothTarget(uint8_t m, int target10)
{
    long diff = ((long)target10 << 12) - GoalAcc[m];
    long step;
    uint8_t sh = SmoothShift;
    if (sh > 0)
    {
        step = diff >> sh;
        if (step == 0) { step = diff; }
    }
    else
    {
        step = diff;
    }
    if (SlewStepQ12 > 0)
    {
        if (step > SlewStepQ12) { step = SlewStepQ12; }
        else if (step < -SlewStepQ12) { step = -SlewStepQ12; }
    }
    if (SoftStart[m])                           // 1.9.1: creeping after power-up / Enable Motors
    {
        long ad = (diff < 0) ? -diff : diff;
        if (ad <= (2L << 12) || SoftStepQ12 <= 0)
        {
            SoftStart[m] = 0;                    // caught up: normal from now on
        }
        else
        {
            long lim = SoftStepQ12;
            uint16_t t = SoftTicks[m];
            if (t < SoftEaseCycles)              // ease in: from standstill to full creeping speed in 0.6 s
            {
                SoftTicks[m] = t + 1;
                lim = lim * (long)(t + 1) / (long)SoftEaseCycles;
            }
            long outLim = (ad >> 10) * PIDProcessDivider + 1;   // ease out: slows down over the last ~15 counts
            if (lim > outLim) { lim = outLim; }
            if (lim < 1) { lim = 1; }
            if (step > lim) { step = lim; }
            else if (step < -lim) { step = -lim; }
        }
    }
    GoalAcc[m] += step;
    return (int)((GoalAcc[m] + 128) >> 8);      // Q12 -> Q4
}

//****************************************************************************************************************
//    PID (positions in Q4, output in Q4 PWM units, same gains/scaling as SMC3_smooth.ino)
//****************************************************************************************************************

static inline int CalcMotorPID(uint8_t m, int goal_q, int fb_q)
{
    int Error = goal_q - fb_q;
    int dz_q = DeadZone[m] << POS_SHIFT;
    int64_t pTerm = (int64_t)Kp_x100[m] * Error;

    if ((Error <= dz_q && Error >= -dz_q) || PidIntegralLimit[m] == 0)
    {
        PidCumError[m] = 0;
    }
    else
    {
        PidCumError[m] += Error;
        if (PidCumError[m] > PidIntegralLimit[m]) { PidCumError[m] = PidIntegralLimit[m]; }
        else if (PidCumError[m] < -PidIntegralLimit[m]) { PidCumError[m] = -PidIntegralLimit[m]; }
    }
    int64_t iTerm = ((int64_t)Ki_x100[m] * PidCumError[m]) >> I_SHIFT;

    PidKdFilterCount[m]++;
    if (PidKdFilterCount[m] >= Ks[m])
    {
        int Delta = fb_q - PidLastPosition[m];
        PidLastPosition[m] = fb_q;
        PidKdFilterCount[m] = 0;
        if (Delta > 4095) { Delta = 4095; } else if (Delta < -4095) { Delta = -4095; }
        int64_t dRaw = ((int64_t)Kd_x100[m] * Delta) << D_SHIFT;
        if (dRaw > 2000000000LL) { dRaw = 2000000000LL; } else if (dRaw < -2000000000LL) { dRaw = -2000000000LL; }
#if D_FILTER_SHIFT > 0
        PidDTerm_x100[m] += (long)((dRaw - PidDTerm_x100[m]) >> D_FILTER_SHIFT);
#else
        PidDTerm_x100[m] = (long)dRaw;
#endif
    }

    int64_t out = (pTerm + iTerm - PidDTerm_x100[m]) >> PowerScale;
    if (out > PWM_FULL_Q) { out = PWM_FULL_Q; } else if (out < -PWM_FULL_Q) { out = -PWM_FULL_Q; }
    return (int)out;
}

//****************************************************************************************************************
//    Drive the H-bridge from the PID result - same decision tree as the original, in Q4 units
//****************************************************************************************************************

static inline void SetOutputsMotor(uint8_t m)
{
    int fb   = Feedback_q[m];
    int goal = Goal_q[m];
    int out  = PWMout_q[m];
    int clipMax_q = InputClipMax[m] << POS_SHIFT;
    int clipMin_q = InputClipMin[m] << POS_SHIFT;
    int dz_q      = DeadZone[m]  << POS_SHIFT;
    int offset_q  = PWMoffset[m] << PWM_FRAC;
    int max_q     = PWMmax[m]    << PWM_FRAC;
    int rev_q     = PWMrev[m]    << PWM_FRAC;
    int lift_q    = LiftFactor[m] << PWM_FRAC;

    bool rev = (PWMrev[m] != 0) && !SoftStart[m];   // 1.9.1: no full-duty "drive back" while creeping
#ifdef MODE1
    if ((fb > clipMax_q) && rev)
    {
        out = rev_q;
        digitalWrite(ENApin[m], LOW);  digitalWrite(ENBpin[m], HIGH);
        SetPwmQ(m, rev_q);
    }
    else if ((fb < clipMin_q) && rev)
    {
        out = rev_q;
        digitalWrite(ENApin[m], HIGH); digitalWrite(ENBpin[m], LOW);
        SetPwmQ(m, rev_q);
    }
    else if ((goal > (fb + dz_q)) || (goal < (fb - dz_q)))
    {
        if (out >= 0)
        {
            out += offset_q;
            if (out > max_q + lift_q) { out = max_q + lift_q; }
            digitalWrite(ENApin[m], HIGH); digitalWrite(ENBpin[m], LOW);
        }
        else
        {
            out = -out + offset_q;
            if (out > max_q) { out = max_q; }
            digitalWrite(ENApin[m], LOW);  digitalWrite(ENBpin[m], HIGH);
        }
        SetPwmQ(m, out);
    }
    else
    {
        digitalWrite(ENApin[m], LOW);  digitalWrite(ENBpin[m], LOW);     // brake
        out = offset_q;
        SetPwmQ(m, 0);
    }
#endif

#ifdef MODE2
    if ((fb > clipMax_q) && rev)
    {
        out = rev_q;
        digitalWrite(ENApin[m], LOW);
        SetPwmQ(m, rev_q);
    }
    else if ((fb < clipMin_q) && rev)
    {
        out = rev_q;
        digitalWrite(ENApin[m], HIGH);
        SetPwmQ(m, PWM_FULL_Q - rev_q);
    }
    else if ((goal > (fb + dz_q)) || (goal < (fb - dz_q)))
    {
        if (out >= 0)
        {
            out += offset_q;
            if (out > max_q + lift_q) { out = max_q + lift_q; }
            digitalWrite(ENApin[m], HIGH);
            SetPwmQ(m, PWM_FULL_Q - out);          // motor driven when PWM = 0 (unset)
        }
        else
        {
            out = -out + offset_q;
            if (out > max_q) { out = max_q; }
            digitalWrite(ENApin[m], LOW);
            SetPwmQ(m, out);                       // motor driven when PWM = 1 (set)
        }
    }
    else
    {
        digitalWrite(ENApin[m], LOW);              // brake
        out = offset_q;
        SetPwmQ(m, 0);
    }
#endif

    PWMout_q[m] = out;
}

//****************************************************************************************************************
//    The 4 kHz control interrupt
//****************************************************************************************************************

void PidIsr(timer_callback_args_t *p_args)
{
    (void)p_args;

    PIDProcessCounter++;
    if (PIDProcessCounter < PIDProcessDivider) { return; }
    PIDProcessCounter = 0;

    uint32_t t0 = CycleCount();
#if POT_FILTER
    // 1.9.4: readings a glitch may be held for - POT_HOLD_MS at this PID rate (4 kHz / SMC3Utils' divider)
    const uint8_t holdN = (uint8_t)((POT_HOLD_MS * (PID_RATE_HZ / 1000) + PIDProcessDivider - 1) / PIDProcessDivider);
#endif

    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        Goal_q[m] = SmoothTarget(m, Target[m]);
        int fb = analogRead(FeedbackPin[m]);

        // diagnostics: pot glitches (a jump no motor could make in 1/4 ms) and the range seen - on the raw readings,
        // so a bad pot shows even with the filter below
        int jump = fb - FbRaw_q[m];
        if (jump < 0) { jump = -jump; }
        if (jump > (POT_JUMP_COUNTS << POS_SHIFT) && PotJumps[m] < 65535) { PotJumps[m]++; }
        if (jump > PotJumpMax_q[m]) { PotJumpMax_q[m] = (uint16_t)jump; }
        if (fb < FbMin_q[m]) { FbMin_q[m] = fb; }
        if (fb > FbMax_q[m]) { FbMax_q[m] = fb; }
        FbRaw_q[m] = fb;
#if POT_FILTER
        // 1.9.4: a reading this far from the last good one is a glitch, not the motor: the last good one stands in,
        // for up to POT_HOLD_MS in a row; a change that lasts longer is real and goes through
        {
            int dg = fb - Feedback_q[m];
            if (dg < 0) { dg = -dg; }
            if (dg > (POT_JUMP_COUNTS << POS_SHIFT) && FbHeld[m] < holdN)
            {
                FbHeld[m]++;
                fb = Feedback_q[m];
            }
            else
            {
                FbHeld[m] = 0;
            }
        }
#endif
        Feedback_q[m] = fb;

        bool past = (fb > (CutHi[m] << POS_SHIFT)) || (fb < (CutLo[m] << POS_SHIFT));   // SMC3 cutoff + 1.9 end stops
        if (past && SoftStart[m] && SoftOutQ[m] > 0)
        {
            // 1.9.1: creeping back after power-up / Enable Motors from past an END STOP (the seat rode down while the
            // rig was off) is allowed - as long as it doesn't go 16 counts further out than it has been, and stays
            // inside the SMC3 cutoff (a broken pot reads 0 or 1023 and still trips at once)
            int lo = CutLo[m] << POS_SHIFT, hi = CutHi[m] << POS_SHIFT;
            int out = (fb < lo) ? (lo - fb) : (fb - hi);
            bool hard = (fb > (CutoffLimitMax[m] << POS_SHIFT)) || (fb < (CutoffLimitMin[m] << POS_SHIFT));
            if (!hard && out <= SoftOutQ[m] + (16 << POS_SHIFT))
            {
                past = false;
                if (out < SoftOutQ[m]) { SoftOutQ[m] = out; }
            }
        }
        else if (!past && SoftOutQ[m] != 0)
        {
            // back inside: once well inside (16 counts) the end stops count as usual again; just inside, pot noise
            // may still cross the end stop for a moment
            int lo = CutLo[m] << POS_SHIFT, hi = CutHi[m] << POS_SHIFT;
            int in = (fb - lo < hi - fb) ? (fb - lo) : (hi - fb);
            SoftOutQ[m] = (in > (16 << POS_SHIFT) || !SoftStart[m]) ? 0 : 1;
        }
        if (past)
        {
            if (Disable[m] == 0)                    // a new trip (not a motor that is already off)
            {
                if (CutoffTrips[m] < 65535) { CutoffTrips[m]++; }
                CutoffAtFb[m] = (int16_t)(fb >> POS_SHIFT);
                DisableReason[m] = DIS_CUTOFF;
            }
            DisableMotor(m);
        }

        PWMout_q[m] = CalcMotorPID(m, Goal_q[m], fb);
        PWMpid_q[m] = PWMout_q[m];

        if (Disable[m] == 0)
        {
            SetOutputsMotor(m);
        }
        else
        {
            PWMout_q[m] = 0;
        }
    }

    LoopCount++;

    uint32_t dt = CycleCount() - t0;
    if (dt > IsrCyclesMax) { IsrCyclesMax = dt; }
}

// 1.9.6: the receive and receive-error interrupts of the USB port (SCI9) and the motion sensor's port (SCI2) get a
// higher priority than the PID interrupt - their receive register holds one byte, so they can't wait for it (see
// v1.9.6).  The core links its interrupts to the 32 ICU slots when a port begins, so this runs after each begin().
void RaiseSerialRxPriority()
{
    for (uint32_t i = 0; i < BSP_ICU_VECTOR_MAX_ENTRIES; i++)
    {
        uint32_t ev = R_ICU->IELSR[i] & R_ICU_IELSR_IELS_Msk;
        if (ev == (uint32_t)ELC_EVENT_SCI9_RXI || ev == (uint32_t)ELC_EVENT_SCI9_ERI ||
            ev == (uint32_t)ELC_EVENT_SCI2_RXI || ev == (uint32_t)ELC_EVENT_SCI2_ERI)
        {
            NVIC_SetPriority((IRQn_Type)i, SERIAL_RX_IRQ_PRIORITY);
        }
    }
}

bool StartPidTimer()
{
    uint8_t type = GPT_TIMER;
    int8_t ch = FspTimer::get_available_timer(type);          // picks GPT if one is free, else AGT
    if (ch < 0)
    {
        ch = FspTimer::get_available_timer(type, true);
    }
    if (ch < 0) { return false; }
    if (!PidTimer.begin(TIMER_MODE_PERIODIC, type, ch, (float)PID_RATE_HZ, 0.0f, PidIsr)) { return false; }
    if (!PidTimer.setup_overflow_irq(PID_IRQ_PRIORITY)) { return false; }
    if (!PidTimer.open()) { return false; }
    if (!PidTimer.start()) { return false; }
    return true;
}

//****************************************************************************************************************
//    EEPROM - layout identical to SMC3 v0.70 (addresses 0-52)
//****************************************************************************************************************

void WriteEEPRomWord(int address, int intvalue)
{
    EEPROM.update(address, (uint8_t)(intvalue / 256));
    EEPROM.update(address + 1, (uint8_t)(intvalue - 256 * (intvalue / 256)));
}

int ReadEEPRomWord(int address)
{
    int high = EEPROM.read(address);
    int low  = EEPROM.read(address + 1);
    return (high * 256) + low;
}

void WriteEEProm()
{
    EEPROM.update(0, 114);
    EEPROM.update(1, CutoffLimitMin[0]);
    EEPROM.update(2, InputClipMin[0]);
    EEPROM.update(5, DeadZone[0]);
    EEPROM.update(6, CutoffLimitMin[1]);
    EEPROM.update(7, InputClipMin[1]);
    EEPROM.update(10, DeadZone[1]);
    WriteEEPRomWord(11, Kp_x100[0]);
    WriteEEPRomWord(13, Ki_x100[0]);
    WriteEEPRomWord(15, Kd_x100[0]);
    WriteEEPRomWord(17, Kp_x100[1]);
    WriteEEPRomWord(19, Ki_x100[1]);
    WriteEEPRomWord(21, Kd_x100[1]);
    EEPROM.update(23, PWMoffset[0]);
    EEPROM.update(24, PWMoffset[1]);
    EEPROM.update(25, PWMmax[0]);
    EEPROM.update(26, PWMmax[1]);
    EEPROM.update(27, PWMrev[0]);
    EEPROM.update(28, PWMrev[1]);
    EEPROM.update(29, PWMrev[2]);
    EEPROM.update(31, CutoffLimitMin[2]);
    EEPROM.update(32, InputClipMin[2]);
    EEPROM.update(35, DeadZone[2]);
    WriteEEPRomWord(36, Kp_x100[2]);
    WriteEEPRomWord(38, Ki_x100[2]);
    WriteEEPRomWord(40, Kd_x100[2]);
    EEPROM.update(42, PWMoffset[2]);
    EEPROM.update(43, PWMmax[2]);
    WriteEEPRomWord(44, Ks[0]);
    WriteEEPRomWord(46, Ks[1]);
    WriteEEPRomWord(48, Ks[2]);
    EEPROM.update(50, constrain(PIDProcessDivider, 1, 10));
    EEPROM.update(51, Timer1FreqkHz);
    EEPROM.update(52, Timer2FreqkHz);
    EEPROM.update(53, SmoothShift);               // 1.6: smoothing and max speed, 55 = "these are valid" marker
    EEPROM.update(54, (uint8_t)(MaxRate / 100));
    EEPROM.update(55, 0x5A);
}

void ReadEEProm()
{
    if (EEPROM.read(0) != 114)          // never saved: write the defaults
    {
        WriteEEProm();
        return;
    }
    CutoffLimitMin[0] = EEPROM.read(1);  InputClipMin[0] = EEPROM.read(2);
    DeadZone[0] = EEPROM.read(5);
    CutoffLimitMin[1] = EEPROM.read(6);  InputClipMin[1] = EEPROM.read(7);
    DeadZone[1] = EEPROM.read(10);
    Kp_x100[0] = ReadEEPRomWord(11);  Ki_x100[0] = ReadEEPRomWord(13);  Kd_x100[0] = ReadEEPRomWord(15);
    Kp_x100[1] = ReadEEPRomWord(17);  Ki_x100[1] = ReadEEPRomWord(19);  Kd_x100[1] = ReadEEPRomWord(21);
    PWMoffset[0] = EEPROM.read(23);   PWMoffset[1] = EEPROM.read(24);
    PWMmax[0] = EEPROM.read(25);      PWMmax[1] = EEPROM.read(26);
    PWMrev[0] = EEPROM.read(27);      PWMrev[1] = EEPROM.read(28);      PWMrev[2] = EEPROM.read(29);
    CutoffLimitMin[2] = EEPROM.read(31); InputClipMin[2] = EEPROM.read(32);
    DeadZone[2] = EEPROM.read(35);
    Kp_x100[2] = ReadEEPRomWord(36);  Ki_x100[2] = ReadEEPRomWord(38);  Kd_x100[2] = ReadEEPRomWord(40);
    PWMoffset[2] = EEPROM.read(42);   PWMmax[2] = EEPROM.read(43);
    Ks[0] = ReadEEPRomWord(44);       Ks[1] = ReadEEPRomWord(46);       Ks[2] = ReadEEPRomWord(48);
    PIDProcessDivider = constrain(EEPROM.read(50), 1, 10);
    Timer1FreqkHz = EEPROM.read(51);
    Timer2FreqkHz = EEPROM.read(52);
    if (Timer1FreqkHz < 1 || Timer1FreqkHz > 100) { Timer1FreqkHz = DEFAULT_TIMER1_KHZ; }
    if (Timer2FreqkHz < 1 || Timer2FreqkHz > 100) { Timer2FreqkHz = DEFAULT_TIMER2_KHZ; }
    if (EEPROM.read(55) == 0x5A)                  // saved by firmware 1.6 or newer
    {
        uint8_t sh = EEPROM.read(53);
        SmoothShift = (sh > 8) ? 8 : sh;
        MaxRate = (unsigned int)EEPROM.read(54) * 100;
    }

    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        CutoffLimitMax[m] = 1023 - CutoffLimitMin[m];
        InputClipMax[m]   = 1023 - InputClipMin[m];
    }
}

// 1.9: the limits in force are the tighter of SMC3's (clip, cutoff - the same both ways) and the seat set-up's
// (travel limits, end stops - set separately each way).  Called whenever either changes.
void UpdateLimits()
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        int lo = (TravelLo[m] > InputClipMin[m]) ? TravelLo[m] : InputClipMin[m];
        int hi = (TravelHi[m] < InputClipMax[m]) ? TravelHi[m] : InputClipMax[m];
        if (hi < lo + 8) { lo = InputClipMin[m]; hi = InputClipMax[m]; }      // makes no sense: SMC3's alone
        int clo = (EndLo[m] > CutoffLimitMin[m]) ? EndLo[m] : CutoffLimitMin[m];
        int chi = (EndHi[m] < CutoffLimitMax[m]) ? EndHi[m] : CutoffLimitMax[m];
        if (chi < clo + 8) { clo = CutoffLimitMin[m]; chi = CutoffLimitMax[m]; }
        TargetLo[m] = lo;
        TargetHi[m] = hi;
        CutLo[m] = clo;
        CutHi[m] = chi;
        int t = Target[m];
        if (t < lo) { Target[m] = lo; } else if (t > hi) { Target[m] = hi; }
    }
    int16_t tight = 0;                           // 1.8's single value for [rdi]: the tightest limit, whole degrees
    for (uint8_t d = 0; d < 4; d++)
    {
        if (TiltLim_x10[d] > 0 && (tight == 0 || TiltLim_x10[d] < tight)) { tight = TiltLim_x10[d]; }
    }
    TiltLimitDeg = (tight == 0) ? 0 : (uint8_t)((tight + 5) / 10 < 1 ? 1 : ((tight + 5) / 10 > 90 ? 90 : (tight + 5) / 10));
}


// 1.8: sensor and matrix settings.  Written as soon as they change (not by [sav]), so they survive a restart
// without touching the SMC3 tuning; boards that never had them keep the defaults.
static inline void EEPromPut16(int a, int16_t v) { EEPROM.update(a, (uint8_t)((uint16_t)v >> 8)); EEPROM.update(a + 1, (uint8_t)((uint16_t)v & 0xFF)); }
static inline int16_t EEPromGet16(int a) { return (int16_t)(((uint16_t)EEPROM.read(a) << 8) | EEPROM.read(a + 1)); }

void WriteBoardSettings()
{
    EEPROM.update(57, LedView);
    EEPROM.update(58, LedFlags);
    EEPROM.update(59, SensorFlags);
    EEPROM.update(60, TiltLimitDeg);
    EEPromPut16(61, LevelRoll_x100);
    EEPromPut16(63, LevelPitch_x100);
#if IMU_PORT
    EEPROM.update(65, ImuBaudIdx);
#endif
    EEPROM.update(56, 0xA6);
}

void ReadBoardSettings()
{
    if (EEPROM.read(56) != 0xA6) { return; }
    uint8_t v = EEPROM.read(57);
    LedView = (v <= 2) ? v : LED_DEFAULT_VIEW;
    LedFlags = EEPROM.read(58);
    SensorFlags = EEPROM.read(59) & 0x0F;
    v = EEPROM.read(60);
    TiltLimitDeg = (v <= 60) ? v : 0;
    LevelRoll_x100 = EEPromGet16(61);
    LevelPitch_x100 = EEPromGet16(63);
#if IMU_PORT
    v = EEPROM.read(65);
    ImuBaudIdx = (v < IMU_NBAUDS) ? v : 0;
#endif
}

// Seat set-up in EEPROM 66-104: 66 = 0xB9 "valid", then per motor trim, travel low/high, end stop low/high (int16,
// 67 + 10 * motor), then the four tilt limits (int16, 97-104).
#define SEAT_EE 66

void WriteSeatSetup()
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        int a = SEAT_EE + 1 + 10 * m;
        EEPromPut16(a, Trim[m]);
        EEPromPut16(a + 2, TravelLo[m]);
        EEPromPut16(a + 4, TravelHi[m]);
        EEPromPut16(a + 6, EndLo[m]);
        EEPromPut16(a + 8, EndHi[m]);
    }
    for (uint8_t d = 0; d < 4; d++) { EEPromPut16(SEAT_EE + 31 + 2 * d, TiltLim_x10[d]); }
    EEPROM.update(SEAT_EE, 0xB9);
}

// 1.9.1: soft-start speed in EEPROM 105-107: 105 = 0xB5 "valid", 106-107 = counts/s (big-endian).
void WriteSoftStart()
{
    EEPromPut16(SOFT_EE + 1, (int16_t)SoftRate);
    EEPROM.update(SOFT_EE, 0xB5);
}

void ReadSoftStart()
{
    if (EEPROM.read(SOFT_EE) != 0xB5) { return; }   // never set: the default
    int16_t v = EEPromGet16(SOFT_EE + 1);
    if (v >= 0 && v <= 1000) { SoftRate = (unsigned int)v; }
}

bool SeatSetupSaved()                            // what runs is what is saved
{
    if (EEPROM.read(SEAT_EE) != 0xB9)
    {
        for (uint8_t m = 0; m < NUM_MOTORS; m++)
        {
            if (Trim[m] != 0 || TravelLo[m] != 0 || TravelHi[m] != 1023 || EndLo[m] != 0 || EndHi[m] != 1023) { return false; }
        }
        return true;                             // nothing saved and nothing set: same thing (tilt lives in 1.8's byte)
    }
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        int a = SEAT_EE + 1 + 10 * m;
        if (EEPromGet16(a) != Trim[m] || EEPromGet16(a + 2) != TravelLo[m] || EEPromGet16(a + 4) != TravelHi[m]
            || EEPromGet16(a + 6) != EndLo[m] || EEPromGet16(a + 8) != EndHi[m]) { return false; }
    }
    for (uint8_t d = 0; d < 4; d++) { if (EEPromGet16(SEAT_EE + 31 + 2 * d) != TiltLim_x10[d]) { return false; } }
    return true;
}

void ClearSeatSetup()
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        Trim[m] = 0; TravelLo[m] = 0; TravelHi[m] = 1023; EndLo[m] = 0; EndHi[m] = 1023;
    }
}

void ReadSeatSetup()                             // after ReadBoardSettings(): 1.8's tilt limit is the fallback
{
    ClearSeatSetup();
    for (uint8_t d = 0; d < 4; d++) { TiltLim_x10[d] = (int16_t)TiltLimitDeg * 10; }
    if (EEPROM.read(SEAT_EE) == 0xB9)
    {
        for (uint8_t m = 0; m < NUM_MOTORS; m++)
        {
            int a = SEAT_EE + 1 + 10 * m;
            int16_t t = EEPromGet16(a);
            Trim[m] = (t >= -300 && t <= 300) ? t : 0;
            int16_t v;
            v = EEPromGet16(a + 2); TravelLo[m] = (v >= 0 && v <= 1023) ? v : 0;
            v = EEPromGet16(a + 4); TravelHi[m] = (v >= 0 && v <= 1023) ? v : 1023;
            v = EEPromGet16(a + 6); EndLo[m]    = (v >= 0 && v <= 1023) ? v : 0;
            v = EEPromGet16(a + 8); EndHi[m]    = (v >= 0 && v <= 1023) ? v : 1023;
        }
        for (uint8_t d = 0; d < 4; d++)
        {
            int16_t v = EEPromGet16(SEAT_EE + 31 + 2 * d);
            TiltLim_x10[d] = (v >= 0 && v <= 900) ? v : 0;
        }
    }
    UpdateLimits();
}

void UpdateSeatAngles();

// [k f c]
void SeatCommand(uint8_t f, uint8_t c)
{
    if (c == '=')
    {
        int16_t v = ValueReg;
        if (f >= 64 && f < 68)
        {
            if (v >= 0 && v <= 900) { TiltLim_x10[f - 64] = v; }
        }
        else if (f == 82)                        // 1.9.1: soft-start speed, counts/s (0 = off), saved at once
        {
            if (v >= 0 && v <= 1000)
            {
                SoftRate = (unsigned int)v;
                UpdateSmoothingRate();
                WriteSoftStart();
            }
            return;
        }
        else if (f == 80 || f == 81)             // the sensor's level (raw angles x100 that count as level), saved at once
        {
            if (v >= -9000 && v <= 9000)
            {
                if (f == 80) { LevelRoll_x100 = v; } else { LevelPitch_x100 = v; }
                UpdateSeatAngles();
                WriteBoardSettings();
            }
            return;
        }
        else if ((f >> 4) < NUM_MOTORS)
        {
            uint8_t m = f >> 4;
            switch (f & 15)
            {
                case 0: if (v >= -300 && v <= 300) { Trim[m] = v; } break;
                case 1: if (v >= 0 && v <= 1023) { TravelLo[m] = v; } break;
                case 2: if (v >= 0 && v <= 1023) { TravelHi[m] = v; } break;
                case 3: if (v >= 0 && v <= 1023) { EndLo[m] = v; } break;
                case 4: if (v >= 0 && v <= 1023) { EndHi[m] = v; } break;
                default: break;
            }
        }
        UpdateLimits();
    }
    else if (f == 'S' && c == 'V') { WriteSeatSetup(); }
    else if (f == 'R' && c == 'L') { ReadSeatSetup(); }
    else if (f == 'C' && c == 'L') { ClearSeatSetup(); UpdateLimits(); }
    else if (f == 'H')                           // hold: c x 100 ms (0 = give the seat back to SimHub now)
    {
        PageHold = (c != 0);
        PageHoldUntil = millis() + (unsigned long)c * 100UL;
    }
}

//****************************************************************************************************************
//    Serial replies (byte format unchanged)
//****************************************************************************************************************

void ReplyWrite(const uint8_t *buf, size_t n)
{
#if ENABLE_WIFI
    if (ReplyTarget == 1)
    {
        if (ClientUp) { TcpClient.write(buf, n); }
        return;
    }
#endif
    Serial.write(buf, n);
}

void SendTwoValues(int id, int v1, int v2)
{
    uint8_t p[5] = {(uint8_t)START_BYTE, (uint8_t)id, (uint8_t)v1, (uint8_t)v2, (uint8_t)END_BYTE};
    ReplyWrite(p, 5);
}

void SendValue(int id, int value)
{
    int high = value / 256;
    int low  = value - (high * 256);
    uint8_t p[5] = {(uint8_t)START_BYTE, (uint8_t)id, (uint8_t)high, (uint8_t)low, (uint8_t)END_BYTE};
    ReplyWrite(p, 5);
}

int DeltaLoopCount()
{
    unsigned long now = LoopCount;
    unsigned long Delta;
    if ((LastCount == 0) || ((now - LastCount) > 32000))
    {
        Delta = 0;
    }
    else
    {
        Delta = now - LastCount;
    }
    LastCount = now;
    return (int)Delta;
}

// Position / goal as SMC3Utils expects them: 0-255 (0-1023 / 4)
static inline int ReportFeedback(uint8_t m)
{
    int v = Feedback_q[m] >> POS_SHIFT;
#ifdef REVERSE_MOTOR1
    if (m == 0) { v = 1023 - v; }
#endif
    return constrain(v, 0, 1023) / 4;
}

static inline int ReportGoal(uint8_t m)
{
    int v = Goal_q[m] >> POS_SHIFT;
#ifdef REVERSE_MOTOR1
    if (m == 0) { v = 1023 - v; }
#endif
    return constrain(v, 0, 1023) / 4;
}

static inline int ReportPWM(uint8_t m)
{
    return constrain(PWMout_q[m] >> PWM_FRAC, 0, 255);
}

//****************************************************************************************************************
//    Command parser (semantics unchanged from the original)
//****************************************************************************************************************

void SetTarget(uint8_t m, int value)
{
#ifdef REVERSE_MOTOR1
    if (m == 0) { value = 1023 - value; }
#endif
    value += Trim[m];                            // 1.9: level trim, then SMC3's clip and the travel limits
    if (value > TargetHi[m]) { value = TargetHi[m]; }
    else if (value < TargetLo[m]) { value = TargetLo[m]; }
    Target[m] = value;
}

static inline bool PageHolds()                   // 1.9: the page has the seat ([kH n]); SimHub's positions wait
{
    if (PageHold && (long)(PageHoldUntil - millis()) <= 0) { PageHold = false; }
    return PageHold;
}

#if ENABLE_WIFI
void SendDiag();
void SendOtaFrame();
void OtaCommand(uint8_t a, uint8_t b);
#endif
void ImuCommand(uint8_t a, uint8_t b);
static inline uint8_t SensorState();
void UsbPageCommand(uint8_t n);

// source 0 = USB (SimHub / SMC3Utils), 1 = TCP (smc3_scope.py).  Same command set on both.
// "Host alive" for the comms-timeout power reduction = any USB packet, or a position command over TCP
// (reading telemetry alone must not keep the rig at full power).
// 1.9.5: in USB page mode the USB commands are the page's: counted as the page's, never held for the seat set-up, and
// (as over WiFi) only its position commands keep the rig at full power.
unsigned long TcpLastTargetMillis = 0;

void ParseCommand(const unsigned int *buf, uint8_t source)
{
    bool isTarget = (buf[0] == 'A' || buf[0] == 'B' || buf[0] == 'C');
    bool usbPage = (source == 0 && UsbPage);
    bool fromPage = (source == 1 || usbPage);
    if ((source == 0 && !usbPage) || isTarget) { LastPacketMillis = millis(); }
    if (isTarget) { LastPosMillis = millis(); }
    if (fromPage && isTarget) { TcpLastTargetMillis = millis(); }
    if (source == 0 && !usbPage)
    {
        UsbFramesOk++;
        LastUsbPacketMillis = millis();
        if (isTarget) { UsbTargetFrames++; LastUsbTargetMillis = LastUsbPacketMillis; }
    }
    else
    {
        TcpFramesOk++;
    }
    (void)fromPage;                              // (only used with WiFi)
    ReplyTarget = source;
    unsigned int b1 = buf[1];
    unsigned int b2 = buf[2];
    int word = (int)((b1 * 256) + b2);

    if (isTarget && source == 0 && !usbPage && PageHolds())  // Seat setup is moving the seat: SimHub waits (1.9)
    {
        UsbHeldFrames++;
        ReplyTarget = 0;
        return;
    }

    switch (buf[0])
    {
        case 'A': SetTarget(0, word); break;
        case 'B': SetTarget(1, word); break;
        case 'C': SetTarget(2, word); break;

        case 'r':
            if (b1 == 'd')
            {
                switch (b2)
                {
                    case 'A': SendTwoValues('A', ReportFeedback(0), ReportGoal(0)); break;
                    case 'B': SendTwoValues('B', ReportFeedback(1), ReportGoal(1)); break;
                    case 'C': SendTwoValues('C', ReportFeedback(2), ReportGoal(2)); break;
                    case 'a': SendTwoValues('a', StatusByte(), ReportPWM(0)); break;
                    case 'b': SendTwoValues('b', StatusByte(), ReportPWM(1)); break;
                    case 'c': SendTwoValues('c', StatusByte(), ReportPWM(2)); break;
                    case 'D': SendValue('D', Kp_x100[0]); break;
                    case 'E': SendValue('E', Kp_x100[1]); break;
                    case 'F': SendValue('F', Kp_x100[2]); break;
                    case 'G': SendValue('G', Ki_x100[0]); break;
                    case 'H': SendValue('H', Ki_x100[1]); break;
                    case 'I': SendValue('I', Ki_x100[2]); break;
                    case 'J': SendValue('J', Kd_x100[0]); break;
                    case 'K': SendValue('K', Kd_x100[1]); break;
                    case 'L': SendValue('L', Kd_x100[2]); break;
                    case 'M': SendValue('M', Ks[0]); break;
                    case 'N': SendValue('N', Ks[1]); break;
                    case 'O': SendValue('O', Ks[2]); break;
                    case 'P': SendTwoValues('P', PWMoffset[0], PWMmax[0]); break;
                    case 'Q': SendTwoValues('Q', PWMoffset[1], PWMmax[1]); break;
                    case 'R': SendTwoValues('R', PWMoffset[2], PWMmax[2]); break;
                    case 'S': SendTwoValues('S', CutoffLimitMin[0], InputClipMin[0]); break;
                    case 'T': SendTwoValues('T', CutoffLimitMin[1], InputClipMin[1]); break;
                    case 'U': SendTwoValues('U', CutoffLimitMin[2], InputClipMin[2]); break;
                    case 'V': SendTwoValues('V', DeadZone[0], PWMrev[0]); break;
                    case 'W': SendTwoValues('W', DeadZone[1], PWMrev[1]); break;
                    case 'X': SendTwoValues('X', DeadZone[2], PWMrev[2]); break;
                    case 'Y': SendTwoValues('Y', StatusByte(), 0); break;
                    case 'Z': SendValue('Z', DeltaLoopCount()); break;
                    case '~': SendTwoValues('~', Timer1FreqkHz, Timer2FreqkHz); break;
                    case '$':                                              // new: worst ISR time in microseconds
                    {
                        uint32_t us = IsrCyclesMax / (F_CPU / 1000000UL);
                        IsrCyclesMax = 0;
                        SendValue('$', (int)constrain(us, 0UL, 65535UL));
                        break;
                    }
                    case 'f': SendTwoValues('f', SmoothShift, MaxRate / 100); break;   // 1.6: smoothing, max speed
                    case '!': SendTwoValues('!', BuildFlags(), FW_VERSION_NUM); break; // 1.4: build flags + version
                    case 'l': SendTwoValues('l', LedView, LedFlags); break;            // 1.8: matrix view + flags
                    case 'i': SendTwoValues('i', SensorState(), TiltLimitDeg); break;  // 1.8: sensor state + tilt limit
#if ENABLE_WIFI
                    case 'd': if (fromPage) { SendDiag(); } break;         // 1.7: diagnostics frame (the page only)
                    case 'u': if (fromPage) { SendOtaFrame(); } break;     // 1.9.2: update state + version text
#endif
                    default: break;
                }
            }
            break;

        case 'D': Kp_x100[0] = word; break;
        case 'E': Kp_x100[1] = word; break;
        case 'F': Kp_x100[2] = word; break;
        case 'G': Ki_x100[0] = word; UpdatePIDLimits(); break;
        case 'H': Ki_x100[1] = word; UpdatePIDLimits(); break;
        case 'I': Ki_x100[2] = word; UpdatePIDLimits(); break;
        case 'J': Kd_x100[0] = word; break;
        case 'K': Kd_x100[1] = word; break;
        case 'L': Kd_x100[2] = word; break;
        case 'M': Ks[0] = word; break;
        case 'N': Ks[1] = word; break;
        case 'O': Ks[2] = word; break;
        case 'P': PWMoffset[0] = b1; PWMmax[0] = b2; break;
        case 'Q': PWMoffset[1] = b1; PWMmax[1] = b2; break;
        case 'R': PWMoffset[2] = b1; PWMmax[2] = b2; break;
        case 'S': CutoffLimitMin[0] = b1; CutoffLimitMax[0] = 1023 - b1; InputClipMin[0] = b2; InputClipMax[0] = 1023 - b2; UpdateLimits(); break;
        case 'T': CutoffLimitMin[1] = b1; CutoffLimitMax[1] = 1023 - b1; InputClipMin[1] = b2; InputClipMax[1] = 1023 - b2; UpdateLimits(); break;
        case 'U': CutoffLimitMin[2] = b1; CutoffLimitMax[2] = 1023 - b1; InputClipMin[2] = b2; InputClipMax[2] = 1023 - b2; UpdateLimits(); break;
        case 'V': DeadZone[0] = b1; PWMrev[0] = b2; break;
        case 'W': DeadZone[1] = b1; PWMrev[1] = b2; break;
        case 'X': DeadZone[2] = b1; PWMrev[2] = b2; break;
        case 'Z': PIDProcessDivider = constrain((int)b1, 1, 10); UpdateSmoothingRate(); break;
        case 'f': SmoothShift = (uint8_t)((b1 > 8) ? 8 : b1); MaxRate = b2 * 100; UpdateSmoothingRate(); break;   // 1.6
        case '~':
            if (b1 >= 1 && b1 <= 100) { Timer1FreqkHz = b1; }
            if (b2 >= 1 && b2 <= 100) { Timer2FreqkHz = b2; }
            ApplyPwmFrequencies();
            break;

        case 'l':                                                // 1.8: matrix view and flags, saved at once
            if (b1 <= 2)
            {
                LedView = b1;
                LedFlags = b2;
                WriteBoardSettings();
            }
            break;
        case 'i':                                                // 1.8: motion sensor
            ImuCommand(b1, b2);
            break;
        case 'w': ValueReg = (int16_t)(uint16_t)word; break;     // 1.9: value register for [k]
        case 'k':                                                // 1.9: seat set-up
            if (b1 == 'U') { if (source == 0) { UsbPageCommand((uint8_t)b2); } }   // 1.9.5: page on the USB cable
            else { SeatCommand((uint8_t)b1, (uint8_t)b2); }
            break;
#if ENABLE_WIFI
        case 'x':                                                // 1.9: sensor acceleration + gyro frames
            if (b1 == 'e' && fromPage) { TelExtras = (b2 == '1' || b2 == 1); }
            break;
        case 'u':                                                // 1.9.2: firmware update (from the page only)
            if (source == 1) { OtaCommand((uint8_t)b1, (uint8_t)b2); }
            break;
#endif
        case 'm':
            if (b1 == 'o')
            {
                if (b2 >= '0' && b2 <= '3') { SerialFeedbackEnabled = b2 - '0'; }
            }
            break;
        case 's':
            if (b1 == 'a' && b2 == 'v') { WriteEEProm(); WriteSeatSetup(); }   // 1.9: the seat set-up too
            break;
        case 'v':
            if (b1 == 'e' && b2 == 'r') { SendValue('v', 70); }   // reports as v0.70 so SMC3Utils accepts it
            break;
        case 'd':
            if (b1 == 'i')                                       // new in 1.5: software stop
            {
                if (b2 == 's')
                {
                    for (uint8_t m = 0; m < NUM_MOTORS; m++) { DisableReason[m] = DIS_COMMAND; DisableMotor(m); }
                }
                else if (b2 >= '1' && b2 <= '3')
                {
                    DisableReason[b2 - '1'] = DIS_COMMAND;
                    DisableMotor(b2 - '1');
                }
            }
            break;
        case 'e':
            if (b1 == 'n')
            {
                if (b2 == 'a') { EnableMotor(0); EnableMotor(1); EnableMotor(2); }
                else if (b2 == '1') { EnableMotor(0); }
                else if (b2 == '2') { EnableMotor(1); }
                else if (b2 == '3') { EnableMotor(2); }
            }
            break;
        default: break;
    }
    ReplyTarget = 0;
}

void CheckSerial()
{
    while (Serial.available())
    {
        if (BufferEnd == -1)
        {
            RxByte = Serial.read();
            if (RxByte != START_BYTE) { errorcount++; UsbBadBytes++; } else { BufferEnd = 0; }
        }
        else
        {
            RxByte = Serial.read();
            RxBuffer[BufferEnd] = RxByte;
            BufferEnd++;
            if (BufferEnd > 3)
            {
                if (RxBuffer[3] == END_BYTE) { ParseCommand(RxBuffer, 0); } else { errorcount++; UsbBadBytes += 5; }
                BufferEnd = -1;
            }
        }
    }
}

void SendMonitorData()
{
    if (SerialFeedbackEnabled < 1 || SerialFeedbackEnabled > 3) { return; }
    uint8_t m = SerialFeedbackEnabled - 1;
    SendTwoValues('A' + m, ReportFeedback(m), ReportGoal(m));
    SendTwoValues('a' + m, StatusByte(), ReportPWM(m));
}

//****************************************************************************************************************
//    WitMotion WT901C motion sensor (Serial1, 1.8).  11-byte packets: 0x55, type, 8 data bytes, checksum (sum of
//    the first 10).  Type 0x53 = angles: roll, pitch, yaw as int16 / 32768 * 180 degrees.  1.9 also reads 0x51 =
//    acceleration (/ 32768 * 16 g) + temperature (/ 100 C) and 0x52 = angular rate (/ 32768 * 2000 deg/s).  The board tries the
//    usual baud rates until packets with good checksums arrive.  Set-up commands are FF AA reg lo hi, after an
//    unlock (FF AA 69 88 B5), at least 100 ms apart.
//****************************************************************************************************************

static inline uint8_t SensorState()       // [rdi] byte 1
{
    uint8_t v = SensorFlags & 0x0F;
    if (SensorFresh()) { v |= 0x10; }
    if (ImuLastValid != 0 && (millis() - ImuLastValid) < 3000) { v |= 0x20; }
#if IMU_PORT
    if (ImuCmdN > 0) { v |= 0x40; }
#endif
    unsigned long now = millis();                // 1.9: acceleration and gyro arriving too
    if (ImuAccMillis != 0 && ImuGyroMillis != 0 && (now - ImuAccMillis) < 500 && (now - ImuGyroMillis) < 500) { v |= 0x80; }
    return v;
}

static inline float Wrap180(float a)
{
    while (a >= 180.0f) { a -= 360.0f; }
    while (a < -180.0f) { a += 360.0f; }
    return a;
}

void UpdateSeatAngles()
{
    float r = Wrap180(ImuRawRoll - LevelRoll_x100 / 100.0f);
    float p = Wrap180(ImuRawPitch - LevelPitch_x100 / 100.0f);
    if (SensorFlags & 2) { float t = r; r = p; p = t; }
    if (SensorFlags & 4) { r = -r; }
    if (SensorFlags & 8) { p = -p; }
    SeatRoll = r;
    SeatPitch = p;
}

// Tilt limits: three angle packets in a row past a limit stop every motor (E4).  Enable motors again from the
// page (or [ena]) once the cause is fixed; the check pauses 1.5 s after enabling so the seat can come back.
// 1.9: a limit for each direction (0 = none) - roll right/left, front up/down.
void CheckTilt(unsigned long now)
{
    bool any = TiltLim_x10[0] || TiltLim_x10[1] || TiltLim_x10[2] || TiltLim_x10[3];
    if (AnySoftStart()) { TiltGraceUntil = now + 1500; }   // 1.9.1: the seat is still creeping back - wait for it
    if (!(SensorFlags & 1) || !any || (long)(now - TiltGraceUntil) < 0) { TiltHits = 0; return; }
    bool past = (TiltLim_x10[0] && SeatRoll  >  TiltLim_x10[0] * 0.1f) || (TiltLim_x10[1] && SeatRoll  < -TiltLim_x10[1] * 0.1f)
             || (TiltLim_x10[2] && SeatPitch >  TiltLim_x10[2] * 0.1f) || (TiltLim_x10[3] && SeatPitch < -TiltLim_x10[3] * 0.1f);
    if (!past) { TiltHits = 0; return; }
    if (++TiltHits < 3) { return; }
    TiltHits = 0;
    bool hit = false;
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        if (Disable[m] == 0) { hit = true; DisableReason[m] = DIS_TILT; DisableMotor(m); }
    }
    if (hit && TiltTrips < 65535) { TiltTrips++; }
}

#if IMU_PORT
void ImuSetBaud(uint8_t idx)
{
    ImuBaudIdx = idx % IMU_NBAUDS;
    Serial1.end();
    Serial1.begin(ImuBauds[ImuBaudIdx]);
    RaiseSerialRxPriority();                     // 1.9.6
    ImuBaudSince = millis();
}

void PollImu()
{
    static uint8_t buf[11];
    static uint8_t n = 0;
    static uint8_t goodRun = 0;                 // good packets in a row: one alone could be noise at the wrong baud
    int budget = 128;                           // never spend long here, whatever arrives

    while (Serial1.available() && budget-- > 0)
    {
        uint8_t c = Serial1.read();
        if (n == 0)
        {
            if (c == 0x55) { buf[n++] = c; }
            continue;
        }
        buf[n++] = c;
        if (n == 2 && (c < 0x50 || c > 0x5F)) { n = (c == 0x55) ? 1 : 0; continue; }   // not a packet type: resync
        if (n < 11) { continue; }
        n = 0;

        uint8_t sum = 0;
        for (uint8_t i = 0; i < 10; i++) { sum += buf[i]; }
        if (sum != buf[10]) { if (ImuBadSums < 65535) { ImuBadSums++; } goodRun = 0; continue; }
        if (goodRun < 255) { goodRun++; }
        if (goodRun < 2) { continue; }

        unsigned long now = millis();
        ImuLastValid = now;
        ImuPackets++;
        if (buf[1] == 0x53)
        {
            int16_t r = (int16_t)((buf[3] << 8) | buf[2]);
            int16_t p = (int16_t)((buf[5] << 8) | buf[4]);
            int16_t y = (int16_t)((buf[7] << 8) | buf[6]);
            ImuRawRoll  = r / 32768.0f * 180.0f;
            ImuRawPitch = p / 32768.0f * 180.0f;
            ImuYaw      = y / 32768.0f * 180.0f;
            ImuLastMillis = now;
            ImuAngleCount++;
            UpdateSeatAngles();
            CheckTilt(now);
        }
        else if (buf[1] == 0x51)                // 1.9: acceleration (x, y, z / 32768 * 16 g) and temperature
        {
            for (uint8_t i = 0; i < 3; i++) { ImuAcc[i] = (int16_t)((buf[3 + 2 * i] << 8) | buf[2 + 2 * i]); }
            ImuTemp_x100 = (int16_t)((buf[9] << 8) | buf[8]);
            ImuAccMillis = now;
            ImuAccCount++;
            float ax = ImuAcc[0], ay = ImuAcc[1], az = ImuAcc[2];
            float dev = sqrtf(ax * ax + ay * ay + az * az) * (1000.0f / 2048.0f) - 1000.0f;   // 2048 = 1 g
            if (dev < 0) { dev = -dev; }
            uint16_t mg = (dev > 65000.0f) ? 65000 : (uint16_t)dev;
            if (mg > AccPeak_mg) { AccPeak_mg = mg; }
            if (AccBurstN < 255) { AccBurstN++; }
        }
        else if (buf[1] == 0x52)                // 1.9: angular rate (x, y, z / 32768 * 2000 deg/s)
        {
            for (uint8_t i = 0; i < 3; i++) { ImuGyro[i] = (int16_t)((buf[3 + 2 * i] << 8) | buf[2 + 2 * i]); }
            ImuGyroMillis = now;
            ImuGyroCount++;
        }
    }
}

void ImuQueue(uint8_t reg, uint16_t val)
{
    if (ImuCmdN >= 10) { return; }
    uint8_t *c = ImuCmd[ImuCmdN++];
    c[0] = 0xFF; c[1] = 0xAA; c[2] = reg; c[3] = (uint8_t)(val & 0xFF); c[4] = (uint8_t)(val >> 8);
}

// Output angles only (1100 bytes/s at 100 Hz), or - 1.9, for smc3_scope's 3D view - angles, gyro and acceleration
// (3300 bytes/s: a 150 ms hold-up of loop() by a WiFi call still fits the 512-byte receive buffer; a longer one
// costs a few packets, nothing else).  rateCode (WitMotion: 0x06 10 Hz, 0x08 50 Hz, 0x09 100 Hz, 0x0B 200 Hz),
// 115200 baud, saved in the sensor.  Needs D1 -> MAX3232 -> sensor RX.  Saved at the old baud and again at the
// new one, because sensors differ in when the new baud rate takes effect.
void ImuStartSetup(uint8_t rateCode, bool all)
{
    if (ImuCmdN > 0) { return; }
    if (rateCode < 0x01 || rateCode > 0x0B) { rateCode = 0x09; }
    ImuCmdPos = 0;
    ImuQueue(0x69, 0xB588);                     // unlock
    ImuQueue(0x02, all ? 0x000E : 0x0008);      // output: angles (0x53), + acceleration (0x51) and gyro (0x52)
    ImuQueue(0x03, rateCode);                   // output rate
    ImuQueue(0x00, 0x0000);                     // save
    ImuQueue(0x69, 0xB588);
    ImuQueue(0x04, 0x0006);                     // 115200 baud
    ImuQueue(0x00, 0x0000);                     // save (sensors that switch on save)
    ImuCmdBaudSwitchAt = (int8_t)ImuCmdN;
    ImuQueue(0x69, 0xB588);                     // ... and again at 115200 (sensors that switch at once)
    ImuQueue(0x00, 0x0000);
    ImuCmdLast = millis();
}

void ServiceImu(unsigned long now)
{
    if (now - ImuRateSince >= 1000)
    {
        ImuRateSince = now;
        ImuRateHz = ImuAngleCount;
        ImuAngleCount = 0;
        ImuAccHz = ImuAccCount;   ImuAccCount = 0;
        ImuGyroHz = ImuGyroCount; ImuGyroCount = 0;
    }

    if (ImuCmdN > 0)                            // set-up in progress
    {
        if (now - ImuCmdLast >= 120)
        {
            if ((int8_t)ImuCmdPos == ImuCmdBaudSwitchAt) { ImuSetBaud(IMU_FAST_BAUD_IDX); }
            Serial1.write(ImuCmd[ImuCmdPos], 5);
            ImuCmdPos++;
            ImuCmdLast = now;
            if (ImuCmdPos >= ImuCmdN)
            {
                ImuCmdN = 0;
                ImuCmdPos = 0;
                ImuCmdBaudSwitchAt = -1;
                ImuScanUntil = now + 30000;     // if it didn't take, find the sensor again wherever it is
                ImuBaudSince = now;
            }
        }
        return;
    }

    bool locked = ImuLastValid != 0 && (now - ImuLastValid) < 3000 && (long)(ImuLastValid - ImuBaudSince) > 0;
    if (locked)
    {
        static uint8_t savedIdx = 255;
        if (savedIdx != ImuBaudIdx && EEPROM.read(65) != ImuBaudIdx && EEPROM.read(56) == 0xA6)
        {
            EEPROM.update(65, ImuBaudIdx);      // start at this baud rate next time
        }
        savedIdx = ImuBaudIdx;
        return;
    }
    // Not receiving: try the next baud rate - while the sensor is switched on, for the first minute after
    // power-up, or for 30 s after the page asked.
    bool scan = (SensorFlags & 1) || now < 60000UL || (long)(ImuScanUntil - now) > 0;
    if (scan && now - ImuBaudSince >= 1500) { ImuSetBaud(ImuBaudIdx + 1); }
}
#endif // IMU_PORT

// [i a b]: a < 16 -> sensor flags a and tilt limit b (degrees, 0 = off).  a = 'L' level now, 'C' clear the
// level, 'S' set the sensor to rate code b (0 = 100 Hz) at 115200 baud, 'R' search for the sensor again.
void ImuCommand(uint8_t a, uint8_t b)
{
    if (a < 16)
    {
        SensorFlags = a;
        if (b != 255)                            // 1.8's single limit: all four directions (255 = leave them, 1.9)
        {
            uint8_t t = (b <= 60) ? b : 60;
            for (uint8_t d = 0; d < 4; d++) { TiltLim_x10[d] = (int16_t)t * 10; }
            UpdateLimits();
            if (EEPROM.read(SEAT_EE) == 0xB9) { WriteSeatSetup(); }
        }
        UpdateSeatAngles();
        WriteBoardSettings();
    }
    else if (a == 'L')
    {
        if (!SensorFresh()) { return; }
        LevelRoll_x100 = (int16_t)(ImuRawRoll * 100.0f);
        LevelPitch_x100 = (int16_t)(ImuRawPitch * 100.0f);
        UpdateSeatAngles();
        WriteBoardSettings();
    }
    else if (a == 'C')
    {
        LevelRoll_x100 = 0;
        LevelPitch_x100 = 0;
        UpdateSeatAngles();
        WriteBoardSettings();
    }
#if IMU_PORT
    else if (a == 'S')
    {
        // b = 0: 100 Hz, angles only (1.8).  1.9: low nibble = WitMotion rate code, bit6 = angles + gyro + acceleration
        if (b == 0) { ImuStartSetup(0x09, false); }
        else        { ImuStartSetup((b & 0x0F) ? (b & 0x0F) : 0x09, (b & 0x40) != 0); }
    }
    else if (a == 'R')
    {
        ImuScanUntil = millis() + 30000;        // a sensor that is already found stays found
    }
#endif
}

//****************************************************************************************************************
//    WiFi: TCP server for telemetry out / SMC3 commands in
//
//    Telemetry frame (little-endian):  'S' 'M' '3' 'T', uint8 version(1), uint8 sampleCount, then per sample:
//      uint32 millis
//      3 x { int16 target(0-1023), int16 goal(Q4), int16 feedback(Q4), int16 pid(Q4, signed, before PWMmin) }
//      uint8 statusByte, uint8 powerScale, int16 roll*100, int16 pitch*100, int16 yaw*100 (seat angles from the
//      motion sensor, levelled and turned to its mounting; -32768 = no data), uint16 isrMaxMicros
//    = 38 bytes per sample.  Commands from the client use the exact SMC3 "[xyz]" format; replies to [rd?]
//    go back over the same connection.
//    1.9, after [xe1] only: every burst is followed by 'S' 'M' '3' 'X', uint8 version (1), uint8 length (18), then
//      int16 acceleration x, y, z (raw, / 32768 * 16 g), int16 gyro x, y, z (raw, / 32768 * 2000 deg/s),
//      int16 temperature x100, uint16 biggest |acceleration| - 1 g since the last burst (milli-g),
//      uint8 flags (bit0 acceleration fresh, bit1 gyro fresh), uint8 acceleration packets since the last burst.
//****************************************************************************************************************

#if ENABLE_WIFI

static inline void Put16(uint8_t *p, int v)  { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); }
static inline void Put32(uint8_t *p, uint32_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF; }

bool ReadIpText();

// ---- WiFi memory (1.9.2) ----

static inline bool SketchHasWifi()             // the sketch has a real network name, not the placeholder
{
    return WIFI_SSID[0] != 0 && strcmp(WIFI_SSID, "YourNetworkName") != 0;
}

void ReadWifiMemory()
{
    WifiMem = false;
    if (EEPROM.read(WIFI_EE) != 0xC7) { return; }
    uint8_t n = EEPROM.read(WIFI_EE + 1), k = EEPROM.read(WIFI_EE_PASS);
    if (n == 0 || n > 32 || k > 64) { return; }
    for (uint8_t i = 0; i < n; i++) { WifiMemSsid[i] = (char)EEPROM.read(WIFI_EE + 2 + i); }
    WifiMemSsid[n] = 0;
    for (uint8_t i = 0; i < k; i++) { WifiMemPass[i] = (char)EEPROM.read(WIFI_EE_PASS + 1 + i); }
    WifiMemPass[k] = 0;
    WifiMem = true;
}

void WriteWifiMemory(const char *ssid, const char *pass)
{
    size_t n = strlen(ssid), k = strlen(pass);
    if (n == 0 || n > 32 || k > 64) { return; }
    if (WifiMem && strcmp(ssid, WifiMemSsid) == 0 && strcmp(pass, WifiMemPass) == 0) { return; }   // nothing new
    EEPROM.update(WIFI_EE, 0);                  // not valid while it is being written
    EEPROM.update(WIFI_EE + 1, (uint8_t)n);
    for (size_t i = 0; i < n; i++) { EEPROM.update(WIFI_EE + 2 + (int)i, (uint8_t)ssid[i]); }
    EEPROM.update(WIFI_EE_PASS, (uint8_t)k);
    for (size_t i = 0; i < k; i++) { EEPROM.update(WIFI_EE_PASS + 1 + (int)i, (uint8_t)pass[i]); }
    EEPROM.update(WIFI_EE, 0xC7);
    strcpy(WifiMemSsid, ssid);
    strcpy(WifiMemPass, pass);
    WifiMem = true;
}

uint32_t ParseWifiFw(const char *s)            // "0.5.2" -> 0x000502; 0 = unknown ("99.99.99" = no answer)
{
    uint32_t part[3] = {0, 0, 0};
    uint8_t i = 0;
    bool digit = false;
    for (; s && *s && i < 3; s++)
    {
        if (*s >= '0' && *s <= '9') { part[i] = part[i] * 10 + (uint32_t)(*s - '0'); digit = true; if (part[i] > 255) { return 0; } }
        else if (*s == '.') { i++; }
        else { break; }
    }
    if (!digit || part[0] >= 99) { return 0; }
    return (part[0] << 16) | (part[1] << 8) | part[2];
}

bool ConnectWifi()
{
    WifiLastAttempt = millis();
    if (WiFi.status() == WL_NO_MODULE)
    {
        strcpy(WifiText, "E0");
        WifiUp = false;
        return false;
    }
    if (WifiFw == 0) { WifiFw = ParseWifiFw(WiFi.firmwareVersion()); }   // 1.9.2: for the firmware update
    WiFi.setHostname(WIFI_HOSTNAME);
#if WIFI_STATIC_IP
    WiFi.config(IPAddress(WIFI_IP), IPAddress(WIFI_DNS), IPAddress(WIFI_GATEWAY), IPAddress(WIFI_SUBNET));
#endif
    WiFi.setTimeout(WIFI_CONNECT_TIMEOUT_MS);
    // 1.9.2: the sketch's name and password first (remembered once they work), then the remembered ones - so a build
    // that still has the placeholders, or an old password, keeps the board on the network
    int st = WL_CONNECT_FAILED;
    bool sketch = SketchHasWifi();
    if (sketch)
    {
        st = WiFi.begin(WIFI_SSID, WIFI_PASS);
        if (st == WL_CONNECTED) { WifiFrom = 0; WriteWifiMemory(WIFI_SSID, WIFI_PASS); }
    }
    if (st != WL_CONNECTED && WifiMem && !(sketch && strcmp(WifiMemSsid, WIFI_SSID) == 0 && strcmp(WifiMemPass, WIFI_PASS) == 0))
    {
        st = WiFi.begin(WifiMemSsid, WifiMemPass);
        if (st == WL_CONNECTED) { WifiFrom = 1; }
    }
    if (st != WL_CONNECTED)
    {
        strcpy(WifiText, "E9");
        WifiUp = false;
        return false;
    }

    // The module says "connected" as soon as it has joined the access point, before DHCP has finished,
    // so localIP() returns 0.0.0.0 for a while.  Poll until a real address shows up.
    unsigned long t0 = millis();
    bool haveIp = false;
    while (!haveIp && (millis() - t0) < WIFI_DHCP_TIMEOUT_MS)
    {
        haveIp = ReadIpText();
        if (!haveIp) { delay(250); }
    }
    if (!haveIp) { strcpy(WifiText, "E8"); }    // joined, no lease yet - ServiceNetwork() keeps checking

    WifiUp = true;
    TcpServer.begin();
    return true;
}

// Reads the current address into WifiText.  Returns false (and leaves WifiText alone) while it is 0.0.0.0.
bool ReadIpText()
{
    IPAddress ip = WiFi.localIP();
    if (ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0) { return false; }
    snprintf(WifiText, sizeof(WifiText), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
    return true;
}

// 1.9.5: to the page, wherever it is - the USB cable in page mode, else the WiFi client
bool PageWrite(const uint8_t *buf, size_t n)
{
    if (UsbPage) { Serial.write(buf, n); return true; }
    if (!ClientUp) { return false; }
    if (TcpClient.write(buf, n) == 0) { ClientUp = false; return false; }
    return true;
}

void TelemetrySample(unsigned long now)
{
    if (TelCount >= TEL_MAX_SAMPLES) { return; }
    uint8_t *p = TelBuf + 6 + TelCount * TEL_SAMPLE_BYTES;
    Put32(p, (uint32_t)now); p += 4;
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        Put16(p, Target[m]);     p += 2;
        Put16(p, Goal_q[m]);     p += 2;
        Put16(p, Feedback_q[m]); p += 2;
        Put16(p, PWMpid_q[m]);   p += 2;
    }
    *p++ = (uint8_t)StatusByte();
    *p++ = PowerScale;
    if (SensorFresh())                           // seat angles x100; -32768 = no sensor data (1.8)
    {
        Put16(p, (int)constrain(SeatRoll * 100.0f, -32767.0f, 32767.0f));  p += 2;
        Put16(p, (int)constrain(SeatPitch * 100.0f, -32767.0f, 32767.0f)); p += 2;
        Put16(p, (int)constrain(ImuYaw * 100.0f, -32767.0f, 32767.0f));    p += 2;
    }
    else
    {
        Put16(p, -32768); p += 2;
        Put16(p, -32768); p += 2;
        Put16(p, -32768); p += 2;
    }
    uint32_t us = IsrCyclesMax / (F_CPU / 1000000UL);
    Put16(p, (int)(us > 65535 ? 65535 : us));
    TelCount++;
}

void TelemetrySend()
{
    if (TelCount == 0) { return; }
    TelBuf[0] = 'S'; TelBuf[1] = 'M'; TelBuf[2] = '3'; TelBuf[3] = 'T';
    TelBuf[4] = 1;
    TelBuf[5] = TelCount;
    size_t n = 6 + (size_t)TelCount * TEL_SAMPLE_BYTES;
    if (TelExtras)                               // 1.9: the sensor's acceleration and gyro, in the same write
    {
        uint8_t *x = TelBuf + n;
        unsigned long now = millis();
        x[0] = 'S'; x[1] = 'M'; x[2] = '3'; x[3] = 'X'; x[4] = 1; x[5] = TEL_EXTRA_BYTES;
        uint8_t *p = x + 6;
        for (uint8_t i = 0; i < 3; i++) { Put16(p, ImuAcc[i]);  p += 2; }
        for (uint8_t i = 0; i < 3; i++) { Put16(p, ImuGyro[i]); p += 2; }
        Put16(p, ImuTemp_x100); p += 2;
        Put16(p, AccPeak_mg);   p += 2;
        uint8_t fl = 0;
        if (ImuAccMillis != 0 && (now - ImuAccMillis) < 500)   { fl |= 1; }
        if (ImuGyroMillis != 0 && (now - ImuGyroMillis) < 500) { fl |= 2; }
        *p++ = fl;
        *p++ = AccBurstN;
        n += 6 + TEL_EXTRA_BYTES;
    }
    AccPeak_mg = 0;                              // per burst, sent or not
    AccBurstN = 0;
    PageWrite(TelBuf, n);
    TelCount = 0;
}

// [rdd] -> one "SM3D" frame, little-endian:  'S' 'M' '3' 'D', uint8 version (1), uint8 payload length, then
//   uint32 millis | uint8 RSTSR0, RSTSR1, RSTSR2 as found at boot | uint8 flags: bit0 = cold/warm flag usable,
//   bits1-2 = SMC3Utils monitor mode | uint32 USB commands ok, USB position commands, USB bytes thrown away,
//   WiFi commands ok | uint16 ms since the last USB command, ms since the last USB position command
//   (65535 = longer, or never) | uint32 longest loop() pass in us since the last [rdd] | int8 WiFi RSSI dBm |
//   uint8 build flags | uint8 firmware number | per motor: uint8 why it is off (0 on, 1 cutoff, 2 command,
//   3 boot), uint16 cutoff trips, int16 pot (0-1023) at the last trip or -1, uint16 pot glitches, uint16 biggest
//   one-cycle pot change since the last [rdd] (Q4), uint16 lowest and highest pot reading since then (Q4) |
//   uint8 n, then EEPROM bytes 0..n-1.
//   Version 2 (1.8) adds, after the EEPROM copy:  uint8 sensor state ([rdi] byte 1), uint8 tilt limit, uint8 baud
//   index, uint32 baud, uint16 angle packets in the last second, uint32 good packets, uint16 bad checksums,
//   uint16 ms since the last angle packet (65535 = longer/never), int16 raw roll, raw pitch, yaw (x100),
//   int16 level roll, level pitch (x100), uint16 tilt trips, uint8 matrix view, uint8 matrix flags,
//   uint8 error code on the matrix (255 = none).
//   Version 3 (1.9) adds:  per motor int16 level trim, travel low, travel high, end stop low, end stop high |
//   int16 tilt limits x10 (roll +, roll -, pitch +, pitch -) | uint8 flags: bit0 seat set-up saved (what runs is in
//   EEPROM), bit1 a seat set-up is in EEPROM, bit2 "SM3X" frames on, bit3 the page holds the seat | uint32 SimHub
//   position commands ignored while the page held the seat | uint16 acceleration and gyro packets in the last
//   second | int16 raw acceleration x, y, z, raw gyro x, y, z, temperature x100.
//   Version 4 (1.9.1) adds:  uint16 soft-start speed (counts/s, 0 = off) | uint8 motors still creeping (bit n =
//   motor n+1).
//   1.9.5: flags bit4 = USB page mode on.
// Only ever sent to the page (the WiFi client, or USB in page mode), so SimHub and SMC3Utils on USB never see it.
uint8_t MatrixError();
int8_t  LastRssi = -127;                         // 1.9.5: in USB page mode the last reading is sent (no WiFi calls)

void SendDiag()
{
    uint8_t buf[6 + 255];
    uint8_t *p = buf + 6;
    unsigned long now = millis();
    Put32(p, (uint32_t)now); p += 4;
    *p++ = ResetFlags[0]; *p++ = ResetFlags[1]; *p++ = ResetFlags[2];
    *p++ = (uint8_t)(ResetCwsfOk | ((SerialFeedbackEnabled & 3) << 1));
    Put32(p, UsbFramesOk);     p += 4;
    Put32(p, UsbTargetFrames); p += 4;
    Put32(p, UsbBadBytes);     p += 4;
    Put32(p, TcpFramesOk);     p += 4;
    unsigned long a = LastUsbPacketMillis ? (now - LastUsbPacketMillis) : 65535UL;
    unsigned long b = LastUsbTargetMillis ? (now - LastUsbTargetMillis) : 65535UL;
    Put16(p, (int)(a > 65535UL ? 65535UL : a)); p += 2;
    Put16(p, (int)(b > 65535UL ? 65535UL : b)); p += 2;
    Put32(p, LoopMaxUs); p += 4;
    LoopMaxUs = 0;
    if (!UsbPage)
    {
        int32_t rssi = WiFi.RSSI();               // one modem round-trip, only when asked
        if (rssi < -127) { rssi = -127; } else if (rssi > 0) { rssi = 0; }
        LastRssi = (int8_t)rssi;
    }
    *p++ = (uint8_t)LastRssi;
    *p++ = (uint8_t)BuildFlags();
    *p++ = (uint8_t)FW_VERSION_NUM;
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        *p++ = DisableReason[m];
        Put16(p, CutoffTrips[m]);  p += 2;
        Put16(p, CutoffAtFb[m]);   p += 2;
        Put16(p, PotJumps[m]);     p += 2;
        Put16(p, PotJumpMax_q[m]); p += 2;
        Put16(p, FbMin_q[m]);      p += 2;
        Put16(p, FbMax_q[m]);      p += 2;
        int fb = Feedback_q[m];                    // start a new interval for the per-read values
        PotJumpMax_q[m] = 0;
        FbMin_q[m] = fb;
        FbMax_q[m] = fb;
    }
    *p++ = 72;
    for (int i = 0; i < 72; i++) { *p++ = EEPROM.read(i); }
    *p++ = SensorState();
    *p++ = TiltLimitDeg;
#if IMU_PORT
    *p++ = ImuBaudIdx;
    Put32(p, ImuBauds[ImuBaudIdx]); p += 4;
#else
    *p++ = 255;
    Put32(p, 0); p += 4;
#endif
    Put16(p, ImuRateHz); p += 2;
    Put32(p, ImuPackets); p += 4;
    Put16(p, ImuBadSums); p += 2;
    unsigned long age = ImuLastMillis ? (now - ImuLastMillis) : 65535UL;
    Put16(p, (int)(age > 65535UL ? 65535UL : age)); p += 2;
    Put16(p, (int)constrain(ImuRawRoll * 100.0f, -32767.0f, 32767.0f));  p += 2;
    Put16(p, (int)constrain(ImuRawPitch * 100.0f, -32767.0f, 32767.0f)); p += 2;
    Put16(p, (int)constrain(ImuYaw * 100.0f, -32767.0f, 32767.0f));      p += 2;
    Put16(p, LevelRoll_x100);  p += 2;
    Put16(p, LevelPitch_x100); p += 2;
    Put16(p, TiltTrips); p += 2;
    *p++ = LedView;
    *p++ = LedFlags;
    *p++ = MatrixError();
    for (uint8_t m = 0; m < NUM_MOTORS; m++)       // version 3 (1.9): seat set-up and the rest of the sensor
    {
        Put16(p, Trim[m]);     p += 2;
        Put16(p, TravelLo[m]); p += 2;
        Put16(p, TravelHi[m]); p += 2;
        Put16(p, EndLo[m]);    p += 2;
        Put16(p, EndHi[m]);    p += 2;
    }
    for (uint8_t d = 0; d < 4; d++) { Put16(p, TiltLim_x10[d]); p += 2; }
    uint8_t fl = 0;
    if (SeatSetupSaved()) { fl |= 1; }
    if (EEPROM.read(SEAT_EE) == 0xB9) { fl |= 2; }
    if (TelExtras) { fl |= 4; }
    if (PageHolds()) { fl |= 8; }
    if (UsbPage) { fl |= 16; }
    *p++ = fl;
    Put32(p, UsbHeldFrames); p += 4;
    Put16(p, ImuAccHz);  p += 2;
    Put16(p, ImuGyroHz); p += 2;
    for (uint8_t i = 0; i < 3; i++) { Put16(p, ImuAcc[i]);  p += 2; }
    for (uint8_t i = 0; i < 3; i++) { Put16(p, ImuGyro[i]); p += 2; }
    Put16(p, ImuTemp_x100); p += 2;
    Put16(p, (int)SoftRate); p += 2;            // version 4 (1.9.1): soft start
    uint8_t cr = 0;
    for (uint8_t m = 0; m < NUM_MOTORS; m++) { if (SoftStart[m] && Disable[m] == 0) { cr |= (uint8_t)(1 << m); } }
    *p++ = cr;
    *p++ = (uint8_t)(WifiFw >> 16);              // version 5 (1.9.2): WiFi module firmware, WiFi memory, last update
    *p++ = (uint8_t)(WifiFw >> 8);
    *p++ = (uint8_t)WifiFw;
    uint8_t wf = 0;
    if (WifiMem) { wf |= 1; }
    if (WifiFrom == 1) { wf |= 2; }
    if (!SketchHasWifi()) { wf |= 4; }
    *p++ = wf;
    *p++ = OtaState;
    Put16(p, OtaCode); p += 2;
    size_t len = (size_t)(p - (buf + 6));
    buf[0] = 'S'; buf[1] = 'M'; buf[2] = '3'; buf[3] = 'D';
    buf[4] = 5;
    buf[5] = (uint8_t)len;
    PageWrite(buf, 6 + len);
}

//****************************************************************************************************************
//    Firmware update over WiFi (1.9.2).  The page converts the .bin into the UNO R4 WiFi's .ota format and serves it at
//    http://<page>:<port>/ota/<token>.ota; the WiFi module downloads it into its own storage, checks it (magic number,
//    CRC) and programs this board through its bootloader - exactly as it does a USB upload - then starts it.  If
//    anything goes wrong before the programming starts, the running firmware carries on and says why.
//
//    "SM3U" frame (after [rdu], on every change and 4 times a second while it runs), little-endian:
//      'S' 'M' '3' 'U', uint8 version (1), uint8 length (23), then uint8 state (0 idle, 1 stopping the motors,
//      2 starting, 3 downloading, 4 checking, 5 installing, 6 failed) | int16 code (0, or why it stopped: -100 SimHub
//      is sending over USB, -101 WiFi module firmware too old, -102 no address for the file, -103 the download stopped,
//      -104 programming failed, else the WiFi module's code) | uint32 bytes downloaded | uint32 file size | uint8
//      flags (bit0 the download reports progress, bit1 WiFi details remembered) | uint8 WiFi module firmware major,
//      minor, patch | char[8] this firmware's version ("1.9.2", zero-padded) - the page checks it after an update.
//****************************************************************************************************************

// UNO R4 boards package 1.3.0 added the non-blocking download (WiFi module 0.5.0); older ones only have the blocking one.
// (1.9.3: no string literal in these signatures - the boards package's GCC 7 can't compile that.)
struct OtaCompat
{
    template <typename T> static auto Start(T &o, const char *url, const char *path, int)
        -> decltype((int)o.startDownload(url, path))
    {
        return (int)o.startDownload(url, path);
    }
    template <typename T> static int Start(T &, const char *, const char *, long) { return OTA_NO_PROGRESS; }
    template <typename T> static auto Progress(T &o, int) -> decltype((int)o.downloadProgress())
    {
        return (int)o.downloadProgress();
    }
    template <typename T> static int Progress(T &, long) { return OTA_NO_PROGRESS; }
};

void SendOtaFrame()
{
    uint8_t b[6 + 23];
    uint8_t *p = b + 6;
    *p++ = OtaState;
    Put16(p, OtaCode); p += 2;
    Put32(p, OtaDone); p += 4;
    Put32(p, OtaTotal); p += 4;
    *p++ = (uint8_t)((OtaProgress ? 1 : 0) | (WifiMem ? 2 : 0));
    *p++ = (uint8_t)(WifiFw >> 16);
    *p++ = (uint8_t)(WifiFw >> 8);
    *p++ = (uint8_t)WifiFw;
    const char *v = FW_VERSION;
    for (uint8_t i = 0; i < 8; i++) { *p++ = (uint8_t)(*v ? *v++ : 0); }
    b[0] = 'S'; b[1] = 'M'; b[2] = '3'; b[3] = 'U'; b[4] = 1; b[5] = 23;
    OtaFrameDue = false;
    OtaLastFrame = millis();
    PageWrite(b, sizeof(b));                     // 1.9.5: over USB in page mode (the version text; updates stay WiFi)
}

void OtaFail(int code)
{
    OtaCode = (int16_t)constrain(code, -32768, 32767);
    if (OtaCode == 0) { OtaCode = OTA_ERR_FLASH; }
    OtaState = OTA_FAILED;
    OtaFrameDue = true;
}

static inline bool UsbBusy(unsigned long now)  // SimHub / SMC3Utils sent something over USB in the last 3 s
{
    return LastUsbPacketMillis != 0 && (now - LastUsbPacketMillis) < 3000UL;
}

void OtaStart()
{
    if (UpdateRunning()) { return; }
    unsigned long now = millis();
    OtaCode = 0;
    OtaDone = OtaTotal = 0;
    OtaProgress = false;
    // where the file is: the page's address (as given, or the one it connects from), its web port, the file's number
    uint8_t ip[4] = {(uint8_t)(OtaIpHi >> 8), (uint8_t)OtaIpHi, (uint8_t)(OtaIpLo >> 8), (uint8_t)OtaIpLo};
    if (OtaIpHi == 0 && OtaIpLo == 0 && ClientUp)
    {
        IPAddress a = TcpClient.remoteIP();
        for (uint8_t i = 0; i < 4; i++) { ip[i] = a[i]; }
    }
    if ((ip[0] | ip[1] | ip[2] | ip[3]) == 0 || OtaPort == 0) { OtaFail(OTA_ERR_NOURL); return; }
    snprintf(OtaUrl, sizeof(OtaUrl), "http://%u.%u.%u.%u:%u/ota/%04x.ota", ip[0], ip[1], ip[2], ip[3],
             (unsigned)OtaPort, (unsigned)OtaToken);
    if (UsbBusy(now)) { OtaFail(OTA_ERR_SIMHUB); return; }
    if (WifiFw == 0) { WifiFw = ParseWifiFw(WiFi.firmwareVersion()); }
    if (WifiFw != 0 && WifiFw < 0x000400UL) { OtaFail(OTA_ERR_OLDWIFI); return; }
    for (uint8_t m = 0; m < NUM_MOTORS; m++) { DisableReason[m] = DIS_UPDATE; DisableMotor(m); }
    OtaState = OTA_PREP;
    OtaSince = now;
    OtaFrameDue = true;
}

// [u a b] from the page
void OtaCommand(uint8_t a, uint8_t b)
{
    if (b == '=')
    {
        uint16_t v = (uint16_t)ValueReg;
        if (a == 'P') { OtaPort = v; }
        else if (a == 'T') { OtaToken = v; }
        else if (a == 'I') { OtaIpHi = v; }
        else if (a == 'J') { OtaIpLo = v; }
    }
    else if (a == 'G' && b == 'O') { OtaStart(); }
    else if (a == 'O' && b == 'K')
    {
        if (!UpdateRunning()) { OtaState = OTA_IDLE; OtaCode = 0; OtaDone = OtaTotal = 0; OtaFrameDue = true; }
    }
}

void ServiceOta(unsigned long now)             // loop(): one step at a time
{
    switch (OtaState)
    {
        case OTA_PREP:                           // give the brakes a moment
            if (now - OtaSince < 300) { return; }
            OtaState = OTA_BEGIN;
            OtaFrameDue = true;
            return;

        case OTA_BEGIN:
        {
            int r = (int)Ota.begin(OTA_FILE);    // clears the WiFi module's storage for the file
            if (r != 0) { OtaFail(r); return; }
            int size = (WifiFw == 0 || WifiFw >= 0x000500UL) ? OtaCompat::Start(Ota, OtaUrl, OTA_FILE, 0) : OTA_NO_PROGRESS;
            if (size == OTA_NO_PROGRESS)
            {
                // WiFi module 0.4.x, or a boards package before 1.3.0: a blocking download (up to a minute, no
                // progress; the page waits and the matrix shows the update)
                OtaState = OTA_DOWNLOAD;
                OtaProgress = false;
                if (ClientUp) { SendOtaFrame(); }
                size = Ota.download(OtaUrl, OTA_FILE);
                if (size <= 0) { OtaFail(size == 0 ? OTA_ERR_STALLED : size); return; }
                OtaTotal = OtaDone = (uint32_t)size;
                OtaState = OTA_VERIFY;
                OtaFrameDue = true;
                return;
            }
            if (size <= 0) { OtaFail(size == 0 ? OTA_ERR_STALLED : size); return; }
            OtaTotal = (uint32_t)size;
            OtaDone = 0;
            OtaProgress = true;
            OtaState = OTA_DOWNLOAD;
            OtaLastPoll = OtaLastMove = now;
            OtaFrameDue = true;
            return;
        }

        case OTA_DOWNLOAD:
        {
            if (now - OtaLastPoll < 200) { return; }
            OtaLastPoll = now;
            int p = OtaCompat::Progress(Ota, 0);
            if (p < 0) { OtaFail(p); return; }
            if ((uint32_t)p != OtaDone) { OtaDone = (uint32_t)p; OtaLastMove = now; }
            if (OtaDone >= OtaTotal) { OtaState = OTA_VERIFY; OtaFrameDue = true; }
            else if (now - OtaLastMove > 20000UL) { OtaFail(OTA_ERR_STALLED); }
            return;
        }

        case OTA_VERIFY:
        {
            int r = (int)Ota.verify();           // magic number and CRC of what arrived
            if (r != 0) { OtaFail(r); return; }
            if (UsbBusy(now)) { OtaFail(OTA_ERR_SIMHUB); return; }   // programming goes over the USB link's serial line
            OtaState = OTA_FLASH;
            OtaSince = now;
            OtaFrameDue = true;
            return;
        }

        case OTA_FLASH:
            if (now - OtaSince < 400) { return; }   // the "installing" frame and the matrix get out first
            if (UsbBusy(now)) { OtaFail(OTA_ERR_SIMHUB); return; }
            // The WiFi module resets this board into its bootloader, programs it and starts the new firmware; this call
            // only comes back if that could not even begin.
            (void)Ota.update(OTA_FILE);
            OtaFail(OTA_ERR_FLASH);
            return;

        default:
            return;
    }
}

void FeedTcpByte(unsigned int c)
{
    if (TcpBufferEnd == -1)
    {
        if (c == START_BYTE) { TcpBufferEnd = 0; }
    }
    else
    {
        TcpRxBuffer[TcpBufferEnd++] = c;
        if (TcpBufferEnd > 3)
        {
            if (TcpRxBuffer[3] == END_BYTE) { ParseCommand(TcpRxBuffer, 1); }
            TcpBufferEnd = -1;
        }
    }
}

void ServiceNetwork(unsigned long now)
{
    if (!WifiUp)
    {
        // Reconnect attempts block for up to WIFI_CONNECT_TIMEOUT_MS, so only try while SimHub is quiet.
        bool hostIdle = (now - LastPacketMillis) > 5000;
        if (hostIdle && (now - WifiLastAttempt) > WIFI_RETRY_MS)
        {
            if (ConnectWifi() && WifiText[0] != 'E') { PendingIpScroll = true; }   // back: show the address once
        }
        return;
    }

    if (!ClientUp)
    {
        if (now - NetLastAccept >= 500)                 // one round-trip every 500 ms while nobody is connected
        {
            NetLastAccept = now;
            WiFiClient c = TcpServer.available();
            if (c)
            {
                TcpClient = c;
                ClientUp = true;
                TelCount = 0;
                TelExtras = false;                      // a new page asks for them if it wants them
                TcpBufferEnd = -1;
                NetLastSample = NetLastSend = NetLastPoll = NetLastCheck = now;
            }
            else if (now - NetLastCheck >= 10000)       // every 10 s make sure the WiFi link itself is still up
            {
                NetLastCheck = now;
                if (WiFi.status() != WL_CONNECTED) { WifiUp = false; }
                else if (WifiText[0] == 'E') { if (ReadIpText()) { PendingIpScroll = true; } }   // E8: lease arrived late
            }
        }
        return;
    }

    if (now - NetLastSample >= TELEMETRY_SAMPLE_MS)     // free: just reads ISR variables into the buffer
    {
        NetLastSample += TELEMETRY_SAMPLE_MS;
        if (now - NetLastSample > 200) { NetLastSample = now; }
        TelemetrySample(now);
    }
    if (now - NetLastSend >= TELEMETRY_SEND_MS)         // one modem round-trip
    {
        NetLastSend = now;
        TelemetrySend();
    }
    // Commands from the PC tool: every 100 ms normally, every 20 ms while it is driving a motor (test signal /
    // auto-tune), so its position commands are not bunched up.  One round-trip per poll plus one per chunk.
    unsigned long pollMs = (now - TcpLastTargetMillis < 2000) ? 20 : 100;
    if (ClientUp && now - NetLastPoll >= pollMs)
    {
        NetLastPoll = now;
        int avail = TcpClient.available();
        uint8_t chunk[64];
        while (avail > 0)
        {
            int n = TcpClient.read(chunk, (size_t)(avail > 64 ? 64 : avail));
            if (n <= 0) { break; }
            for (int i = 0; i < n; i++) { FeedTcpByte(chunk[i]); }
            avail -= n;
        }
    }
    // 1.9.2: firmware update state - on every change, and 4 times a second while it runs
    if (ClientUp && (OtaFrameDue || (UpdateRunning() && now - OtaLastFrame >= 250))) { SendOtaFrame(); }
    if (ClientUp && now - NetLastCheck >= 2000)         // one round-trip; detect a vanished client
    {
        NetLastCheck = now;
        if (!TcpClient.connected()) { ClientUp = false; }
    }
    if (!ClientUp)
    {
        TcpClient.stop();
        TelCount = 0;
    }
}

// 1.9.5: USB page mode.  [kU1] from USB switches it on (or keeps it on) for USB_PAGE_TIMEOUT_MS, [kU0] ends it.
// Not while a firmware update runs: that needs the WiFi module.
void UsbPageEnd()
{
    UsbPage = false;
    TelCount = 0;
    TelExtras = false;                           // the page asks for them again wherever it connects next
    NetLastSample = NetLastSend = NetLastPoll = NetLastCheck = NetLastAccept = millis();
}

void UsbPageCommand(uint8_t n)
{
    if (n == 0 || n == '0') { if (UsbPage) { UsbPageEnd(); } return; }
    if (UpdateRunning()) { return; }
    unsigned long now = millis();
    UsbPageMillis = now;
    if (UsbPage) { return; }
    if (ClientUp)                                // the page left WiFi for the cable: drop the old connection
    {
        TcpClient.stop();
        ClientUp = false;
    }
    UsbPage = true;
    UsbPageStarts++;
    TelCount = 0;
    TelExtras = false;
    NetLastSample = NetLastSend = now;
}

void ServiceUsbPage(unsigned long now)           // loop() in page mode, instead of ServiceOta()/ServiceNetwork()
{
    if ((long)(now - UsbPageMillis) > (long)USB_PAGE_TIMEOUT_MS) { UsbPageEnd(); return; }
    if (now - NetLastSample >= TELEMETRY_SAMPLE_MS)
    {
        NetLastSample += TELEMETRY_SAMPLE_MS;
        if (now - NetLastSample > 200) { NetLastSample = now; }
        TelemetrySample(now);
    }
    if (now - NetLastSend >= TELEMETRY_SEND_MS)
    {
        NetLastSend = now;
        TelemetrySend();
    }
}

#else
void UsbPageCommand(uint8_t) {}                  // no WiFi build: no telemetry to send, nothing changes
#endif // ENABLE_WIFI

//****************************************************************************************************************
//    LED matrix (1.8).  One thing at a time, readable from across the room:
//      * the IP address scrolls after power-up (and every minute while the page isn't connected);
//      * problems scroll as words, "ERROR E3 MOTOR 3 CUTOFF", again and again while they last;
//      * otherwise the chosen view: motor bars, the seat horizon (motion sensor), or quiet;
//      * motor bars view while SimHub is idle: a slow oscilloscope trace.
//    The code between the BEGIN/END markers only uses the globals listed here, so the preview tool can build it on
//    a PC: Frame, Disable, DisableReason, Feedback_q, Goal_q, TargetLo/Hi, SeatRoll/SeatPitch, SensorFresh(),
//    SensorFlags, LedView, LedFlags, LastPosMillis, WifiText, WifiUp, ClientUp, PendingIpScroll, LastIpScroll,
//    UpdateShow(), UpdateFailed() (1.9.2).
//****************************************************************************************************************

// ==== LED MATRIX BEGIN ====
#if LED_MATRIX

struct GlyphDef { char c; uint8_t w; uint8_t col[5]; };   // up to 5 columns, bit0 = top row
const GlyphDef Font[] = {
    {'0', 3, {0x1F, 0x11, 0x1F, 0x00, 0x00}},
    {'1', 3, {0x12, 0x1F, 0x10, 0x00, 0x00}},
    {'2', 3, {0x1D, 0x15, 0x17, 0x00, 0x00}},
    {'3', 3, {0x11, 0x15, 0x1F, 0x00, 0x00}},
    {'4', 3, {0x07, 0x04, 0x1F, 0x00, 0x00}},
    {'5', 3, {0x17, 0x15, 0x1D, 0x00, 0x00}},
    {'6', 3, {0x1F, 0x15, 0x1D, 0x00, 0x00}},
    {'7', 3, {0x01, 0x19, 0x07, 0x00, 0x00}},
    {'8', 3, {0x1F, 0x15, 0x1F, 0x00, 0x00}},
    {'9', 3, {0x17, 0x15, 0x1F, 0x00, 0x00}},
    {'A', 3, {0x1E, 0x05, 0x1E, 0x00, 0x00}},
    {'B', 3, {0x1F, 0x15, 0x0A, 0x00, 0x00}},
    {'C', 3, {0x0E, 0x11, 0x11, 0x00, 0x00}},
    {'D', 3, {0x1F, 0x11, 0x0E, 0x00, 0x00}},
    {'E', 3, {0x1F, 0x15, 0x11, 0x00, 0x00}},
    {'F', 3, {0x1F, 0x05, 0x01, 0x00, 0x00}},
    {'G', 3, {0x0E, 0x11, 0x1D, 0x00, 0x00}},
    {'H', 3, {0x1F, 0x04, 0x1F, 0x00, 0x00}},
    {'I', 3, {0x11, 0x1F, 0x11, 0x00, 0x00}},
    {'J', 3, {0x08, 0x10, 0x0F, 0x00, 0x00}},
    {'K', 3, {0x1F, 0x04, 0x1B, 0x00, 0x00}},
    {'L', 3, {0x1F, 0x10, 0x10, 0x00, 0x00}},
    {'M', 5, {0x1F, 0x02, 0x04, 0x02, 0x1F}},
    {'N', 4, {0x1F, 0x02, 0x04, 0x1F, 0x00}},
    {'O', 3, {0x0E, 0x11, 0x0E, 0x00, 0x00}},
    {'P', 3, {0x1F, 0x05, 0x02, 0x00, 0x00}},
    {'Q', 3, {0x0E, 0x19, 0x16, 0x00, 0x00}},
    {'R', 3, {0x1F, 0x05, 0x1A, 0x00, 0x00}},
    {'S', 3, {0x12, 0x15, 0x09, 0x00, 0x00}},
    {'T', 3, {0x01, 0x1F, 0x01, 0x00, 0x00}},
    {'U', 3, {0x1F, 0x10, 0x1F, 0x00, 0x00}},
    {'V', 3, {0x0F, 0x10, 0x0F, 0x00, 0x00}},
    {'W', 5, {0x1F, 0x08, 0x04, 0x08, 0x1F}},
    {'X', 3, {0x1B, 0x04, 0x1B, 0x00, 0x00}},
    {'Y', 3, {0x03, 0x1C, 0x03, 0x00, 0x00}},
    {'Z', 3, {0x19, 0x15, 0x13, 0x00, 0x00}},
    {'.', 1, {0x10, 0x00, 0x00, 0x00, 0x00}},
    {':', 1, {0x0A, 0x00, 0x00, 0x00, 0x00}},
    {'!', 1, {0x17, 0x00, 0x00, 0x00, 0x00}},
    {'-', 2, {0x04, 0x04, 0x00, 0x00, 0x00}},
    {'/', 3, {0x18, 0x04, 0x03, 0x00, 0x00}},
    {' ', 2, {0x00, 0x00, 0x00, 0x00, 0x00}},
};
#define FONT_N (sizeof(Font) / sizeof(Font[0]))

// Returns an index into Font[], not a pointer: the Arduino IDE copies every function's declaration to the top of
// the sketch, where the GlyphDef type doesn't exist yet - so no sketch type may appear in a function signature.
uint8_t FindGlyph(char c)
{
    if (c >= 'a' && c <= 'z') { c = (char)(c - 32); }
    for (uint8_t i = 0; i < FONT_N; i++) { if (Font[i].c == c) { return i; } }
    return (uint8_t)(FONT_N - 1);               // unknown -> space (last entry)
}

void FrameClear()
{
    memset(Frame, 0, sizeof(Frame));
}

void FramePixel(int row, int col, uint8_t on)
{
    if (row < 0 || row > 7 || col < 0 || col > 11) { return; }
    Frame[row][col] = on ? 1 : 0;
}

void FrameToggle(int row, int col)
{
    if (row < 0 || row > 7 || col < 0 || col > 11) { return; }
    Frame[row][col] ^= 1;
}

// Text drawn at a fixed place (rows row..row+4)
void DrawText(int col, int row, const char *s)
{
    for (; *s; s++)
    {
        const GlyphDef *g = &Font[FindGlyph(*s)];
        for (int c = 0; c < g->w; c++)
        {
            for (int r = 0; r < 5; r++) { if ((g->col[c] >> r) & 1) { FramePixel(row + r, col + c, 1); } }
        }
        col += g->w + 1;
    }
}

// ---- Scrolling text (non-blocking; rows 1-5) ----

#define MARQUEE_MAX_COLS 200
uint8_t       MarqueeBits[MARQUEE_MAX_COLS];   // one byte per column, bit r = pixel in text row r (0-4)
int           MarqueeLen = 0;
int           MarqueeOffset = 0;
bool          MarqueeActive = false;
uint8_t       MarqueeKind = 0;                 // 0 = address, 1 = urgent problem, 2 = other problem
unsigned long MarqueeLastStep = 0;

void MarqueeStart(const char *s, uint8_t kind)
{
    MarqueeLen = 0;
    for (int i = 0; i < 12; i++) { MarqueeBits[MarqueeLen++] = 0; }        // lead-in: text enters from the right
    for (const char *p = s; *p != 0 && MarqueeLen < MARQUEE_MAX_COLS; p++)
    {
        const GlyphDef *g = &Font[FindGlyph(*p)];
        for (int c = 0; c < g->w && MarqueeLen < MARQUEE_MAX_COLS; c++) { MarqueeBits[MarqueeLen++] = g->col[c]; }
        if (MarqueeLen < MARQUEE_MAX_COLS) { MarqueeBits[MarqueeLen++] = 0; }   // one blank column between letters
    }
    MarqueeOffset = 0;
    MarqueeActive = true;
    MarqueeKind = kind;
    MarqueeLastStep = millis();
}

void DrawMarquee(unsigned long now)
{
    if (now - MarqueeLastStep >= (MarqueeKind == 0 ? (unsigned long)MARQUEE_STEP_MS : (unsigned long)ERROR_STEP_MS))
    {
        MarqueeLastStep = now;
        MarqueeOffset++;
        if (MarqueeOffset > MarqueeLen) { MarqueeActive = false; return; }
    }
    for (int c = 0; c < 12; c++)
    {
        int i = MarqueeOffset + c;
        uint8_t bits = (i >= 0 && i < MarqueeLen) ? MarqueeBits[i] : 0;
        for (int r = 0; r < 5; r++) { FramePixel(1 + r, c, (bits >> r) & 1); }
    }
}

// ---- Problems ----
// Codes: 0 WiFi module, 1-3 motor cutoff, 4 tilt limit, 5 sensor switched on but silent, 7 firmware update failed
// (1.9.2), 8 no IP address, 9 no WiFi.

#define ERR_MAX 8
uint8_t       ErrCodes[ERR_MAX];
uint8_t       ErrN = 0;
uint8_t       ErrNext = 0;                     // round robin through several problems
unsigned long ErrNextAt = 0;                   // next problem scroll not before this

void CollectErrors(unsigned long now)
{
    ErrN = 0;
    bool tilt = false;
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        if (Disable[m] && DisableReason[m] == DIS_CUTOFF) { ErrCodes[ErrN++] = 1 + m; }
        if (Disable[m] && DisableReason[m] == DIS_TILT) { tilt = true; }
    }
    if (tilt) { ErrCodes[ErrN++] = 4; }
    if ((SensorFlags & 1) && !SensorFresh() && now > 8000) { ErrCodes[ErrN++] = 5; }
    if (UpdateFailed()) { ErrCodes[ErrN++] = 7; }   // 1.9.2: until the page clears it, or the next update
#if ENABLE_WIFI
    if (WifiText[0] == 'E') { ErrCodes[ErrN++] = (uint8_t)(WifiText[1] - '0'); }
    else if (!WifiUp) { ErrCodes[ErrN++] = 9; }
#endif
}

uint8_t MatrixError()                          // first problem, for [rdd] (255 = none)
{
    CollectErrors(millis());
    return ErrN ? ErrCodes[0] : 255;
}

void ErrorText(uint8_t code, char *out, size_t n)
{
    switch (code)
    {
        case 0:  snprintf(out, n, "ERROR E0 WIFI MODULE"); break;
        case 1: case 2: case 3: snprintf(out, n, "ERROR E%u MOTOR %u CUTOFF", (unsigned)code, (unsigned)code); break;
        case 4:  snprintf(out, n, "ERROR E4 TILT LIMIT"); break;
        case 5:  snprintf(out, n, "ERROR E5 NO SENSOR"); break;
        case 7:  snprintf(out, n, "ERROR E7 UPDATE FAILED"); break;
        case 8:  snprintf(out, n, "ERROR E8 NO IP ADDRESS"); break;
        case 9:  snprintf(out, n, "ERROR E9 NO WIFI"); break;
        default: snprintf(out, n, "ERROR E%u", (unsigned)code); break;
    }
}

// ---- Views ----

static inline int HalfSteps(uint8_t m, int q)   // position (Q4) -> bar height in half pixels, 2..16
{
    int lo = TargetLo[m], hi = TargetHi[m];       // the range in force (SMC3 clip + travel limits, 1.9)
    if (hi - lo < 16) { lo = 0; hi = 1023; }
    long v = ((long)q * 16) >> POS_SHIFT;        // 1/16 count
    long h = 2 + ((v - (long)lo * 16) * 14 + (long)(hi - lo) * 8) / ((long)(hi - lo) * 16);
    if (h < 2) { h = 2; } else if (h > 16) { h = 16; }
    return (int)h;
}

// Three bars (columns 1-2, 5-6, 9-10), as high as each motor is; the two columns of a bar differ by half a pixel
// when the motor is in between.  A dot blinks where a motor should be when it is well behind.  A motor switched
// off on purpose shows a dash at the bottom; one stopped by a cutoff or the tilt limit blinks full height.
void DrawBars(bool blink)
{
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        int x = 1 + 4 * m;
        if (Disable[m])
        {
            if (DisableReason[m] == DIS_CUTOFF || DisableReason[m] == DIS_TILT)
            {
                if (blink) { for (int r = 0; r < 8; r++) { FramePixel(r, x, 1); FramePixel(r, x + 1, 1); } }
            }
            else
            {
                FramePixel(7, x, 1); FramePixel(7, x + 1, 1);
            }
            continue;
        }
        int h2 = HalfSteps(m, Feedback_q[m]);
        int left = h2 / 2, right = (h2 + 1) / 2;
        for (int r = 8 - left; r < 8; r++)  { FramePixel(r, x, 1); }
        for (int r = 8 - right; r < 8; r++) { FramePixel(r, x + 1, 1); }
        int t2 = HalfSteps(m, Goal_q[m]);
        int d = t2 - h2;
        if ((d >= 3 || d <= -3) && blink)
        {
            int row = 8 - (t2 + 1) / 2;
            FrameToggle(row, x); FrameToggle(row, x + 1);
        }
    }
}

// Seat horizon: the edge between dark and lit is the seat - it moves up when the seat's front goes up and tilts
// with the seat's roll (mounting and direction are set in the page).  LED_HORIZON_DEG puts it at the edge.
void DrawHorizon()
{
    float fs = LED_HORIZON_DEG;
    float pitch = SeatPitch, roll = SeatRoll;
    if (pitch > fs) { pitch = fs; } else if (pitch < -fs) { pitch = -fs; }
    if (roll > fs) { roll = fs; } else if (roll < -fs) { roll = -fs; }
    float yc = 4.0f - (pitch / fs) * 4.0f;           // row of the edge at the centre
    float slope = (roll / fs) * 0.6f;               // rows per column
    for (int c = 0; c < 12; c++)
    {
        float edge = yc + slope * ((float)c - 5.5f);
        for (int r = 0; r < 8; r++) { if ((float)r + 0.5f > edge) { FramePixel(r, c, 1); } }
    }
}

// Oscilloscope trace while idle
void DrawIdleTrace(unsigned long now)
{
    float ph = (float)(now % 100000UL) * 0.0025f;
    int prev = -1;
    for (int c = 0; c < 12; c++)
    {
        float y = 3.5f + 2.6f * sinf(ph + (float)c * 0.55f);
        int r = (int)(y + 0.5f);
        if (prev < 0) { prev = r; }
        if (r > prev + 1)      { for (int rr = prev + 1; rr <= r; rr++) { FramePixel(rr, c, 1); } }   // join steep parts
        else if (r < prev - 1) { for (int rr = r; rr < prev; rr++) { FramePixel(rr, c, 1); } }
        else                   { FramePixel(r, c, 1); }
        prev = r;
    }
}

void UpdateMatrix()
{
    static unsigned long last = 0;
    unsigned long now = millis();
    if (now - last < (1000 / LED_FPS)) { return; }
    last = now;
    bool blink = (now / 250) & 1;

    FrameClear();
    CollectErrors(now);

    int8_t up = UpdateShow();                    // 1.9.2: firmware update - "UP" and how far the download got
    if (up >= 0)
    {
        DrawText(2, 0, "UP");
        if (up > 100)                            // the WiFi module checks and installs it: the bar blinks
        {
            if (blink) { for (int c = 0; c < 12; c++) { FramePixel(7, c, 1); } }
        }
        else
        {
            int n = (up * 12 + 50) / 100;
            for (int c = 0; c < n; c++) { FramePixel(7, c, 1); }
            if (n < 12) { FramePixel(7, n, blink); }
        }
        Matrix.renderBitmap(Frame, 8, 12);
        return;
    }

    if (!MarqueeActive)
    {
#if ENABLE_WIFI
        if (PendingIpScroll)
        {
            PendingIpScroll = false;
            MarqueeStart(WifiText, 0);
            LastIpScroll = now;
        }
        else
#endif
        if (ErrN > 0 && (long)(now - ErrNextAt) >= 0)
        {
            if (ErrNext >= ErrN) { ErrNext = 0; }
            uint8_t code = ErrCodes[ErrNext++];
            char text[32];
            ErrorText(code, text, sizeof(text));
            MarqueeStart(text, (code >= 1 && code <= 4) ? 1 : 2);
        }
#if ENABLE_WIFI
        // Re-scroll the address every minute while the page isn't connected (and nothing is wrong)
        else if (ErrN == 0 && (LedFlags & 2) && WifiUp && !ClientUp && !UsbPage && WifiText[0] != 'E'
                 && (now - LastIpScroll) >= IP_SCROLL_REPEAT_MS)
        {
            LastIpScroll = now;
            MarqueeStart(WifiText, 0);
        }
#endif
    }

    if (MarqueeActive)
    {
        DrawMarquee(now);
        if (!MarqueeActive && MarqueeKind != 0)
        {
            ErrNextAt = now + (MarqueeKind == 1 ? 1500UL : 8000UL);   // a moment of the view between scrolls
        }
    }
    else
    {
        bool idle = (now - LastPosMillis) > LED_IDLE_AFTER_MS;
        if (LedView == 1 && SensorFresh())       { DrawHorizon(); }
        else if (LedView == 2)                   { FramePixel(7, 11, (now % 3000UL) < 150); }   // quiet: heartbeat
        else if (idle && (LedFlags & 1) && ErrN == 0) { DrawIdleTrace(now); }
        else                                     { DrawBars(blink); }
    }
    Matrix.renderBitmap(Frame, 8, 12);
}

#else
uint8_t MatrixError() { return 255; }          // no matrix: nothing shown
#endif // LED_MATRIX
// ==== LED MATRIX END ====

//****************************************************************************************************************
//    setup / loop
//****************************************************************************************************************

void setup()
{
#ifdef R_SYSTEM
    // Why did the board (re)start?  Keep the reset flags for [rdd], then clear them so the next reset is reported
    // on its own.  RSTSR2.CWSF: 0 = cold start (the supply was really off or collapsed), set to 1 here, and only a
    // power-on reset clears it again - so after a reset button / upload / watchdog it reads 1 ("warm").
    ResetFlags[0] = R_SYSTEM->RSTSR0;
    ResetFlags[1] = (uint8_t)R_SYSTEM->RSTSR1;
    ResetFlags[2] = R_SYSTEM->RSTSR2;
    R_SYSTEM->RSTSR0 = 0;
    R_SYSTEM->RSTSR1 = 0;
    R_SYSTEM->RSTSR2 = 1;
    ResetCwsfOk = (R_SYSTEM->RSTSR2 & 1) ? 1 : 0;
#endif

    Serial.begin(500000);
    RaiseSerialRxPriority();                     // 1.9.6: before the PID interrupt starts - see v1.9.6

#ifdef DWT
    // cycle counter for ISR load measurement
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif

    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        pinMode(ENApin[m], OUTPUT);
        pinMode(ENBpin[m], OUTPUT);
        digitalWrite(ENApin[m], LOW);
        digitalWrite(ENBpin[m], LOW);
    }
    pinMode(LED_BUILTIN, OUTPUT);

    analogReadResolution(ADC_BITS);
    for (uint8_t m = 0; m < NUM_MOTORS; m++) { (void)analogRead(FeedbackPin[m]); }   // configure the pins now, not in the ISR

    // 1. PWM first - it must get the GPT channels that belong to D9/D10/D11
    ReadEEProm();
    ReadBoardSettings();
    ReadSeatSetup();                            // 1.9: level trim and limits (needs the two above)
    ReadSoftStart();                            // 1.9.1: soft-start speed
#if IMU_PORT
    Serial1.begin(ImuBauds[ImuBaudIdx]);        // the baud rate that worked last time (9600 if never)
    RaiseSerialRxPriority();                     // 1.9.6
    ImuBaudSince = millis();
#endif
    ApplyPwmFrequencies();
    for (uint8_t m = 0; m < NUM_MOTORS; m++) { SetPwmQ(m, 0); }
    UpdatePIDLimits();
    UpdateSmoothingRate();

    // 2. Start the smoothed goals from wherever the motors actually are; the target is the centre - with a level
    //    trim (1.9) that is the level seat.  1.9.1: with the soft start the goal starts exactly at the pot (even
    //    past a travel limit - the seat may have ridden down while the rig was off) and creeps to the centre once
    //    the motors are switched on (EnableMotor() below); without it, as before, from inside the travel limits.
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        SetTarget(m, 512);
        int fb = analogRead(FeedbackPin[m]);
        int fb10 = constrain(fb >> POS_SHIFT, TargetLo[m], TargetHi[m]);
        GoalAcc[m] = (SoftRate > 0) ? ((long)fb << 8) : ((long)fb10 << 12);
        Goal_q[m] = (SoftRate > 0) ? fb : (fb10 << POS_SHIFT);
        Feedback_q[m] = fb;
        FbRaw_q[m] = fb;
        FbHeld[m] = 0;
        PidLastPosition[m] = fb;
    }

    // 3. LED matrix (takes one of the remaining timers)
#if LED_MATRIX
    Matrix.begin();
    FrameClear();
    DrawText(2, 1, "R4");                       // stays up while WiFi connects
    Matrix.renderBitmap(Frame, 8, 12);
    delay(600);
#endif

    // 3b. WiFi - done before the motors are enabled because WiFi.begin() blocks for up to the timeout.
#if ENABLE_WIFI
    ReadWifiMemory();                           // 1.9.2: the name and password it last joined with
    bool wifiOk = ConnectWifi();
    if (Serial)                                 // a terminal has the USB port open: print one text line
    {
        Serial.print(wifiOk ? "SMC3_R4 v" FW_VERSION " WiFi connected, IP " : "SMC3_R4 v" FW_VERSION " WiFi failed, code ");
        Serial.print(WifiText);
        Serial.print(" " FW_CAPS "\r\n");       // 1.9.2: the page finds this text in a .bin to see what it can do
    }
#if LED_MATRIX
    if (wifiOk && WifiText[0] != 'E') { PendingIpScroll = true; }   // the address scrolls once the loop runs
    LastIpScroll = millis();                    // (a WiFi problem scrolls as an ERROR instead)
#endif
#endif

    // 4. Control loop
    bool ok = StartPidTimer();
    if (!ok)
    {
        // No timer available: leave everything disabled and flash the built-in LED forever.
        while (1) { digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(100); }
    }

    // 5. Enable the motors (same as the original, which enables at startup)
    for (uint8_t m = 0; m < NUM_MOTORS; m++) { EnableMotor(m); }
    LastPacketMillis = millis();
    LastPosMillis = millis();
}

void loop()
{
    unsigned long now = millis();
    uint32_t loopStartUs = micros();

    CheckSerial();
    now = millis();                             // 1.9.5: the commands just parsed stamped millis() - keep "now" at or after
                                                // them, or "now - LastPacketMillis" wraps (a one-pass LOW POWER, and page
                                                // mode ending at once)

#if IMU_PORT
    PollImu();
    ServiceImu(now);
#endif

    if (now - LastMonitorMillis >= 20)          // every 20 ms, like the original
    {
        LastMonitorMillis = now;
        SendMonitorData();
    }

    // Comms timeout -> quarter power, as in the original (PowerScale 9 instead of 7) - 1.9.1: not while a motor is
    // still creeping to its position after power-up (it may need full power to lift the seat back)
    bool creeping = AnySoftStart();
    PowerScale = ((now - LastPacketMillis) > COMMS_TIMEOUT_MS && !creeping) ? 9 : 7;
    if (creeping)
    {
        for (uint8_t m = 0; m < NUM_MOTORS; m++)
        {
            if (SoftStart[m] && (now - SoftSince[m]) > SOFT_MAX_MS) { SoftStart[m] = 0; }   // never stuck creeping
        }
    }

#if ENABLE_WIFI
    if (UsbPage)                                // 1.9.5: the page is on the USB cable - no WiFi calls at all
    {
        ServiceUsbPage(now);
    }
    else
    {
        ServiceOta(now);                        // 1.9.2: firmware update, one step per pass
        ServiceNetwork(now);
    }
#endif

#if LED_MATRIX
    UpdateMatrix();
#endif

    uint32_t loopUs = micros() - loopStartUs;       // a long pass delays SimHub's commands (diagnostics)
    if (loopUs > LoopMaxUs) { LoopMaxUs = loopUs; }
}

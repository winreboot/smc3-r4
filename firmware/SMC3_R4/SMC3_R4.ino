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
//   * 12x8 LED matrix status display (tracking view, drive bars, fault codes, optional IMU bubble level).
//   * Serial1 (D0/D1) reserved for a WitMotion WT901C IMU through a MAX3232 (optional, off by default).
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
//                             D0 (RX) <- MAX3232 <- WT901C TX  (only if IMU_SERIAL1 is 1)
//
// Pots: wiper must stay within 0-5 V (the R4 is a 5 V board, so existing wiring is fine).
//****************************************************************************************************************

#define FW_VERSION     "1.7"           // shown in the USB boot banner; [ver] still answers 70 for SMC3Utils
#define FW_VERSION_NUM 17              // reported by [rd!] so smc3_scope can check what it is talking to

// v1.7 changes:  diagnostics for smc3_scope's Diagnostics page.  New read code [rdd] (WiFi only) returns one
//                "SM3D" frame: cause of the last reset, SimHub/USB command counters, longest loop() pass,
//                WiFi signal, and per motor why it is off, safety-cutoff trips (and where), pot glitches seen
//                at the full 4 kHz loop rate, pot range seen - plus a copy of EEPROM 0-63 so the page can
//                tell running settings from saved ones.  [rd!] flags gain bit1 MODE2, bit2 IMU_SERIAL1,
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

// Extra low-pass on the derivative term (0 = off, 2-4 sensible).
#define D_FILTER_SHIFT 3

// Fixed-point scaling of the I and D terms (matches original SMC3 motor 3 / v1.x). Leave alone.
#define I_SHIFT 6
#define D_SHIFT 5

//----------------------------------------------------------------------------------------------------------------
//  LED MATRIX / IMU OPTIONS
//----------------------------------------------------------------------------------------------------------------

#define LED_MATRIX 1               // 1 = use the 12x8 matrix, 0 = off
#define LED_VIEW   0               // 0 = tracking view (target/feedback cursors + drive bars)
                                   // 1 = IMU bubble level (needs IMU_SERIAL1 = 1)
#define LED_FPS    20              // display refresh rate from loop()

#define IMU_SERIAL1 0              // 1 = parse WitMotion WT901C angle packets on Serial1 (D0/D1)
#define IMU_BAUD    115200         // must match what the sensor is configured for
#define IMU_ROLL_FULL_SCALE  15.0f // degrees that put the bubble at the edge of the matrix
#define IMU_PITCH_FULL_SCALE 15.0f

//----------------------------------------------------------------------------------------------------------------
//  WIFI / TELEMETRY OPTIONS   <<< put your network name and password here >>>
//----------------------------------------------------------------------------------------------------------------

#define ENABLE_WIFI    1                   // 0 = no networking at all
#define WIFI_SSID      "YourNetworkName"   // 2.4 GHz network (the ESP32-S3 module has no 5 GHz)
#define WIFI_PASS      "YourPassword"
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

//----------------------------------------------------------------------------------------------------------------
//  INTERNALS - no need to touch
//----------------------------------------------------------------------------------------------------------------

#define PID_RATE_HZ      4000      // PROCESS_PERIOD_uS 250 in the original
#define PID_IRQ_PRIORITY 6         // lower number = higher priority; matrix refresh uses the default 12
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

#if TARGET_MAX_RATE > 30000
#error "TARGET_MAX_RATE must be 30000 or less"
#endif
#if LED_VIEW == 1 && IMU_SERIAL1 == 0
#error "LED_VIEW 1 (bubble level) needs IMU_SERIAL1 1"
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
//    [rdd]                     Diagnostics frame "SM3D" (WiFi client only) - new in 1.7
//    [sav]                     Save parameters to EEPROM
//    [ver]                     Software version (replies 70 so SMC3Utils is happy)

#include <EEPROM.h>
#include "FspTimer.h"
#include "pwm.h"
#if LED_MATRIX
#include "Arduino_LED_Matrix.h"
#endif
#if ENABLE_WIFI
#include <WiFiS3.h>
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

volatile uint8_t  DisableReason[NUM_MOTORS] = {DIS_BOOT, DIS_BOOT, DIS_BOOT};
volatile uint16_t CutoffTrips[NUM_MOTORS]   = {0, 0, 0};       // since power-up
volatile int16_t  CutoffAtFb[NUM_MOTORS]    = {-1, -1, -1};    // pot reading (0-1023) when the cutoff last tripped
volatile uint16_t PotJumps[NUM_MOTORS]      = {0, 0, 0};       // glitches (> POT_JUMP_COUNTS in one cycle) since power-up
volatile uint16_t PotJumpMax_q[NUM_MOTORS]  = {0, 0, 0};       // biggest cycle-to-cycle change since the last [rdd], Q4
volatile int      FbMin_q[NUM_MOTORS]       = {16383, 16383, 16383};   // pot range seen since the last [rdd], Q4
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
unsigned long NetLastSample = 0, NetLastSend = 0, NetLastPoll = 0, NetLastCheck = 0, NetLastAccept = 0;
int           TcpBufferEnd = -1;
unsigned int  TcpRxBuffer[5] = {0};

#define TEL_SAMPLE_BYTES   38
#define TEL_MAX_SAMPLES    (TELEMETRY_SEND_MS / TELEMETRY_SAMPLE_MS + 1)
uint8_t       TelBuf[6 + TEL_MAX_SAMPLES * TEL_SAMPLE_BYTES];
uint8_t       TelCount = 0;
#endif

//----------------------------------------------------------------------------------------------------------------
//  IMU state
//----------------------------------------------------------------------------------------------------------------

float         ImuRoll = 0, ImuPitch = 0, ImuYaw = 0;
unsigned long ImuLastMillis = 0;

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
#if IMU_SERIAL1
    flags |= 4;
#endif
#if LED_MATRIX
    flags |= 8;
#endif
#if ENABLE_WIFI && WIFI_STATIC_IP
    flags |= 16;
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

void EnableMotor(uint8_t m)           // loop() only
{
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

#ifdef MODE1
    if ((fb > clipMax_q) && (PWMrev[m] != 0))
    {
        out = rev_q;
        digitalWrite(ENApin[m], LOW);  digitalWrite(ENBpin[m], HIGH);
        SetPwmQ(m, rev_q);
    }
    else if ((fb < clipMin_q) && (PWMrev[m] != 0))
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
    if ((fb > clipMax_q) && (PWMrev[m] != 0))
    {
        out = rev_q;
        digitalWrite(ENApin[m], LOW);
        SetPwmQ(m, rev_q);
    }
    else if ((fb < clipMin_q) && (PWMrev[m] != 0))
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

    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        Goal_q[m] = SmoothTarget(m, Target[m]);
        int fb = analogRead(FeedbackPin[m]);

        // diagnostics: pot glitches (a jump no motor could make in 1/4 ms) and the range seen
        int jump = fb - Feedback_q[m];
        if (jump < 0) { jump = -jump; }
        if (jump > (POT_JUMP_COUNTS << POS_SHIFT) && PotJumps[m] < 65535) { PotJumps[m]++; }
        if (jump > PotJumpMax_q[m]) { PotJumpMax_q[m] = (uint16_t)jump; }
        if (fb < FbMin_q[m]) { FbMin_q[m] = fb; }
        if (fb > FbMax_q[m]) { FbMax_q[m] = fb; }
        Feedback_q[m] = fb;

        if ((fb > (CutoffLimitMax[m] << POS_SHIFT)) || (fb < (CutoffLimitMin[m] << POS_SHIFT)))
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
    if (value > InputClipMax[m]) { value = InputClipMax[m]; }
    else if (value < InputClipMin[m]) { value = InputClipMin[m]; }
    Target[m] = value;
}

#if ENABLE_WIFI
void SendDiag();
#endif

// source 0 = USB (SimHub / SMC3Utils), 1 = TCP (smc3_scope.py).  Same command set on both.
// "Host alive" for the comms-timeout power reduction = any USB packet, or a position command over TCP
// (reading telemetry alone must not keep the rig at full power).
unsigned long TcpLastTargetMillis = 0;

void ParseCommand(const unsigned int *buf, uint8_t source)
{
    bool isTarget = (buf[0] == 'A' || buf[0] == 'B' || buf[0] == 'C');
    if (source == 0 || isTarget) { LastPacketMillis = millis(); }
    if (source == 1 && isTarget) { TcpLastTargetMillis = millis(); }
    if (source == 0)
    {
        UsbFramesOk++;
        LastUsbPacketMillis = millis();
        if (isTarget) { UsbTargetFrames++; LastUsbTargetMillis = LastUsbPacketMillis; }
    }
    else
    {
        TcpFramesOk++;
    }
    ReplyTarget = source;
    unsigned int b1 = buf[1];
    unsigned int b2 = buf[2];
    int word = (int)((b1 * 256) + b2);

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
#if ENABLE_WIFI
                    case 'd': if (source == 1) { SendDiag(); } break;      // 1.7: diagnostics frame (WiFi only)
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
        case 'S': CutoffLimitMin[0] = b1; CutoffLimitMax[0] = 1023 - b1; InputClipMin[0] = b2; InputClipMax[0] = 1023 - b2; break;
        case 'T': CutoffLimitMin[1] = b1; CutoffLimitMax[1] = 1023 - b1; InputClipMin[1] = b2; InputClipMax[1] = 1023 - b2; break;
        case 'U': CutoffLimitMin[2] = b1; CutoffLimitMax[2] = 1023 - b1; InputClipMin[2] = b2; InputClipMax[2] = 1023 - b2; break;
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

        case 'm':
            if (b1 == 'o')
            {
                if (b2 >= '0' && b2 <= '3') { SerialFeedbackEnabled = b2 - '0'; }
            }
            break;
        case 's':
            if (b1 == 'a' && b2 == 'v') { WriteEEProm(); }
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
//    WitMotion WT901C parser (Serial1).  11-byte packets: 0x55, type, 8 data bytes, checksum.
//    Type 0x53 = angles: roll, pitch, yaw as int16 / 32768 * 180 degrees.
//****************************************************************************************************************

#if IMU_SERIAL1
void PollImu()
{
    static uint8_t buf[11];
    static uint8_t n = 0;

    while (Serial1.available())
    {
        uint8_t c = Serial1.read();
        if (n == 0)
        {
            if (c == 0x55) { buf[n++] = c; }
            continue;
        }
        buf[n++] = c;
        if (n < 11) { continue; }
        n = 0;

        uint8_t sum = 0;
        for (uint8_t i = 0; i < 10; i++) { sum += buf[i]; }
        if (sum != buf[10]) { continue; }

        if (buf[1] == 0x53)
        {
            int16_t r = (int16_t)((buf[3] << 8) | buf[2]);
            int16_t p = (int16_t)((buf[5] << 8) | buf[4]);
            int16_t y = (int16_t)((buf[7] << 8) | buf[6]);
            ImuRoll  = r / 32768.0f * 180.0f;
            ImuPitch = p / 32768.0f * 180.0f;
            ImuYaw   = y / 32768.0f * 180.0f;
            ImuLastMillis = millis();
        }
    }
}
#endif

//****************************************************************************************************************
//    WiFi: TCP server for telemetry out / SMC3 commands in
//
//    Telemetry frame (little-endian):  'S' 'M' '3' 'T', uint8 version(1), uint8 sampleCount, then per sample:
//      uint32 millis
//      3 x { int16 target(0-1023), int16 goal(Q4), int16 feedback(Q4), int16 pid(Q4, signed, before PWMmin) }
//      uint8 statusByte, uint8 powerScale, int16 roll*100, int16 pitch*100, int16 yaw*100, uint16 isrMaxMicros
//    = 38 bytes per sample.  Commands from the client use the exact SMC3 "[xyz]" format; replies to [rd?]
//    go back over the same connection.
//****************************************************************************************************************

#if ENABLE_WIFI

static inline void Put16(uint8_t *p, int v)  { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); }
static inline void Put32(uint8_t *p, uint32_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF; }

bool ReadIpText();

bool ConnectWifi()
{
    WifiLastAttempt = millis();
    if (WiFi.status() == WL_NO_MODULE)
    {
        strcpy(WifiText, "E0");
        WifiUp = false;
        return false;
    }
    WiFi.setHostname(WIFI_HOSTNAME);
#if WIFI_STATIC_IP
    WiFi.config(IPAddress(WIFI_IP), IPAddress(WIFI_DNS), IPAddress(WIFI_GATEWAY), IPAddress(WIFI_SUBNET));
#endif
    WiFi.setTimeout(WIFI_CONNECT_TIMEOUT_MS);
    int st = WiFi.begin(WIFI_SSID, WIFI_PASS);
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
    Put16(p, (int)(ImuRoll * 100.0f));  p += 2;
    Put16(p, (int)(ImuPitch * 100.0f)); p += 2;
    Put16(p, (int)(ImuYaw * 100.0f));   p += 2;
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
    if (TcpClient.write(TelBuf, n) == 0) { ClientUp = false; }
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
// Only ever sent to the WiFi client, so SimHub and SMC3Utils on USB never see it.
void SendDiag()
{
    uint8_t buf[6 + 160];
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
    int32_t rssi = WiFi.RSSI();                   // one modem round-trip, only when asked
    if (rssi < -127) { rssi = -127; } else if (rssi > 0) { rssi = 0; }
    *p++ = (uint8_t)(int8_t)rssi;
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
    *p++ = 64;
    for (int i = 0; i < 64; i++) { *p++ = EEPROM.read(i); }
    size_t len = (size_t)(p - (buf + 6));
    buf[0] = 'S'; buf[1] = 'M'; buf[2] = '3'; buf[3] = 'D';
    buf[4] = 1;
    buf[5] = (uint8_t)len;
    if (TcpClient.write(buf, 6 + len) == 0) { ClientUp = false; }
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
        if (hostIdle && (now - WifiLastAttempt) > WIFI_RETRY_MS) { ConnectWifi(); }
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
                TcpBufferEnd = -1;
                NetLastSample = NetLastSend = NetLastPoll = NetLastCheck = now;
            }
            else if (now - NetLastCheck >= 10000)       // every 10 s make sure the WiFi link itself is still up
            {
                NetLastCheck = now;
                if (WiFi.status() != WL_CONNECTED) { WifiUp = false; }
                else if (WifiText[0] == 'E') { ReadIpText(); }   // E8: lease arrived late? pick it up
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

#endif // ENABLE_WIFI

//****************************************************************************************************************
//    LED matrix
//****************************************************************************************************************

#if LED_MATRIX

// 3x5 glyphs, one byte per row, bit2 = left pixel.  Index: 0-9 = digits, 10 = 'E', 11 = 'R', 12 = '-', 13 = '.', 14+ = blank
#define GL_E     10
#define GL_R     11
#define GL_DASH  12
#define GL_DOT   13
#define GL_BLANK 14
const uint8_t Glyph[14][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111},   // 0
    {0b010, 0b110, 0b010, 0b010, 0b111},   // 1
    {0b111, 0b001, 0b111, 0b100, 0b111},   // 2
    {0b111, 0b001, 0b111, 0b001, 0b111},   // 3
    {0b101, 0b101, 0b111, 0b001, 0b001},   // 4
    {0b111, 0b100, 0b111, 0b001, 0b111},   // 5
    {0b111, 0b100, 0b111, 0b101, 0b111},   // 6
    {0b111, 0b001, 0b001, 0b010, 0b010},   // 7
    {0b111, 0b101, 0b111, 0b101, 0b111},   // 8
    {0b111, 0b101, 0b111, 0b001, 0b111},   // 9
    {0b111, 0b100, 0b110, 0b100, 0b111},   // E
    {0b110, 0b101, 0b110, 0b101, 0b101},   // R
    {0b000, 0b000, 0b111, 0b000, 0b000},   // -
    {0b000, 0b000, 0b000, 0b000, 0b010},   // .
};

uint8_t GlyphIndex(char c)
{
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c == 'E') { return GL_E; }
    if (c == 'R') { return GL_R; }
    if (c == '-') { return GL_DASH; }
    if (c == '.') { return GL_DOT; }
    return GL_BLANK;
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

void FrameGlyph(int row, int col, uint8_t g)
{
    if (g > GL_DOT) { return; }
    for (int r = 0; r < 5; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            FramePixel(row + r, col + c, (Glyph[g][r] >> (2 - c)) & 1);
        }
    }
}

// Up to three 3x5 glyphs side by side, starting at column 0 (cols 0-2, 4-6, 8-10)
void FrameText3(uint8_t g0, uint8_t g1, uint8_t g2)
{
    FrameGlyph(1, 0, g0);
    FrameGlyph(1, 4, g1);
    FrameGlyph(1, 8, g2);
}

// ---- Scrolling text (non-blocking; rendered by UpdateMatrix() in rows 1-5) ----

#define MARQUEE_MAX_COLS 100
uint8_t       MarqueeBits[MARQUEE_MAX_COLS];   // one byte per column, bit r = pixel in text row r (0-4)
int           MarqueeLen = 0;
int           MarqueeOffset = 0;
bool          MarqueeActive = false;
unsigned long MarqueeLastStep = 0;

void MarqueeStart(const char *s)
{
    MarqueeLen = 0;
    for (int i = 0; i < 12; i++) { MarqueeBits[MarqueeLen++] = 0; }        // lead-in: text enters from the right
    for (const char *p = s; *p != 0; p++)
    {
        uint8_t g = GlyphIndex(*p);
        for (int c = 0; c < 3 && MarqueeLen < MARQUEE_MAX_COLS; c++)
        {
            uint8_t bits = 0;
            if (g <= GL_DOT)
            {
                for (int r = 0; r < 5; r++) { if ((Glyph[g][r] >> (2 - c)) & 1) { bits |= (1 << r); } }
            }
            MarqueeBits[MarqueeLen++] = bits;
        }
        if (MarqueeLen < MARQUEE_MAX_COLS) { MarqueeBits[MarqueeLen++] = 0; }   // one blank column between glyphs
    }
    MarqueeOffset = 0;
    MarqueeActive = true;
    MarqueeLastStep = millis();
}

void DrawMarquee(unsigned long now)
{
    if (now - MarqueeLastStep >= MARQUEE_STEP_MS)
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

static inline int PosToCol(int q)
{
    int v = (q >> POS_SHIFT);                  // 0..1023
    if (v < 0) { v = 0; } else if (v > 1023) { v = 1023; }
    return (v * 12) / 1024;                    // 0..11
}

void DrawTrackingView(bool blink)
{
    // Rows 0-5: per motor, a target cursor row above a feedback cursor row.
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        int r = m * 2;
        if (Disable[m])
        {
            // disabled motor: both rows blink solid
            for (int c = 0; c < 12; c++) { FramePixel(r, c, blink); FramePixel(r + 1, c, blink); }
        }
        else
        {
            FramePixel(r,     PosToCol(Goal_q[m]), 1);
            FramePixel(r + 1, PosToCol(Feedback_q[m]), 1);
        }
    }

    // Row 6: drive bars, 4 pixels per motor (M1 cols 0-3, M2 cols 4-7, M3 cols 8-11)
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        int pwm = ReportPWM(m);                                 // 0..255
        int len = (pwm * 4 + 127) / 255;                        // 0..4
        for (int i = 0; i < len; i++) { FramePixel(6, m * 4 + i, 1); }
    }
}

#if IMU_SERIAL1
void DrawBubbleView(bool blink)
{
    // crosshair
    for (int c = 0; c < 12; c++) { FramePixel(3, c, 1); }
    for (int r = 0; r < 8; r++)  { FramePixel(r, 5, 1); }

    bool fresh = (millis() - ImuLastMillis) < 500;
    if (!fresh)
    {
        // no data: blink the centre
        FramePixel(3, 5, blink);
        return;
    }
    float cx = 5.5f + (ImuRoll / IMU_ROLL_FULL_SCALE) * 5.5f;    // +roll -> right
    float cy = 3.5f - (ImuPitch / IMU_PITCH_FULL_SCALE) * 3.5f;  // +pitch -> up
    int col = (int)(cx + 0.5f); if (col < 0) { col = 0; } else if (col > 11) { col = 11; }
    int row = (int)(cy + 0.5f); if (row < 0) { row = 0; } else if (row > 7)  { row = 7; }
    // 2x2 bubble, inverted over the crosshair so it is always visible
    for (int r = row - 1; r <= row; r++)
    {
        for (int c = col - 1; c <= col; c++)
        {
            if (r >= 0 && r <= 7 && c >= 0 && c <= 11) { Frame[r][c] = 1 - Frame[r][c]; }
        }
    }
}
#endif

void DrawStatusRow(bool blink, unsigned long now)
{
    bool packets = (now - LastPacketMillis) < 250;
    bool timeout = (now - LastPacketMillis) > COMMS_TIMEOUT_MS;
    bool overrun = IsrCyclesMax > ((F_CPU / PID_RATE_HZ) * 85UL / 100UL);   // ISR used >85% of its slot

    FramePixel(7, 0, packets);                 // SimHub / Utils packets arriving
    FramePixel(7, 1, timeout ? blink : 0);     // comms timeout -> low power mode
    FramePixel(7, 3, !Disable[0]);             // motor enable flags
    FramePixel(7, 4, !Disable[1]);
    FramePixel(7, 5, !Disable[2]);
    FramePixel(7, 7, overrun ? blink : 0);     // control loop overrun warning
#if IMU_SERIAL1
    FramePixel(7, 9, (now - ImuLastMillis) < 500);
#endif
#if ENABLE_WIFI
    FramePixel(7, 10, ClientUp ? 1 : (WifiUp ? blink : 0));   // solid = PC tool connected, blink = WiFi up, off = no WiFi
#endif
    FramePixel(7, 11, (now / 500) & 1);        // heartbeat
}


void UpdateMatrix()
{
    static unsigned long last = 0;
    unsigned long now = millis();
    if (now - last < (1000 / LED_FPS)) { return; }
    last = now;
    bool blink = (now / 250) & 1;

    FrameClear();

#if ENABLE_WIFI
    // Re-scroll the address (or the E-code) periodically while nothing is connected to the scope
    if (IP_SCROLL_REPEAT_MS > 0 && !ClientUp && !MarqueeActive && (now - LastIpScroll) >= IP_SCROLL_REPEAT_MS)
    {
        LastIpScroll = now;
        MarqueeStart(WifiText);
    }
#endif

    bool anyDisabled = Disable[0] || Disable[1] || Disable[2];
    if (MarqueeActive)
    {
        DrawMarquee(now);
    }
    else if (anyDisabled && ((now / 1000) & 1))
    {
        // alternate: one second of fault code ("E" + motor numbers), one second of the normal view
        uint8_t g[3] = {GL_E, GL_BLANK, GL_BLANK};
        uint8_t n = 1;
        for (uint8_t m = 0; m < NUM_MOTORS && n < 3; m++) { if (Disable[m]) { g[n++] = m + 1; } }
        FrameText3(g[0], g[1], g[2]);
    }
    else
    {
#if LED_VIEW == 1
        DrawBubbleView(blink);
#else
        DrawTrackingView(blink);
#endif
    }
    DrawStatusRow(blink, now);
    Matrix.renderBitmap(Frame, 8, 12);
}

#endif // LED_MATRIX

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
#if IMU_SERIAL1
    Serial1.begin(IMU_BAUD);
#endif

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
    ApplyPwmFrequencies();
    for (uint8_t m = 0; m < NUM_MOTORS; m++) { SetPwmQ(m, 0); }
    UpdatePIDLimits();
    UpdateSmoothingRate();

    // 2. Start the smoothed goals from wherever the motors actually are
    for (uint8_t m = 0; m < NUM_MOTORS; m++)
    {
        int fb = analogRead(FeedbackPin[m]);
        int fb10 = constrain(fb >> POS_SHIFT, InputClipMin[m], InputClipMax[m]);
        GoalAcc[m] = (long)fb10 << 12;
        Goal_q[m] = fb10 << POS_SHIFT;
        Feedback_q[m] = fb;
        PidLastPosition[m] = fb;
    }

    // 3. LED matrix (takes one of the remaining timers)
#if LED_MATRIX
    Matrix.begin();
    FrameClear();
    FrameText3(GL_R, 4, GL_BLANK);              // "R4" - stays up while WiFi connects
    Matrix.renderBitmap(Frame, 8, 12);
    delay(600);
#endif

    // 3b. WiFi - done before the motors are enabled because WiFi.begin() blocks for up to the timeout.
#if ENABLE_WIFI
    bool wifiOk = ConnectWifi();
    if (Serial)                                 // a terminal has the USB port open: print one text line
    {
        Serial.print(wifiOk ? "SMC3_R4 v" FW_VERSION " WiFi connected, IP " : "SMC3_R4 v" FW_VERSION " WiFi failed, code ");
        Serial.print(WifiText);
        Serial.print("\r\n");
    }
#if LED_MATRIX
    MarqueeStart(WifiText);                     // scrolls the full address (or E-code) once the loop is running
    LastIpScroll = millis();
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
}

void loop()
{
    unsigned long now = millis();
    uint32_t loopStartUs = micros();

    CheckSerial();

#if IMU_SERIAL1
    PollImu();
#endif

    if (now - LastMonitorMillis >= 20)          // every 20 ms, like the original
    {
        LastMonitorMillis = now;
        SendMonitorData();
    }

    // Comms timeout -> quarter power, as in the original (PowerScale 9 instead of 7)
    PowerScale = ((now - LastPacketMillis) > COMMS_TIMEOUT_MS) ? 9 : 7;

#if ENABLE_WIFI
    ServiceNetwork(now);
#endif

#if LED_MATRIX
    UpdateMatrix();
#endif

    uint32_t loopUs = micros() - loopStartUs;       // a long pass delays SimHub's commands (diagnostics)
    if (loopUs > LoopMaxUs) { LoopMaxUs = loopUs; }
}

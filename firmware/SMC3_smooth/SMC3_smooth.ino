//****************************************************************************************************************
// SMC3.ino is basic Motor PID driver designed for motion simulators with upto 3 motors (written for UNO R3)
//
// Based on SMC3 v0.70 by RufusDufus (xsimulator.net).  Changes in this version (Oct 2026):
//
//   1. TARGET SMOOTHING (new).  The position commanded by SimHub/SimTools arrives as a series of steps
//      (one per packet, typically every 10-20 ms).  The PID used to be slammed with each step.  The
//      commanded position is now passed through a first-order low-pass filter (and an optional
//      slew-rate limit) before the PID sees it, so direction changes are eased in over a few
//      milliseconds instead of being instantaneous.  See TARGET_SMOOTH_SHIFT / TARGET_MAX_RATE below.
//
//   2. BUG FIXES
//        - Motors 2 and 3 compared their feedback against CutoffLimitMin1 instead of their own limit.
//        - Motors 2 and 3 updated LastPosition twice per PID cycle, so their D term was ALWAYS zero
//          (Kd had no effect at all on motors 2 and 3).
//        - Motors 1 and 2 scaled the I and D terms differently from motor 3 (the "was >>6" / "was <<5"
//          edits), so the same Ki/Kd numbers meant very different things on different motors.
//
//   3. PID is now one shared routine for all three motors.  P scaling is IDENTICAL to the original, so
//      your Kp values carry over.  The I term is now a real integrator with anti-windup and the D term
//      is a filtered derivative-on-measurement (no kick when the target jumps).  Ki and Kd behave
//      differently from the old code - set them to 0 after flashing and re-tune upward if wanted.
//
//   4. The debug toggle of pin D8 every PID cycle is now optional (DEBUG_TIMING_PIN).
//
//   SERIAL PROTOCOL, COMMAND SET, BAUD RATE (500000) AND EEPROM LAYOUT ARE UNCHANGED.
//   SimHub and SMC3Utils talk to this exactly as they did to the original.
//
//****************************************************************************************************************

// Set to MODE1 for use with a typical H-Bride that requires PWM and 1 or 2 direction inputs
// Set to MODE2 for a 43A "Chinese" IBT-2 H-Bridge from e-bay or equiv

#define MODE2

// Uncomment the following line to reverse the direction of Motor 1.

//#define REVERSE_MOTOR1

// Uncomment ONE of the following lines to enable analogue input AN5 as a scaler for the motion values.

// #define ENABLE_POT_SCALING
// #define ENABLE_NON_LINEAR_POT_SCALING

// Uncomment the following line to enable the second software serial port.
// NOTE: This is currently not working - leave commented out until fixed!!!

// #define SECOND_SERIAL

//----------------------------------------------------------------------------------------------------------------
//  SMOOTHING SETTINGS (new)
//----------------------------------------------------------------------------------------------------------------

// TARGET_SMOOTH_SHIFT - low-pass filter applied to the commanded position before the PID.
//   0 = off (identical to the original SMC3 behaviour).
//   The filter time constant is about 2^shift PID cycles.  With the default 4 kHz PID loop (Fpid/1 in
//   SMC3Utils) that is:
//        4 -> ~4 ms     5 -> ~8 ms     6 -> ~16 ms     7 -> ~32 ms     8 -> ~64 ms
//   A step from SimHub is ~63% done after one time constant and ~95% done after three.  If you run
//   Fpid/2 or Fpid/4 in SMC3Utils multiply these times by 2 or 4.  Start at 6; go to 5 if the rig feels
//   lazy, 7 if it still feels jagged.

#define TARGET_SMOOTH_SHIFT 6

// TARGET_MAX_RATE - hard limit on how fast the commanded position may move, in feedback counts per
//   second (0 = off).  Full travel is 1023 counts, so 2000 = full travel in about half a second.
//   This caps peak velocity on big steps (crashes, game start) and gives a gentle move to centre
//   at power-up.  It applies on top of the low-pass filter.  Keep it <= 30000.

#define TARGET_MAX_RATE 0

// D_FILTER_SHIFT - extra low-pass on the derivative term to cut potentiometer noise (0 = off).
//   2..4 is sensible.  Works together with the Ks setting in SMC3Utils.

#define D_FILTER_SHIFT 3

// Fixed-point scaling of the I and D terms (same as the original SMC3 motor 3 code).  Leave alone.

#define I_SHIFT 6
#define D_SHIFT 5

// Uncomment to toggle pin D8 every PID cycle so loop timing can be checked on an oscilloscope.

//#define DEBUG_TIMING_PIN

#if TARGET_MAX_RATE > 30000
#error "TARGET_MAX_RATE must be 30000 or less"
#endif

//----------------------------------------------------------------------------------------------------------------

//    COMMAND SET:
//
//    []              		Drive all motors to defined stop positions and hold there
//    [Axx],[Bxx],[Cxx]         Send position updates for Motor 1,2,3 where xx is the binary position limitted to range 0-1024
//    [Dxx],[Exx],[Fxx]		Send the Kp parameter for motor 1,2,3 where xx is the Kp value multiplied by 100
//    [Gxx],[Hxx],[Ixx]		Send the Ki parameter for motor 1,2,3 where xx is the Ki value multiplied by 100
//    [Jxx],[Kxx],[Lxx]		Send the Kd parameter for motor 1,2,3 where xx is the Kd value multiplied by 100
//    [Mxx],[Nxx],[Oxx]		Send the Ks parameter for motor 1,2,3 where xx is the Ks value
//    [Pxy],[Qxy],[Rxy]		Send the PWMmin and PWMmax values x is the PWMmin and y is PWMmax each being in range 0-255
//    [Sxy],[Txy],[Uxy]         Send the Motor Min/Max Limits (x) and Input Min/Max Limits (y) (Note same value used for Min and Max)
//    [Vxy],[Wxy],[Xxy]         Send the Feedback dead zone (x) and the PWM reverse duty (y) for each motor
//    [rdx]          		Read a value from the controller where x is the code for the parameter to read
//    [ena]			Enable all motors
//    [sav]			Save parameters to non-volatile memory
//    [ver]                     Send the SMC3 software version
//

//    Arduino UNO Pinouts Used
//
//    9 - Motor 1 PWM
//    10 - Motor 2 PWM
//    11 - Motor 3 PWM
//    2 - Motor 1 H-Bridge ENA
//    3 - Motor 1 H-Bridge ENB
//    4 - Motor 2 H-Bridge ENA
//    5 - Motor 2 H-Bridge ENB
//    6 - Motor 3 H-Bridge ENA
//    7 - Motor 3 H-Bridge ENB
//    A0 - Motor 1 Feedback
//    A1 - Motor 2 Feedback
//    A2 - Motor 3 Feedback
//    A5 - Motion scaler pot input



// BOF preprocessor bug prevention - leave me at the top of the arduino-code
#if 1
__asm volatile ("nop");
#endif

#include <EEPROM.h>
#include <SoftwareSerial.h>

// defines for setting and clearing register bits
#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define COM0 0                          // hardware Serial Port
#define COM1 1                          // Software Serial Port
#define START_BYTE '['                  // Start Byte for serial commands
#define END_BYTE ']'                    // End Byte for serial commands
#define PROCESS_PERIOD_uS 250           // 244 -> 4096 per second approx
#define TIMER_UPPER_LIMIT 4294966000    // Used to check for timer wraparound
#define TIMER_LOWER_LIMIT 1000          // Used to check for timer wraparound


unsigned long TimesUp=0;           // Counter used to see if it's time to calculate next PID update

int Feedback1 = 512;
int Feedback2 = 512;
int Feedback3 = 512;
int Target1 = 512;                 // Raw commanded position (as received from SimHub, after clipping)
int Target2 = 512;
int Target3 = 512;
int Goal1 = 512;                   // Smoothed commanded position - this is what the PID actually tracks
int Goal2 = 512;
int Goal3 = 512;
long GoalAcc[3] = {512L << 8, 512L << 8, 512L << 8};   // 24.8 fixed-point accumulators for the smoothed targets
long SlewStepQ8 = 0;               // Max change of the goal per PID cycle, in 1/256 counts (0 = no limit)
int PotInput = 512;

unsigned int RxByte[2]={0};         // Current byte received from each of the two comm ports
int BufferEnd[2]={-1};              // Rx Buffer end index for each of the two comm ports
unsigned int RxBuffer[5][2]={0};    // 5 byte Rx Command Buffer for each of the two comm ports
unsigned long LoopCount	= 0;        // unsigned 32bit, 0 to 4,294,967,295 to count times round loop
unsigned long LastCount = 0;        // loop counter the last time it was read so can calc delta
byte errorcount	= 0;                // serial receive error detected by invalid packet start/end bytes
unsigned int CommsTimeout = 0;      // used to reduce motor power if there has been no comms for a while
byte PowerScale = 7;                // used to divide(shift) PID result changes when in low power

//
//
//                    +----+              +------+
//                  +-|PWR |--------------| USB  |-----+
//                  | |    |              |      |     |
//                  | +----+              +------+    o|
//                  |                                 o|
//                  |                                 o|AREF
//                  |                                 o|GND
//                NC|o                                o|13    TTL (software) Serial
//             IOREF|o   +----+                       o|12    TTL (software) Serial
//             RESET|o   |    |                       o|11~   PWM Motor 3
//              3.3V|o   |    |                       o|10~   PWM Motor 2
//                5V|o   |    |    Arduino            o|9~    PWM Motor 1
//               GND|o   |    |      UNO              o|8     (optional timing debug pin)
//               GND|o   |    |                        |
//               VIN|o   |    |                       o|7      Motor 3 ENB
//                  |    |    |                       o|6~     Motor 3 ENA
//  Motor 1 Pot   A0|o   |    |                       o|5~     Motor 2 ENB
//  Motor 2 Pot   A1|o   |    |                       o|4      Motor 2 ENA
//  Motor 3 Pot   A2|o   |    |                       o|3~     Motor 1 ENB
//                A3|o   |    |                       o|2      Motor 1 ENA
//                A4|o   |    |                       o|1
//  Motion Scaler A5|o   +-/\-+                       o|0
//                  +-\                      /---------+
//                     \--------------------/
//
//

int OutputPort =PORTD;         // read the current port D bit mask
const int ENApin1 =2;          // ENA output pin for Motor H-Bridge 1 (ie PortD bit position)
const int ENBpin1 =3;          // ENB output pin for Motor H-Bridge 1 (ie PortD bit position)
const int ENApin2 =4;          // ENA output pin for Motor H-Bridge 2 (ie PortD bit position)
const int ENBpin2 =5;          // ENB output pin for Motor H-Bridge 2 (ie PortD bit position)
const int ENApin3 =6;          // ENA output pin for Motor H-Bridge 3 (ie PortD bit position)
const int ENBpin3 =7;          // ENB output pin for Motor H-Bridge 3 (ie PortD bit position)
const int PWMpin1 =9;          // PWM output pin for Motor 1
const int PWMpin2 =10;         // PWM output pin for Motor 2
const int PWMpin3 =11;         // PWM output pin for Motor 3
const int FeedbackPin1 = A0;   // Motor 1 feedback pin
const int FeedbackPin2 = A1;   // Motor 2 feedback pin
const int FeedbackPin3 = A2;   // Motor 3 feedback pin
const int PotInputPin = A5;  // User adjustable POT used to scale the motion (if enabled)

int DeadZone1 = 0;  // feedback deadzone
int DeadZone2 = 0;  // feedback deadzone
int DeadZone3 = 0;  // feedback deadzone

int CenterOffset1 = 0;    // Adjust center offset of feedback position
int CenterOffset2 = 0;    // Adjust center offset of feedback position
int CenterOffset3 = 0;    // Adjust center offset of feedback position

int CutoffLimitMax1 = 1000;    // The position beyond which the motors are disabled
int CutoffLimitMax2 = 1000;    // The position beyond which the motors are disabled
int CutoffLimitMax3 = 1000;    // The position beyond which the motors are disabled
int CutoffLimitMin1 = 23;      // The position beyond which the motors are disabled
int CutoffLimitMin2 = 23;      // The position beyond which the motors are disabled
int CutoffLimitMin3 = 23;      // The position beyond which the motors are disabled

int InputClipMax1 = 923;    // The input position beyond which the target input is clipped
int InputClipMax2 = 923;    // The input position beyond which the target input is clipped
int InputClipMax3 = 923;    // The input position beyond which the target input is clipped
int InputClipMin1 = 100;      // The input position beyond which the target input is clipped
int InputClipMin2 = 100;      // The input position beyond which the target input is clipped
int InputClipMin3 = 100;      // The input position beyond which the target input is clipped


int LiftFactor1 = 0;   // was 10 - Increase PWM when driving motor in direction it has to work harder
int LiftFactor2 = 0;   // was 10 - Increase PWM when driving motor in direction it has to work harder

int PIDProcessDivider = 1;  // divider for the PID process timer
int PIDProcessCounter = 0;
int SerialFeedbackCounter = 0;
int SerialFeedbackEnabled = 0;
int SerialFeedbackPort = 0;

int Ks1 = 1;
long Kp1_x100 = 100;		//initial value
long Ki1_x100 = 40;
long Kd1_x100 = 40;
int Ks2 = 1;
long Kp2_x100 = 420;
long Ki2_x100 = 40;
long Kd2_x100 = 40;
int Ks3 = 1;
long Kp3_x100 = 420;
long Ki3_x100 = 40;
long Kd3_x100 = 40;
int PWMout1 = 0;
int PWMout2 = 0;
int PWMout3 = 0;

// Per-motor PID state (index 0,1,2 = motor 1,2,3)
long PidCumError[3]     = {0, 0, 0};       // integral accumulator
long PidIntegralLimit[3] = {0, 0, 0};      // anti-windup clamp for PidCumError, derived from Ki
int  PidLastPosition[3] = {512, 512, 512}; // previous feedback, for derivative-on-measurement
long PidDTerm_x100[3]   = {0, 0, 0};       // filtered derivative term
int  PidKdFilterCount[3] = {0, 0, 0};      // counts PID cycles between derivative samples (Ks)

int Disable1 = 1;                     //Motor stop flag
int Disable2 = 1;                     //Motor stop flag
int Disable3 = 1;                     //Motor stop flag
int PWMoffset1 = 50;
int PWMoffset2 = 50;
int PWMoffset3 = 50;
int PWMmax1 = 100;
int PWMmax2 = 100;
int PWMmax3 = 100;
int PWMrev1 = 200;
int PWMrev2 = 200;
int PWMrev3 = 200;
unsigned int Timer1FreqkHz = 25;   // PWM freq used for Motor 1 and 2
unsigned int Timer2FreqkHz = 31;   // PWM freq used for Motor 3

#ifdef SECOND_SERIAL
SoftwareSerial mySerial(12, 13);    // RX, TX
#endif

//****************************************************************************************************************
//    Setup the PWM pin frequencies
//
//****************************************************************************************************************


void setPwmFrequency(int pin, int divisor)
{
    byte mode;
    if(pin == 5 || pin == 6 || pin == 9 || pin == 10)
    {
        switch(divisor)
        {
            case 1: mode = 0x01; break;
            case 8: mode = 0x02; break;
            case 64: mode = 0x03; break;
            case 256: mode = 0x04; break;
            case 1024: mode = 0x05; break;
            default: return;
        }
        if(pin == 5 || pin == 6)
        {
            TCCR0B = TCCR0B & 0b11111000 | mode;
        }
        else
        {
            TCCR1B = TCCR1B & 0b11111000 | mode;
        }
    }
    else
    {
        if(pin == 3 || pin == 11)
        {
            switch(divisor)
            {
                case 1: mode = 0x01; break;
                case 8: mode = 0x02; break;
                case 32: mode = 0x03; break;
                case 64: mode = 0x04; break;
                case 128: mode = 0x05; break;
                case 256: mode = 0x06; break;
                case 1024: mode = 0x7; break;
                default: return;
            }
            TCCR2B = TCCR2B & 0b11111000 | mode;
        }
    }
}

void InitialisePWMTimer1(unsigned int Freq)     // Used for pins 9 and 10
{
	uint8_t wgm = 8;    //setting the waveform generation mode to 8 - PWM Phase and Freq Correct
	TCCR1A = (TCCR1A & B11111100) | (wgm & B00000011);
	TCCR1B = (TCCR1B & B11100111) | ((wgm & B00001100) << 1);
	TCCR1B = (TCCR1B & B11111000) | 0x01;    // Set the prescaler to minimum (ie divide by 1)

	unsigned int CountTOP;

        CountTOP = (F_CPU / 2) / Freq;    // F_CPU is the oscillator frequency - Freq is the wanted PWM freq

        // Examples of CountTOP:
	//  400 = 20000Hz  -> 8MHz / Freq = CountTOP
        //  320 = 25000Hz
        //  266 = 30075Hz

	ICR1 = CountTOP;          // Set the TOP of the count for the PWM
}


void InitialisePWMTimer2(unsigned int Freq)
{
    int Timer2DivideMode;

    if (Freq>=15000)
    {
       Timer2DivideMode=0x01;   // Anything above 15000 set divide by 1 which gives 31250Hz PWM freq
    }
    else
    {
       Timer2DivideMode=0x02;   // Anything below 15000 set divide by 8 (mode 2) which gives 3906Hz PWM freq
    }
    TCCR2B = TCCR2B & 0b11111000 | Timer2DivideMode;
}


void MyPWMWrite(uint8_t pin, uint8_t val)
{
    #define OCR1A_MEM      0x88
    #define OCR1B_MEM      0x8A

    pinMode(pin, OUTPUT);

    //casting "val" to be larger so that the final value (which is the partially
    //the result of multiplying two potentially high value int16s) will not truncate
    uint32_t tmp = val;

    if (val == 0)
        digitalWrite(pin, LOW);
    else if (val == 255)
        digitalWrite(pin, HIGH);
    else
    {
        uint16_t regLoc16 = 0;
        uint16_t top;

        switch(pin)
        {
            case 9:
                sbi(TCCR1A, COM1A1);
                regLoc16 = OCR1A_MEM;
                top = ICR1;                  //Timer1_GetTop();
                break;
            case 10:
                sbi(TCCR1A, COM1B1);
                regLoc16 = OCR1B_MEM;
                top = ICR1;                  //Timer1_GetTop();
                break;
        }
        tmp=(tmp*top)/255;
        _SFR_MEM16(regLoc16) = tmp;       //(tmp*top)/255;
    }
}


void DisableMotor1();
void DisableMotor2();
void DisableMotor3();


//****************************************************************************************************************
//    Derived constants for the smoothing and the PID anti-windup.
//    Called after the EEPROM is read and whenever Ki or the PID divider change over serial, so the
//    4 kHz loop never has to do a 32-bit divide.
//
//****************************************************************************************************************

void UpdateSmoothingRate()
{
    // counts per second -> 1/256 counts per PID cycle
    SlewStepQ8 = ((long)TARGET_MAX_RATE * 256L * PROCESS_PERIOD_uS / 1000000L) * PIDProcessDivider;
    if (TARGET_MAX_RATE > 0 && SlewStepQ8 < 1) { SlewStepQ8 = 1; }   // never let a non-zero rate round down to "frozen"
}

void UpdatePIDLimits()
{
    // Clamp the integral so the I term on its own can never ask for more than full PWM (255 after >>7).
    // (255 << (I_SHIFT + 7)) / Ki_x100
    PidIntegralLimit[0] = (Ki1_x100 > 0) ? (255L << (I_SHIFT + 7)) / Ki1_x100 : 0;
    PidIntegralLimit[1] = (Ki2_x100 > 0) ? (255L << (I_SHIFT + 7)) / Ki2_x100 : 0;
    PidIntegralLimit[2] = (Ki3_x100 > 0) ? (255L << (I_SHIFT + 7)) / Ki3_x100 : 0;
}


//****************************************************************************************************************
//    Arduino setup subroutine called at startup/reset
//
//****************************************************************************************************************


void setup()
{
    Serial.begin(500000);     //115200
    // set the data rate for the SoftwareSerial port
#ifdef SECOND_SERIAL
    mySerial.begin(115200);
#endif
    OutputPort=PORTD;
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(ENApin1, OUTPUT);
    pinMode(ENBpin1, OUTPUT);
    pinMode(ENApin2, OUTPUT);
    pinMode(ENBpin2, OUTPUT);
    pinMode(ENApin3, OUTPUT);
    pinMode(ENBpin3, OUTPUT);
    pinMode(PWMpin1, OUTPUT);
    pinMode(PWMpin2, OUTPUT);
    pinMode(PWMpin3, OUTPUT);
#ifdef DEBUG_TIMING_PIN
    pinMode(8, OUTPUT);
#endif
    MyPWMWrite(PWMpin1,0);      //analogWrite(PWMpin1, 0);
    MyPWMWrite(PWMpin2,0);      //analogWrite(PWMpin2, 0);
    analogWrite(PWMpin3, 0);
    DisableMotor1();
    DisableMotor2();
    DisableMotor3();

    // Note that the base frequency for pins 3, 9, 10, and 11 is 31250 Hz
    //setPwmFrequency(PWMpin1, 1);     // Divider can be 1, 8, 64, 256
    //setPwmFrequency(PWMpin2, 1);     // Divider can be 1, 8, 64, 256
    //setPwmFrequency(PWMpin3, 1);     // Divider can be 1, 8, 64, 256
    InitialisePWMTimer1(Timer1FreqkHz * 1000);        // PWM Freq for Motor 1 and 2 can be set quite precisely - pass the desired freq and nearest selected
    InitialisePWMTimer2(Timer2FreqkHz * 1000);        // Motor 3 PWM Divider can only be 1, 8, 64, 256 giving Freqs 31250,3906, 488 and 122 Hz

    //CLKPR = 0x80;   // These two lines halve the processor clock
    //CLKPR = 0x01;

    // set analogue prescale to 16
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    cbi(ADCSRA,ADPS0);
}

void WriteEEPRomWord(int address, int intvalue)
{
    int low,high;
    high=intvalue/256;
    low=intvalue-(256*high);
    EEPROM.write(address,high);
    EEPROM.write(address+1,low);
}

int ReadEEPRomWord(int address)
{
    int low,high, returnvalue;
    high=EEPROM.read(address);
    low=EEPROM.read(address+1);
    returnvalue=(high*256)+low;
    return returnvalue;
}

//****************************************************************************************************************
//    Write all configurable parameters to EEPROM   (layout unchanged from the original)
//
//****************************************************************************************************************


void WriteEEProm()
{
    EEPROM.write(0,114);
    EEPROM.write(1,CutoffLimitMin1);
    EEPROM.write(2,InputClipMin1);

    EEPROM.write(5,DeadZone1);
    EEPROM.write(6,CutoffLimitMin2);
    EEPROM.write(7,InputClipMin2);

    EEPROM.write(10,DeadZone2);
    WriteEEPRomWord(11,Kp1_x100);    // **** Even though these variables are long the actual values stored are only 0 to 1000 so OK ****
    WriteEEPRomWord(13,Ki1_x100);
    WriteEEPRomWord(15,Kd1_x100);
    WriteEEPRomWord(17,Kp2_x100);
    WriteEEPRomWord(19,Ki2_x100);
    WriteEEPRomWord(21,Kd2_x100);
    EEPROM.write(23,PWMoffset1);
    EEPROM.write(24,PWMoffset2);
    EEPROM.write(25,PWMmax1);
    EEPROM.write(26,PWMmax2);
    EEPROM.write(27,PWMrev1);
    EEPROM.write(28,PWMrev2);
    EEPROM.write(29,PWMrev3);


    EEPROM.write(31,CutoffLimitMin3);
    EEPROM.write(32,InputClipMin3);

    EEPROM.write(35,DeadZone3);
    WriteEEPRomWord(36,Kp3_x100);
    WriteEEPRomWord(38,Ki3_x100);
    WriteEEPRomWord(40,Kd3_x100);
    EEPROM.write(42,PWMoffset3);
    EEPROM.write(43,PWMmax3);

    WriteEEPRomWord(44,Ks1);
    WriteEEPRomWord(46,Ks2);
    WriteEEPRomWord(48,Ks3);

    EEPROM.write(50,constrain(PIDProcessDivider,1,10));
    EEPROM.write(51,Timer1FreqkHz);
    EEPROM.write(52,Timer2FreqkHz);
}

//****************************************************************************************************************
//    Read all configurable parameters from the EEPROM.
//
//****************************************************************************************************************


void ReadEEProm()
{
    int evalue = EEPROM.read(0);
    if(evalue != 114) //EEProm was not set before, set default values
    {
        WriteEEProm();
        return;
    }
    CutoffLimitMin1 = EEPROM.read(1);
    InputClipMin1 = EEPROM.read(2);
    CutoffLimitMax1 = 1023-CutoffLimitMin1;
    InputClipMax1 = 1023-InputClipMin1;

    DeadZone1=EEPROM.read(5);
    CutoffLimitMin2 = EEPROM.read(6);
    InputClipMin2 = EEPROM.read(7);
    CutoffLimitMax2 = 1023-CutoffLimitMin2;
    InputClipMax2 = 1023-InputClipMin2;

    DeadZone2=EEPROM.read(10);
    Kp1_x100=ReadEEPRomWord(11);
    Ki1_x100=ReadEEPRomWord(13);
    Kd1_x100=ReadEEPRomWord(15);
    Kp2_x100=ReadEEPRomWord(17);
    Ki2_x100=ReadEEPRomWord(19);
    Kd2_x100=ReadEEPRomWord(21);
    PWMoffset1=EEPROM.read(23);
    PWMoffset2=EEPROM.read(24);
    PWMmax1=EEPROM.read(25);
    PWMmax2=EEPROM.read(26);
    PWMrev1=EEPROM.read(27);
    PWMrev2=EEPROM.read(28);
    PWMrev3=EEPROM.read(29);


    CutoffLimitMin3 = EEPROM.read(31);
    InputClipMin3 = EEPROM.read(32);
    CutoffLimitMax3 = 1023-CutoffLimitMin3;
    InputClipMax3 = 1023-InputClipMin3;

    DeadZone3=EEPROM.read(35);
    Kp3_x100=ReadEEPRomWord(36);
    Ki3_x100=ReadEEPRomWord(38);

    Kd3_x100=ReadEEPRomWord(40);
    PWMoffset3=EEPROM.read(42);
    PWMmax3=EEPROM.read(43);
    Ks1=ReadEEPRomWord(44);
    Ks2=ReadEEPRomWord(46);
    Ks3=ReadEEPRomWord(48);

    PIDProcessDivider=EEPROM.read(50);
    PIDProcessDivider=constrain(PIDProcessDivider,1,10);
    Timer1FreqkHz=EEPROM.read(51);
    Timer2FreqkHz=EEPROM.read(52);
}



//****************************************************************************************************************
//    Send two bytes via serial in response to a request for information
//
//****************************************************************************************************************

void SendTwoValues(int id, int v1, int v2, int ComPort)
{
    if (ComPort==0)
    {
        Serial.write(START_BYTE);
        Serial.write(id);
        Serial.write(v1);
        Serial.write(v2);
        Serial.write(END_BYTE);
    }
    else
    {
#ifdef SECOND_SERIAL
        mySerial.write(START_BYTE);
        mySerial.write(id);
        mySerial.write(v1);
        mySerial.write(v2);
        mySerial.write(END_BYTE);
#endif
    }
}


//****************************************************************************************************************
//    Send a single word value via serial in response to a request for information
//
//****************************************************************************************************************

void SendValue(int id, int value, int ComPort)
{
    int low,high;

    high=value/256;
    low=value-(high*256);

    if (ComPort==0)
    {
        Serial.write(START_BYTE);
        Serial.write(id);
        Serial.write(high);
        Serial.write(low);
        Serial.write(END_BYTE);
    }
    else
    {
#ifdef SECOND_SERIAL
        mySerial.write(START_BYTE);
        mySerial.write(id);
        mySerial.write(high);
        mySerial.write(low);
        mySerial.write(END_BYTE);
#endif
    }
}


//****************************************************************************************************************
//    Calculate how many PID calculations have been performed since last requested
//
//****************************************************************************************************************

int DeltaLoopCount()
{
    unsigned long Delta;

    if ( (LastCount==0) || ((LoopCount-LastCount)>32000) )
    {
        Delta = 0;
        LastCount = LoopCount;
    }
    else
    {
         Delta = LoopCount-LastCount;
         LastCount  = LoopCount;
    }
    return (int)Delta;
}


//****************************************************************************************************************
//    Process the incoming serial commands from the specified comm port
//
//****************************************************************************************************************


void ParseCommand(int ComPort)
{
    CommsTimeout = 0;    // reset the comms timeout counter to indicate we are getting packets

    switch (RxBuffer[0][ComPort])
    {
        case 'A':
            Target1=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
#ifdef REVERSE_MOTOR1
            Target1=1023-Target1;
#endif

#ifdef ENABLE_POT_SCALING
            if (PotInput < 25)
            {
                Target1 = 512;     // Center Motor1
            }
            else if (PotInput < 1000)
            {
                Target1 = int(float(Target1-512) * (0.0 + float(PotInput) * 0.001)) + 512;
            }
#endif

#ifdef ENABLE_NON_LINEAR_POT_SCALING
            if (PotInput < 25)
            {
                Target1 = 512;     // Center Motor1
            }
            else if (PotInput < 512)
            {
                Target1 = (Target1 - 512) << 1;
                if (Target1 < 0)
                {
                    Target1 = int((-Target1 - float((double(Target1) * double(Target1))) * 0.000488) * float(PotInput * 0.001953));
                    Target1 = (-Target1 >> 1) + 512;
                }
                else
                {
                    Target1 = int((Target1 - float((double(Target1) * double(Target1))) * 0.000488) * float(PotInput * 0.001953));
                    Target1 = (Target1 >> 1) + 512;
                }
            }
            else if (PotInput < 1000)
            {
                Target1 = (Target1 - 512) << 1;
                if (Target1 < 0)
                {
                    Target1 = int(-Target1 - float((double(Target1) * double(Target1) * double(1024 - PotInput))) * 0.000000953674);
                    Target1= (-Target1 >> 1) + 512;
                }
                else
                {
                    Target1 = int(Target1 - float((double(Target1) * double(Target1) * double(1024 - PotInput))) * 0.000000953674);
                    Target1= (Target1 >> 1) + 512;
                }
            }
#endif

            if (Target1>InputClipMax1) { Target1=InputClipMax1; }
            else if (Target1<InputClipMin1) { Target1=InputClipMin1; }
            break;
        case 'B':
            Target2=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
#ifdef ENABLE_POT_SCALING
            if (PotInput < 25)
            {
                Target2 = 512;     // Center Motor1
            }
            else if (PotInput < 1000)
            {
                Target2 = int(float(Target2-512) * (0.0 + float(PotInput) * 0.001)) + 512;
            }
#endif

#ifdef ENABLE_NON_LINEAR_POT_SCALING
            if (PotInput < 25)
            {
                Target2 = 512;     // Center Motor1
            }
            else if (PotInput < 512)
            {
                Target2 = (Target2 - 512) << 1;
                if (Target2 < 0)
                {
                    Target2 = int((-Target2 - float((double(Target2) * double(Target2))) * 0.000488) * float(PotInput * 0.001953));
                    Target2 = (-Target2 >> 1) + 512;
                }
                else
                {
                    Target2 = int((Target2 - float((double(Target2) * double(Target2))) * 0.000488) * float(PotInput * 0.001953));
                    Target2 = (Target2 >> 1) + 512;
                }
            }
            else if (PotInput < 1000)
            {
                Target2 = (Target2 - 512) << 1;
                if (Target2 < 0)
                {
                    Target2 = int(-Target2 - float((double(Target2) * double(Target2) * double(1024 - PotInput))) * 0.000000953674);
                    Target2= (-Target2 >> 1) + 512;
                }
                else
                {
                    Target2 = int(Target2 - float((double(Target2) * double(Target2) * double(1024 - PotInput))) * 0.000000953674);
                    Target2= (Target2 >> 1) + 512;
                }
            }
#endif

            if (Target2>InputClipMax2) { Target2=InputClipMax2; }
            else if (Target2<InputClipMin2) { Target2=InputClipMin2; }
            break;
        case 'C':
            Target3=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            if (Target3>InputClipMax3) { Target3=InputClipMax3; }
            else if (Target3<InputClipMin3) { Target3=InputClipMin3; }
            break;
        case 'r':
            if (RxBuffer[1][ComPort]=='d')      // rd - Read a value from the ArduinoPID - next byte identifies value to read
            {
                switch (RxBuffer[2][ComPort])
                {
                    case 'A':
#ifdef REVERSE_MOTOR1
                        SendTwoValues('A',(1023-Feedback1)/4,(1023-Goal1)/4,ComPort);
#else
                        SendTwoValues('A',Feedback1/4,Goal1/4,ComPort);
#endif
                        break;
                    case 'B':
                        SendTwoValues('B',Feedback2/4,Goal2/4,ComPort);
                        break;
                    case 'C':
                        SendTwoValues('C',Feedback3/4,Goal3/4,ComPort);
                        break;
                    case 'a':
                        SendTwoValues('a',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout1,0,255),ComPort);
                        break;
                    case 'b':
                        SendTwoValues('b',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout2,0,255),ComPort);
                        break;
                    case 'c':
                        SendTwoValues('c',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout3,0,255),ComPort);
                        break;
                    case 'D':
                        SendValue('D',Kp1_x100,ComPort);
                        break;
                    case 'E':
                        SendValue('E',Kp2_x100,ComPort);
                        break;
                    case 'F':
                        SendValue('F',Kp3_x100,ComPort);
                        break;
                    case 'G':
                        SendValue('G',Ki1_x100,ComPort);
                        break;
                    case 'H':
                        SendValue('H',Ki2_x100,ComPort);
                        break;
                    case 'I':
                        SendValue('I',Ki3_x100,ComPort);
                        break;
                    case 'J':
                        SendValue('J',Kd1_x100,ComPort);
                        break;
                    case 'K':
                        SendValue('K',Kd2_x100,ComPort);
                        break;
                    case 'L':
                        SendValue('L',Kd3_x100,ComPort);
                        break;
                    case 'M':
                        SendValue('M',Ks1,ComPort);
                        break;
                    case 'N':
                        SendValue('N',Ks2,ComPort);
                        break;
                    case 'O':
                        SendValue('O',Ks3,ComPort);
                        break;
                    case 'P':
                        SendTwoValues('P',PWMoffset1,PWMmax1,ComPort);
                        break;
                    case 'Q':
                        SendTwoValues('Q',PWMoffset2,PWMmax2,ComPort);
                        break;
                    case 'R':
                        SendTwoValues('R',PWMoffset3,PWMmax3,ComPort);
                        break;
                    case 'S':
                        SendTwoValues('S',CutoffLimitMin1,InputClipMin1,ComPort);
                        break;
                    case 'T':
                        SendTwoValues('T',CutoffLimitMin2,InputClipMin2,ComPort);
                        break;
                    case 'U':
                        SendTwoValues('U',CutoffLimitMin3,InputClipMin3,ComPort);
                        break;
                    case 'V':
                        SendTwoValues('V',DeadZone1,PWMrev1,ComPort);
                        break;
                    case 'W':
                        SendTwoValues('W',DeadZone2,PWMrev2,ComPort);
                        break;
                    case 'X':
                        SendTwoValues('X',DeadZone3,PWMrev3,ComPort);
                        break;
                    case 'Y':
                        SendTwoValues('Y',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),0,ComPort);  // Second byte not yet used
                        break;
                    case 'Z':
                        SendValue('Z',DeltaLoopCount(),ComPort);
//                        SendValue('Z',errorcount,ComPort);            // *** TEMP CODE FOR ETESTING
                        break;
                    case '~':
                        SendTwoValues('~',Timer1FreqkHz,Timer2FreqkHz,ComPort);  // PWM Frequencies to set
                        break;
                     case '?':
                        break;
//                    default:
                }
            }
            break;
        case 'D':
            Kp1_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'E':
            Kp2_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'F':
            Kp3_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'G':
            Ki1_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            UpdatePIDLimits();
            break;
        case 'H':
            Ki2_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            UpdatePIDLimits();
            break;
        case 'I':
            Ki3_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            UpdatePIDLimits();
            break;
        case 'J':
            Kd1_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'K':
            Kd2_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'L':
            Kd3_x100=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'M':
            Ks1=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'N':
            Ks2=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'O':
            Ks3=(RxBuffer[1][ComPort]*256)+RxBuffer[2][ComPort];
            break;
        case 'P':
            PWMoffset1=RxBuffer[1][ComPort];
            PWMmax1=RxBuffer[2][ComPort];
            break;
        case 'Q':
            PWMoffset2=RxBuffer[1][ComPort];
            PWMmax2=RxBuffer[2][ComPort];
            break;
        case 'R':
            PWMoffset3=RxBuffer[1][ComPort];
            PWMmax3=RxBuffer[2][ComPort];
            break;
        case 'S':
            CutoffLimitMin1=RxBuffer[1][ComPort];
            CutoffLimitMax1=1023-CutoffLimitMin1;
            InputClipMin1=RxBuffer[2][ComPort];
            InputClipMax1=1023-InputClipMin1;
            break;
        case 'T':
            CutoffLimitMin2=RxBuffer[1][ComPort];
            CutoffLimitMax2=1023-CutoffLimitMin2;
            InputClipMin2=RxBuffer[2][ComPort];
            InputClipMax2=1023-InputClipMin2;
            break;
        case 'U':
            CutoffLimitMin3=RxBuffer[1][ComPort];
            CutoffLimitMax3=1023-CutoffLimitMin3;
            InputClipMin3=RxBuffer[2][ComPort];
            InputClipMax3=1023-InputClipMin3;
            break;
        case 'V':
            DeadZone1=RxBuffer[1][ComPort];
            PWMrev1=RxBuffer[2][ComPort];
            break;
        case 'W':
            DeadZone2=RxBuffer[1][ComPort];
            PWMrev2=RxBuffer[2][ComPort];
            break;
        case 'X':
            DeadZone3=RxBuffer[1][ComPort];
            PWMrev3=RxBuffer[2][ComPort];
            break;
        case 'Z':
            PIDProcessDivider=constrain(RxBuffer[1][ComPort],1,10);
            // ???=RxBuffer[2][ComPort];   // second byte not yet used
            UpdateSmoothingRate();
            break;
        case '~':
            Timer1FreqkHz=RxBuffer[1][ComPort];
            Timer2FreqkHz=RxBuffer[2][ComPort];
            InitialisePWMTimer1(Timer1FreqkHz * 1000);
            InitialisePWMTimer2(Timer2FreqkHz * 1000);
            break;
        case 'm':
            if (RxBuffer[1][ComPort]=='o' && RxBuffer[2][ComPort]=='1')   // Monitor motor 1 (auto sends position and feedback data to host
            {
                SerialFeedbackEnabled= (ComPort << 4) | 1;   // Motor 1
            }
            if (RxBuffer[1][ComPort]=='o' && RxBuffer[2][ComPort]=='2')   // Monitor motor 2 (auto sends position and feedback data to host
            {
                SerialFeedbackEnabled= (ComPort << 4) | 2;   // Motor 2
            }
            if (RxBuffer[1][ComPort]=='o' && RxBuffer[2][ComPort]=='3')   // Monitor motor 3 (auto sends position and feedback data to host
            {
                SerialFeedbackEnabled= (ComPort << 4) | 3;   // Motor 3
            }
            if (RxBuffer[1][ComPort]=='o' && RxBuffer[2][ComPort]=='0')   // Switch off auto monitoring data feedback
            {
                SerialFeedbackEnabled=0;
            }
            break;
        case 's':
            if (RxBuffer[1][ComPort]=='a' && RxBuffer[2][ComPort]=='v')   // Save all settings to EEPROM
            {
                WriteEEProm();
            }
            break;
        case 'v':
            if (RxBuffer[1][ComPort]=='e' && RxBuffer[2][ComPort]=='r')   // Send back the SMC3 software version
            {
                SendValue('v',70,ComPort);     // Software Version - divide by 100 to get version - ie 101= ver1.01
            }
            break;
        case 'e':
            if (RxBuffer[1][ComPort]=='n' && RxBuffer[2][ComPort]=='a')   // Enable all motors
            {
                Disable1=0;
                Disable2=0;
                Disable3=0;
#ifdef MODE2    // 43A "Chinese" H-Bridge
                // Reset any overtemp shutdowns and enable the H-Bridges  - toggle pins low then leave high
                OutputPort &= ~(1 << ENBpin1);        // Unset Motor1 In 2
                OutputPort &= ~(1 << ENBpin2);        // Unset Motor2 In 2
                OutputPort &= ~(1 << ENBpin3);        // Unset Motor3 In 2
                PORTD = OutputPort;
                OutputPort |= 1 << ENBpin1;           // Set Motor1 In 2
                OutputPort |= 1 << ENBpin2;           // Set Motor2 In 2
                OutputPort |= 1 << ENBpin3;           // Set Motor3 In 2
                PORTD = OutputPort;
#endif
            }
            else if (RxBuffer[1][ComPort]=='n' && RxBuffer[2][ComPort]=='1')   // Enable motor 1
            {
                Disable1=0;
#ifdef MODE2    // 43A "Chinese" H-Bridge
                // Reset any overtemp shutdowns and enable the H-Bridges  - toggle pins low then leave high
                OutputPort &= ~(1 << ENBpin1);        // Unset Motor1 In 2
                PORTD = OutputPort;
                OutputPort |= 1 << ENBpin1;           // Set Motor1 In 2
                PORTD = OutputPort;
#endif
            }
            else if (RxBuffer[1][ComPort]=='n' && RxBuffer[2][ComPort]=='2')   // Enable motor 2
            {
                Disable2=0;
#ifdef MODE2    // 43A "Chinese" H-Bridge
                // Reset any overtemp shutdowns and enable the H-Bridges  - toggle pins low then leave high
                OutputPort &= ~(1 << ENBpin2);        // Unset Motor2 In 2
                PORTD = OutputPort;
                OutputPort |= 1 << ENBpin2;           // Set Motor2 In 2
                PORTD = OutputPort;
#endif
            }
            else if (RxBuffer[1][ComPort]=='n' && RxBuffer[2][ComPort]=='3')   // Enable motor 3
            {
                Disable3=0;
#ifdef MODE2    // 43A "Chinese" H-Bridge
                // Reset any overtemp shutdowns and enable the H-Bridges  - toggle pins low then leave high
                OutputPort &= ~(1 << ENBpin3);        // Unset Motor3 In 2
                PORTD = OutputPort;
                OutputPort |= 1 << ENBpin3;           // Set Motor3 In 2
                PORTD = OutputPort;
#endif
            }
            break;
        case '?':
            break;
//        default:
    }


}

//****************************************************************************************************************
//    Check the incoming serial data to see if a command has been received
//
//****************************************************************************************************************


void CheckSerial0()
{
    while(Serial.available())
    {
        if(BufferEnd[COM0]==-1)
        {
            RxByte[COM0] = Serial.read();
            if(RxByte[COM0] != START_BYTE){BufferEnd[COM0]=-1;errorcount++;}else{BufferEnd[COM0]=0;}
        }
        else
        {
            RxByte[COM0] = Serial.read();
            RxBuffer[BufferEnd[COM0]][COM0]=RxByte[COM0];
            BufferEnd[COM0]++;
            if(BufferEnd[COM0] > 3)
            {
                if(RxBuffer[3][COM0]==END_BYTE)
                {
                    ParseCommand(COM0);
                }
                else
                {
                    errorcount++;
                }
                BufferEnd[COM0]=-1;
            }
        }
    }
}


void CheckSerial1()
{
#ifdef SECOND_SERIAL
    while(mySerial.available())
    {
        if(BufferEnd[COM1]==-1)
        {
            RxByte[COM1] = mySerial.read();
            if(RxByte[COM1] != START_BYTE){BufferEnd[COM1]=-1;}else{BufferEnd[COM1]=0;}
        }
        else
        {
            RxByte[COM1] = mySerial.read();
            RxBuffer[BufferEnd[COM1]][COM1]=RxByte[COM1];
            BufferEnd[COM1]++;
            if(BufferEnd[COM1] > 3)
            {
                if(RxBuffer[3][COM1]==END_BYTE)
                {
                    ParseCommand(COM1);
                }
                else
                {
                    errorcount++;
                }
                BufferEnd[COM1]=-1;
            }
        }
    }
#endif
}


#ifdef MODE1    // This mode outputs a PWM and two motor Direction pins

//****************************************************************************************************************
//    Set the Motor output pins to drive the motor in required direction and speed
//    (Goal = smoothed target, see SmoothTarget())
//
//****************************************************************************************************************


void SetOutputsMotor1()
{
    if((Feedback1 > InputClipMax1) && (PWMrev1 != 0))
    {
        PWMout1 = PWMrev1;
        OutputPort &= ~(1 << ENApin1);  // Unset Motor1 In 1
        OutputPort |= 1 << ENBpin1;           // Set Motor1 In 2
        MyPWMWrite(PWMpin1, PWMrev1);
    }
    else if((Feedback1<InputClipMin1) && (PWMrev1 != 0))
    {
        PWMout1 = PWMrev1;
        OutputPort |= 1 << ENApin1;    // Set Motor1 In 1
        OutputPort &= ~(1 << ENBpin1);        // Unset Motor1 In 2
        MyPWMWrite(PWMpin1, PWMrev1);
    }
    else if((Goal1 > (Feedback1 + DeadZone1)) || (Goal1 < (Feedback1 - DeadZone1)))
    {
        if (PWMout1 >= 0)
        {
            // Drive Motor Forward
            PWMout1+=PWMoffset1;
            if(PWMout1 > (PWMmax1+LiftFactor1)){PWMout1=PWMmax1+LiftFactor1;}
            OutputPort |= 1 << ENApin1;           // Set Motor1 In 1
            OutputPort &= ~(1 << ENBpin1);        // Unset Motor1 In 2
        }
        else
        {
            // Drive Motor Backwards
            PWMout1 = abs(PWMout1);
            PWMout1+=PWMoffset1;
            if(PWMout1 > PWMmax1){PWMout1=PWMmax1;}
            OutputPort &= ~(1 << ENApin1);        // Unset Motor1 In 1
            OutputPort |= 1 << ENBpin1;           // Set Motor1 In 2
        }
        MyPWMWrite(PWMpin1, PWMout1);
    }
    else
    {
        // Brake Motor
        OutputPort &= ~(1 << ENApin1);        // Unset Motor1 In 1
        OutputPort &= ~(1 << ENBpin1);        // Unset Motor1 In 2
        PWMout1=PWMoffset1;
        MyPWMWrite(PWMpin1, 0);
    }
    PORTD = OutputPort;
}


void SetOutputsMotor2()
{
    if((Feedback2 > InputClipMax2) && (PWMrev2 != 0))
    {
        PWMout2 = PWMrev2;
        OutputPort &= ~(1 << ENApin2);  // Unset Motor1 In 1
        OutputPort |= 1 << ENBpin2;           // Set Motor1 In 2
        MyPWMWrite(PWMpin2, PWMrev2);
    }
    else if((Feedback2<InputClipMin2) && (PWMrev2 != 0))
    {
        PWMout2 = PWMrev2;
        OutputPort |= 1 << ENApin2;    // Set Motor1 In 1
        OutputPort &= ~(1 << ENBpin2);        // Unset Motor1 In 2
        MyPWMWrite(PWMpin2, PWMrev2);
    }
    else if(Goal2 > (Feedback2 + DeadZone2) || Goal2 < (Feedback2 - DeadZone2))
    {
        if (PWMout2 >= 0)
        {
            // Drive Motor Forward
            PWMout2+=PWMoffset2;
            if(PWMout2 > PWMmax2+LiftFactor2){PWMout2=PWMmax2+LiftFactor2;}
            OutputPort |= 1 << ENApin2;           // Set Motor2 In 1
            OutputPort &= ~(1 << ENBpin2);        // Unset Motor2 In 2
        }
        else
        {
            // Drive Motor Backwards
            PWMout2 = abs(PWMout2);
            PWMout2+=PWMoffset2;
            if(PWMout2 > PWMmax2){PWMout2=PWMmax2;}
            OutputPort &= ~(1 << ENApin2);        // Unset Motor2 In 1
            OutputPort |= 1 << ENBpin2;           // Set Motor2 In 2
        }
        MyPWMWrite(PWMpin2, PWMout2);
    }
    else
    {
        // Brake Motor
        OutputPort &= ~(1 << ENApin2);        // Unset Motor2 In 1
        OutputPort &= ~(1 << ENBpin2);        // Unset Motor2 In 2
        PWMout2=PWMoffset2;
        MyPWMWrite(PWMpin2, 0);
    }
    PORTD = OutputPort;
}


void SetOutputsMotor3()
{
    if((Feedback3 > InputClipMax3) && (PWMrev3 != 0))
    {
        PWMout3 = PWMrev3;
        OutputPort &= ~(1 << ENApin3);  // Unset Motor1 In 1
        OutputPort |= 1 << ENBpin3;           // Set Motor1 In 2
        analogWrite(PWMpin3, PWMrev3);
    }
    else if((Feedback3<InputClipMin3) && (PWMrev3 != 0))
    {
        PWMout3 = PWMrev3;
        OutputPort |= 1 << ENApin3;    // Set Motor1 In 1
        OutputPort &= ~(1 << ENBpin3);        // Unset Motor1 In 2
        analogWrite(PWMpin3, PWMrev3);
    }
    else if(Goal3 > (Feedback3 + DeadZone3) || Goal3 < (Feedback3 - DeadZone3))
    {
        if (PWMout3 >= 0)
        {
            // Drive Motor Forward
            PWMout3+=PWMoffset3;
            if(PWMout3> PWMmax3){PWMout3=PWMmax3;}
            OutputPort |= 1 << ENApin3;           // Set Motor3 In 1
            OutputPort &= ~(1 << ENBpin3);        // Unset Motor3 In 2
        }
        else
        {
            // Drive Motor Backwards
            PWMout3 = abs(PWMout3);
            PWMout3+=PWMoffset3;
            if(PWMout3> PWMmax3){PWMout3=PWMmax3;}
            OutputPort &= ~(1 << ENApin3);        // Unset Motor3 In 1
            OutputPort |= 1 << ENBpin3;           // Set Motor3 In 2
        }
        analogWrite(PWMpin3, PWMout3);
    }
    else
    {
        // Brake Motor
        OutputPort &= ~(1 << ENApin3);        // Unset Motor3 In 1
        OutputPort &= ~(1 << ENBpin3);        // Unset Motor3 In 2
        PWMout3=PWMoffset3;
        analogWrite(PWMpin3, 0);
    }
    PORTD = OutputPort;
}

#endif


#ifdef MODE2    // This mode outputs a H-Bridge Enable, One Dir Pin and a +/-PWM

//****************************************************************************************************************
//    Set the Motor output pins to drive the motor in required direction and speed
//    (Goal = smoothed target, see SmoothTarget())
//
//****************************************************************************************************************


void SetOutputsMotor1()   // MODE2
{
    if((Feedback1 > InputClipMax1) && (PWMrev1 != 0))
    {
       PWMout1 = PWMrev1;
       OutputPort &= ~(1 << ENApin1);  // Unset Motor1 In 1
       MyPWMWrite(PWMpin1, PWMrev1);
    }
    else if((Feedback1<InputClipMin1) && (PWMrev1 != 0))
    {
       PWMout1 = PWMrev1;
       OutputPort |= 1 << ENApin1;    // Set Motor1 In 1
       MyPWMWrite(PWMpin1, 255-PWMrev1);
    }
    else if((Goal1 > (Feedback1 + DeadZone1)) || (Goal1 < (Feedback1 - DeadZone1)))
    {
        if (PWMout1 >= 0)
        {
            // Drive Motor Forward
            PWMout1+=PWMoffset1;
            if(PWMout1 > (PWMmax1+LiftFactor1)){PWMout1=PWMmax1+LiftFactor1;}
            OutputPort |= 1 << ENApin1;           // Set Motor1 In 1
            MyPWMWrite(PWMpin1, 255-PWMout1);    // Motor driven when PWM=0 (unset)
        }
        else
        {
            // Drive Motor Backwards
            PWMout1 = abs(PWMout1);
            PWMout1+=PWMoffset1;
            if(PWMout1 > PWMmax1){PWMout1=PWMmax1;}
            OutputPort &= ~(1 << ENApin1);        // Unset Motor1 In 1
            MyPWMWrite(PWMpin1, PWMout1);        // Motor driven when PWM=1 (set)
        }
    }
    else
    {
        // Brake Motor
        OutputPort &= ~(1 << ENApin1);        // Unset Motor1 In 1
        PWMout1=PWMoffset1;
        MyPWMWrite(PWMpin1, 0);
    }
    PORTD = OutputPort;
}


void SetOutputsMotor2()   // MODE2
{
    if ((Feedback2>InputClipMax2) && (PWMrev2 != 0))
    {
       PWMout2 = PWMrev2;
       OutputPort &= ~(1 << ENApin2);  // Unset Motor2 In 1
       MyPWMWrite(PWMpin2, PWMrev2);
    }
    else if ((Feedback2<InputClipMin2) && (PWMrev2 != 0))
    {
       PWMout2 = PWMrev2;
       OutputPort |= 1 << ENApin2;    // Set Motor2 In 1
       MyPWMWrite(PWMpin2, 255-PWMrev2);
    }
    else if((Goal2 > (Feedback2 + DeadZone2)) || (Goal2 < (Feedback2 - DeadZone2)))
    {
        if (PWMout2 >= 0)
        {
            // Drive Motor Forward
            PWMout2+=PWMoffset2;
            if(PWMout2 > PWMmax2+LiftFactor2){PWMout2=PWMmax2+LiftFactor2;}
            OutputPort |= 1 << ENApin2;           // Set Motor2 In 1
            MyPWMWrite(PWMpin2, 255-PWMout2);
        }
        else
        {
            // Drive Motor Backwards
            PWMout2 = abs(PWMout2);
            PWMout2+=PWMoffset2;
            if(PWMout2 > PWMmax2){PWMout2=PWMmax2;}
            OutputPort &= ~(1 << ENApin2);        // Unset Motor2 In 1
            MyPWMWrite(PWMpin2, PWMout2);
        }
    }
    else
    {
        // Brake Motor
        OutputPort &= ~(1 << ENApin2);        // Unset Motor2 In 1
        PWMout2=PWMoffset2;
        MyPWMWrite(PWMpin2, 0);
    }
    PORTD = OutputPort;
}


void SetOutputsMotor3()   // MODE2
{
    if ((Feedback3>InputClipMax3) && (PWMrev3 != 0))
    {
       PWMout3 = PWMrev3;
       OutputPort &= ~(1 << ENApin3);  // Unset Motor3 In 1
       analogWrite(PWMpin3, PWMrev3);
    }
    else if ((Feedback3<InputClipMin3) && (PWMrev3 != 0))
    {
       PWMout3 = PWMrev3;
       OutputPort |= 1 << ENApin3;    // Set Motor3 In 1
       analogWrite(PWMpin3, 255-PWMrev3);
    }
    else if((Goal3 > (Feedback3 + DeadZone3)) || (Goal3 < (Feedback3 - DeadZone3)))
    {
        if (PWMout3 >= 0)
        {
            // Drive Motor Forward
            PWMout3+=PWMoffset3;
            if(PWMout3> PWMmax3){PWMout3=PWMmax3;}
            OutputPort |= 1 << ENApin3;           // Set Motor3 In 1
            analogWrite(PWMpin3, 255-PWMout3);
        }
        else
        {
            // Drive Motor Backwards
            PWMout3 = abs(PWMout3);
            PWMout3+=PWMoffset3;
            if(PWMout3> PWMmax3){PWMout3=PWMmax3;}
            OutputPort &= ~(1 << ENApin3);        // Unset Motor3 In 1
            analogWrite(PWMpin3, PWMout3);
        }
    }
    else
    {
        // Brake Motor
        OutputPort &= ~(1 << ENApin3);        // Unset Motor3 In 1
        PWMout3=PWMoffset3;
        analogWrite(PWMpin3, 0);
    }
    PORTD = OutputPort;
}

#endif




//****************************************************************************************************************
//    Target smoothing
//
//    Every PID cycle the smoothed goal moves a fraction (1/2^TARGET_SMOOTH_SHIFT) of the way towards the
//    raw target, optionally limited to SlewStepQ8 per cycle.  Done in 24.8 fixed point so it converges
//    all the way to the target instead of stalling a few counts short.  Costs a handful of microseconds.
//
//****************************************************************************************************************

int SmoothTarget(uint8_t m, int target)
{
    long diff = ((long)target << 8) - GoalAcc[m];
    long step;

#if TARGET_SMOOTH_SHIFT > 0
    step = diff >> TARGET_SMOOTH_SHIFT;
    if (step == 0) { step = diff; }     // last fraction of a count: just land on the target
#else
    step = diff;
#endif

    if (SlewStepQ8 > 0)
    {
        if (step > SlewStepQ8) { step = SlewStepQ8; }
        else if (step < -SlewStepQ8) { step = -SlewStepQ8; }
    }

    GoalAcc[m] += step;
    return (int)((GoalAcc[m] + 128) >> 8);
}


//****************************************************************************************************************
//    Calculate the motor PID output - one routine shared by all three motors.
//    Based on http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//    Integer only for speed.
//
//      P:  Kp_x100 * Error                          (same scaling as the original SMC3)
//      I:  Ki_x100 * CumError >> I_SHIFT            (CumError is a real running sum, clamped so the I term
//                                                    alone cannot exceed full PWM, reset inside the dead zone)
//      D:  Kd_x100 * dPosition << D_SHIFT           (derivative of the FEEDBACK, sampled every Ks cycles and
//                                                    low-pass filtered - no kick when the target jumps)
//
//    The sum is shifted right by PowerScale (7 normally, 9 in comms-timeout low power) - the /128 is an
//    approximation of /100 to bring the x100 terms back to PWM units, as in the original.
//
//****************************************************************************************************************

int CalcMotorPID(uint8_t m, int TargetPosition, int CurrentPosition,
                 long Kp_x100, long Ki_x100, long Kd_x100, int Ks, int DeadZone)
{
    long pTerm_x100;
    long iTerm_x100;
    int Error = TargetPosition - CurrentPosition;

    // ---- Proportional ----
    pTerm_x100 = Kp_x100 * (long)Error;          // Error is +/-1023 max and Kp is 0..65535 so this fits in a long

    // ---- Integral ----
    if (abs(Error) <= DeadZone || PidIntegralLimit[m] == 0)
    {
        PidCumError[m] = 0;                      // inside the dead zone (or Ki = 0): no integral action
    }
    else
    {
        PidCumError[m] += Error;
        if (PidCumError[m] > PidIntegralLimit[m]) { PidCumError[m] = PidIntegralLimit[m]; }
        else if (PidCumError[m] < -PidIntegralLimit[m]) { PidCumError[m] = -PidIntegralLimit[m]; }
    }
    iTerm_x100 = (Ki_x100 * PidCumError[m]) >> I_SHIFT;

    // ---- Derivative (on measurement) ----
    PidKdFilterCount[m]++;
    if (PidKdFilterCount[m] >= Ks)               // sample the velocity every Ks PID cycles (Ks from SMC3Utils)
    {
        int DeltaPosition = CurrentPosition - PidLastPosition[m];
        PidLastPosition[m] = CurrentPosition;
        PidKdFilterCount[m] = 0;
        long dRaw_x100 = (Kd_x100 * (long)DeltaPosition) << D_SHIFT;
#if D_FILTER_SHIFT > 0
        PidDTerm_x100[m] += (dRaw_x100 - PidDTerm_x100[m]) >> D_FILTER_SHIFT;
#else
        PidDTerm_x100[m] = dRaw_x100;
#endif
    }

    return constrain((pTerm_x100 + iTerm_x100 - PidDTerm_x100[m]) >> PowerScale, -255, 255);
}


//****************************************************************************************************************
//    Disable the Motors - ie put the brakes on
//    Used when motor feedback is outside allowed range to minimise damage
//
//****************************************************************************************************************

void DisableMotor1()
{
    Disable1 = 1;

    OutputPort &= ~(1 << ENApin1);
    OutputPort &= ~(1 << ENBpin1);

    PORTD = OutputPort;

    MyPWMWrite(PWMpin1, 0);

}

void DisableMotor2()
{
    Disable2 = 1;

    OutputPort &= ~(1 << ENApin2);
    OutputPort &= ~(1 << ENBpin2);

    PORTD = OutputPort;

    MyPWMWrite(PWMpin2, 0);

}

void DisableMotor3()
{
    Disable3 = 1;

    OutputPort &= ~(1 << ENApin3);
    OutputPort &= ~(1 << ENBpin3);

    PORTD = OutputPort;

    analogWrite(PWMpin3, 0);

}


#ifdef DEBUG_TIMING_PIN
void TogglePin()
{
   static int PinOut=0;
   PinOut=1-PinOut;
   digitalWrite(8, PinOut);
}
#endif


//****************************************************************************************************************
//    Main Arduino Program Loop
//
//****************************************************************************************************************

void loop()
{


    //Read all stored Parameters from EEPROM

    ReadEEProm();
    UpdatePIDLimits();
    UpdateSmoothingRate();

    // Start the smoothed goals from wherever the motors actually are, so the move to centre at power-up
    // is a ramp rather than a jump (how gentle depends on TARGET_SMOOTH_SHIFT / TARGET_MAX_RATE).

    Feedback1 = analogRead(FeedbackPin1);
    Feedback2 = analogRead(FeedbackPin2);
    Feedback3 = analogRead(FeedbackPin3);
    GoalAcc[0] = (long)constrain(Feedback1, InputClipMin1, InputClipMax1) << 8;
    GoalAcc[1] = (long)constrain(Feedback2, InputClipMin2, InputClipMax2) << 8;
    GoalAcc[2] = (long)constrain(Feedback3, InputClipMin3, InputClipMax3) << 8;
    Goal1 = (int)(GoalAcc[0] >> 8);
    Goal2 = (int)(GoalAcc[1] >> 8);
    Goal3 = (int)(GoalAcc[2] >> 8);
    PidLastPosition[0] = Feedback1;
    PidLastPosition[1] = Feedback2;
    PidLastPosition[2] = Feedback3;

    // Enable all motors

    Disable1=0;
    Disable2=0;
    Disable3=0;

#ifdef MODE2    // 43A "Chinese" H-Bridge
    // Reset any overtemp shutdowns and enable the H-Bridges  - toggle pins low then leave high
    OutputPort &= ~(1 << ENBpin1);        // Unset Motor1 In 2
    OutputPort &= ~(1 << ENBpin2);        // Unset Motor2 In 2
    OutputPort &= ~(1 << ENBpin3);        // Unset Motor3 In 2
    PORTD = OutputPort;
    OutputPort |= 1 << ENBpin1;           // Set Motor1 In 2
    OutputPort |= 1 << ENBpin2;           // Set Motor2 In 2
    OutputPort |= 1 << ENBpin3;           // Set Motor3 In 2
    PORTD = OutputPort;
#endif


    // Initialise the PID ready timer

    TimesUp = micros();
    PIDProcessCounter=0;

    // Main Program loop

    while (1==1)
    {

        // Wait until its time and then update PID calcs for first motor

        while ((micros() - TimesUp) < PROCESS_PERIOD_uS) { ; }
        TimesUp += PROCESS_PERIOD_uS;
#ifdef DEBUG_TIMING_PIN
        TogglePin();                      // Used for testing to monitor PID timing on Oscilloscope
#endif

        PIDProcessCounter++;

        if (PIDProcessCounter >= PIDProcessDivider)
        {
            PIDProcessCounter=0;

            // Check and Update Motor 1 drive

            Goal1 = SmoothTarget(0, Target1);
            Feedback1 = analogRead(FeedbackPin1);
            if ((Feedback1 > CutoffLimitMax1) || (Feedback1 < CutoffLimitMin1)) { DisableMotor1(); }
            PWMout1=CalcMotorPID(0, Goal1, Feedback1, Kp1_x100, Ki1_x100, Kd1_x100, Ks1, DeadZone1);
            if (Disable1==0)
            {
                SetOutputsMotor1();
            }
            else
            {
                PWMout1=0;
            }

            // Check and Update Motor 2 drive

            Goal2 = SmoothTarget(1, Target2);
            Feedback2 = analogRead(FeedbackPin2);
            if ((Feedback2 > CutoffLimitMax2) || (Feedback2 < CutoffLimitMin2)) { DisableMotor2(); }     // was CutoffLimitMin1
            PWMout2=CalcMotorPID(1, Goal2, Feedback2, Kp2_x100, Ki2_x100, Kd2_x100, Ks2, DeadZone2);
            if (Disable2==0)
            {
                SetOutputsMotor2();
            }
            else
            {
                PWMout2=0;
            }

            // Check and Update Motor 3 drive

            Goal3 = SmoothTarget(2, Target3);
            Feedback3 = analogRead(FeedbackPin3);
            if ((Feedback3 > CutoffLimitMax3) || (Feedback3 < CutoffLimitMin3)) { DisableMotor3(); }     // was CutoffLimitMin1
            PWMout3=CalcMotorPID(2, Goal3, Feedback3, Kp3_x100, Ki3_x100, Kd3_x100, Ks3, DeadZone3);
            if (Disable3==0)
            {
                SetOutputsMotor3();
            }
            else
            {
                PWMout3=0;
            }

            LoopCount++;
        }

        // Check if there is any received serial data and process (Hardware UART)

        CheckSerial0();

        // Check if there is any received serial data and process (Software Serial)

        CheckSerial1();

        SerialFeedbackCounter++;
        if (SerialFeedbackCounter >= 80)  // every 20ms send back position, pwm and motor status updates if enabled
        {
            SerialFeedbackPort = (SerialFeedbackEnabled >> 4) & 0x01;
            if  ((SerialFeedbackEnabled & 0x03) == 1)     // Monitor Motor 1
            {
#ifdef REVERSE_MOTOR1
               SendTwoValues('A',(1023-Feedback1)/4,(1023-Goal1)/4,SerialFeedbackPort);
#else
               SendTwoValues('A',Feedback1/4,Goal1/4,SerialFeedbackPort);
#endif
               SendTwoValues('a',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout1,0,255),SerialFeedbackPort);
            }
            else if  ((SerialFeedbackEnabled & 0x03) == 2)     // Monitor Motor 2
            {
               SendTwoValues('B',Feedback2/4,Goal2/4,SerialFeedbackPort);
               SendTwoValues('b',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout2,0,255),SerialFeedbackPort);
            }
            else if  ((SerialFeedbackEnabled & 0x03) == 3)     // Monitor Motor 3
            {
               SendTwoValues('C',Feedback3/4,Goal3/4,SerialFeedbackPort);
               SendTwoValues('c',PIDProcessDivider*16 + Disable1+(Disable2*2)+(Disable3*4),constrain(PWMout3,0,255),SerialFeedbackPort);
            }
            SerialFeedbackCounter = 0;

#ifdef ENABLE_POT_SCALING
            PotInput = PotInput * 0.9;
            PotInput = PotInput + (0.1 * analogRead(PotInputPin));  // Also read the User Pot Input every 20ms
#endif

#ifdef ENABLE_NON_LINEAR_POT_SCALING
            PotInput = PotInput * 0.9;
            PotInput = PotInput + (0.1 * analogRead(PotInputPin));  // Also read the User Pot Input every 20ms
#endif

        }

        CommsTimeout++;
        if (CommsTimeout >= 60000)       // 15 second timeout ie 60000 * PROCESS_PERIOD_uS
        {
            CommsTimeout = 60000;        // So the counter doesn't overflow
            PowerScale = 9;
        }
        else
        {
            PowerScale = 7;              // Number of bits to shift PID result (ie divide by 128 - approximate for /100)
        }


    }
}

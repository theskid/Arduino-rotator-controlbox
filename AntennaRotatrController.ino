// Ham Antenna Rotator Device
// Version 1.00 (Rev. 3)
// 
// A SkyDubh production (http://www.skydubh.com/)
// 
// Crew
// Diego Cioschi: PoC and first code writing
// Bruno Passeri: code optimization, refactoring, graphical overhaul
//
// Dependencies:
// UTFT Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)
// UTFT Geometry Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)

#include "Settings.h"                                                   // User settings take precedence (flags and debug mode)
#include "Global.h"                                                     // Main header
#include "Displays.h"                                                   // Displays and initializations
#include "SerialPrint.h"                                                // Serial (and debug) print(f) helpers
#include <microsmooth.h>

/*** PIN DESIGNATION AND COMPONENTS SETTINGS *************/

const int StartStopSwitch = BUTTON_START_STOP;
const int UserActionSwitch = BUTTON_CHANGE_PLANNED_DIRECTION;
const int SpeedControlSwitch = BUTTON_AUTO_MANUAL;
#ifndef PROTEUS_VSM
const int PWMSpeedControl = PWM_SPEED_CONTROL;
const int CWMotor = MOTOR_CLOCKWISE_SIGNAL;
const int CCWMotor = MOTOR_COUNTERCLOCKWISE_SIGNAL;
#endif

const uint8_t spdSetPotentiometer = POTENTIOMETER_MANUAL_SPEED;
const uint8_t beamSetPotentiometer = POTENTIOMETER_PLANNED_DIRECTION;
const uint8_t rotatorSensor = POTENTIOMETER_ROTOR_SENSOR;

/*** GLOBAL DECLARATIONS *********************************/

#define FASTADC                                                         // Fast ADC for 12bit readings
#define POTENTIOMETER_MAX 1023                                          // Potentiometer range [0..x]

#define MULTIPLE_SAMPLING 4                                             // How many N/16 samples get acquired per 12bit read cycle

int beamDir = 0;                                                        // Actual beam direction
int beamDirStart = 0;                                                   // The initial bearing before the rotation begins
int beamSet = 0;                                                        // Beam directione to set
int beamSetRoute = 0;                                                   // Beam setting bearing the shortest route
int spdValue = 0;                                                       // Rotation speed

boolean bMoveAntenna = false;                                           // Start Stop flag
boolean bSpeedModeAuto = true;                                          // Speed Mode Flag
boolean bChoosingNewAngle = false;                                      // User Action flag
boolean bPreSetup = true;                                               // Signal we're not out of the setup yet

#ifndef DISABLE_SKIPPING
    boolean bUpdateScreen = true;                                       // Draw/Skip signal
#else
    #define bUpdateScreen true
#endif
#if ((!defined(MULTIPLE_SAMPLING)) || (0 == MULTIPLE_SAMPLING))
#define DISABLE_MULTISTEP_SAMPLING
#endif

/*** BUTTON MAPPING AND EVENT TRIGGERS *******************/

void StartStopToggle();                                                 // Start/Stop button event toggle
void AutoManualToggle();                                                // Auto/Manual button event toggle
void UserSetConfirmToggle();                                            // Set/Confirm button event toggle

const BUTTON_MAP ButtonsMap[] = {
    { UserActionSwitch, UserSetConfirmToggle },
    { StartStopSwitch, StartStopToggle },
    { SpeedControlSwitch, AutoManualToggle },
};
#define BUTTON_COUNT (sizeof ButtonsMap / sizeof ButtonsMap[0])

/*** EXTERNALLY DEFINED **********************************/

extern COMPASS compass;                                                 // Compass basic coordinates
extern UTFT display;                                                    // Main display
extern UTFT_Geometry geo;                                               // Geometric helper functions

/*******************************************************************
**** THERE BE DRAGONS **********************************************
*******************************************************************/

// Sensors reading helpers
#ifdef DEBUG
    int rawBeamDir;
    #define ReadBeamDir() map(rawBeamDir = AnalogRead12Bits(rotatorSensor), ROTATION_BEGIN, ROTATION_END, 0, 360)
#else
    #define ReadBeamDir() map(AnalogRead12Bits(rotatorSensor), ROTATION_BEGIN, ROTATION_END, 0, 360)
#endif
#define ReadBeamSet() map(analogRead(beamSetPotentiometer), 0, POTENTIOMETER_MAX, 0, 359)

// Print the angle
void UpdateAngles(int angle, const COLORS& color, const BHTYPE& type) {
    static COLORS lastla[2] = { (COLORS)0 };                            // Prev. colors
    static COLORS lastra[2] = { (COLORS)0 };
    static COLORS lastac[2] = { (COLORS)0 };
    static int lastAngle[2] = { 0x7fff };
    static boolean bWasLastOver = false;
    COLORS la = COLORS::Black;                                          // Left arrow
    COLORS ra = COLORS::Black;                                          // Right arrow
    boolean bIsOver = false;
    HUD_BEAM hb = { type, nullptr, (COLORS)0 };

    // Overlaps
    if (0 > angle) {
        angle += 360;
        la = COLORS::Red;
        bIsOver = true;
    }
    if (359 < angle) {
        angle -= 360;
        ra = COLORS::Red;
        bIsOver = true;
    }
    if (BHTYPE::BeamDIR == type) {
        if (bIsOver != bWasLastOver) {
            bWasLastOver = bIsOver;
            DrawHudElement(bIsOver ? "OVER" : "    ", HUD::OverlapAlert);
        }
    }

    if (lastla[type] != la) {
        lastla[type] = la;
        hb.color = la;
        DrawHudElement(&hb, HUD::BeamLeftArrow);
    }
    if (lastra[type] != ra) {
        lastra[type] = ra;
        hb.color = ra;
        DrawHudElement(&hb, HUD::BeamRightArrow);
    }
    if ((lastAngle[type] != angle) || (lastac[type] != color)) {
        lastAngle[type] = angle;
        lastac[type] = color;
        char buff[4];                                                   // 3 digits + '\0'
        sprintf(buff, "%03d", angle);
        hb.angle = buff;
        hb.color = color;
        DrawHudElement(&hb, HUD::BeamAngle);
    }
}

// Update the azimuthal beam direction
inline void BeamDirection() {
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Entering BeamDirection()\r\n");
    #endif
    beamDir = ReadBeamDir();
    #ifdef DEBUG
        static int lbd = 0x7FFF;
        if (lbd != beamDir) {
            lbd = beamDir;
            DebugPrintf("Antenna bearing: %d\r\n", beamDir);
        }
    #endif
    UpdateAngles(beamDir, beamSetRoute != beamDir ? COLORS::Yellow : COLORS::Green, BeamDIR);
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Exiting BeamDirection()\r\n");
    #endif
}

// Update the azimuthal beam setting direction
inline void BeamSetting() {
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Entering BeamSetting()\r\n");
    #endif
    COLORS color = COLORS::Green;
    if ((!bPreSetup) && bChoosingNewAngle)
        color = COLORS::Yellow;
    if (bPreSetup || bChoosingNewAngle)
        beamSet = ReadBeamSet();

    // Find the shortest path from beamDir
    int angle = beamSet;
    if (OVERLAP_TOLERANCE >= (360 - angle))
        angle = abs(beamDir - (angle - 360)) < abs(beamDir - angle) ? angle - 360 : angle;
    if (OVERLAP_TOLERANCE >= (angle + 1))
        angle = abs(beamDir - (angle + 360)) < abs(beamDir - angle) ? angle + 360 : angle;
    beamSetRoute = angle;

    #ifdef DEBUG_VERBOSE
        static int lbs = 0x7FFF, lbsr = 0x7FFF;
        if ((lbs != beamSet) || (lbsr != beamSetRoute)) {
            lbs = beamSet;
            lbsr = beamSetRoute;
            DebugPrintf("Planned direction: %d\r\nShortest calculated route: %d\r\n", beamSet, beamSetRoute);
        }
    #endif

    UpdateAngles(beamSetRoute, color, BeamSET);
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Exiting BeamSetting()\r\n");
    #endif
}

// Toggles the Set/Confirm state [CB]
void UserSetConfirmToggle() {
    if (bMoveAntenna)                                                   // Forbid a change of destination during rotor movement
        return;
    bChoosingNewAngle = !bChoosingNewAngle;
    #ifdef DEBUG_VERBOSE
        DebugPrintf("UserActionSwitch has been pushed, now: %s\r\n", bChoosingNewAngle ? "Setting" : "Confirmed");
    #endif
}

// Toggles the Start/Stop state [CB]
void StartStopToggle() {
    if (bChoosingNewAngle && !bMoveAntenna)                             // Auto-confirm upon starting the engine
        UserSetConfirmToggle();
    bMoveAntenna = !bMoveAntenna;
    if (bMoveAntenna)
        beamDirStart = beamDir;
    #ifdef DEBUG_VERBOSE
        DebugPrintf("StartStopSwitch has been pushed, now: %s\r\n", bMoveAntenna ? "Start" : "Stop");
    #endif
}

// Toggles the Auto/Manual state [CB]
void AutoManualToggle() {
    if (!bPreSetup) {
        bSpeedModeAuto = !bSpeedModeAuto;
        #ifdef DEBUG_VERBOSE
            DebugPrintf("SpeedControlSwitch has been pushed, now: %s\r\n", bSpeedModeAuto ? "Automatic" : "Manual");
        #endif
    }
    DrawHudElement(bSpeedModeAuto ? "  Auto" : "Manual", HUD::AutoManual);
 }

 // Starts and stops the rotor upon need
inline void SpinRotor() {
    static const char* msg = nullptr;
    static const char* last = nullptr;
    
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Entering SpinRotor()\r\n");
    #endif

    uint8_t cw = LOW;
    uint8_t ccw = LOW;

    if ((!bMoveAntenna) || (beamDir == beamSetRoute)) {
        bMoveAntenna = false;
        msg = ("   ");
    } else if (bMoveAntenna && (beamDir < beamSetRoute)) {
        cw = HIGH;
        msg = (" CW ");
    } else if (bMoveAntenna && (beamDir > beamSetRoute)) {
        ccw = HIGH;
        msg = ("CCW");
    }
    #ifdef DEBUG_VERBOSE
        static uint8_t lcw = LOW, lccw = LOW;
        if ((lcw != cw) || (lccw != ccw)) {
            if ((LOW == lcw) && (HIGH == cw))
                DebugPrint("Rotating clockwise\r\n");
            else if ((LOW == lccw) && (HIGH == ccw))
                DebugPrint("Rotating counterclockwise\r\n");
            lcw = cw;
            lccw = ccw;
            if ((LOW == cw) && (LOW == ccw))
                DebugPrintf("Stopping the rotor\r\n"
                    "- Current antenna bearing: %d\r\n"
                    "- Current planned direction: %d\r\n"
                    "- Current shortest route: %d\r\n",
                    beamDir, beamSet, beamSetRoute
                );
        }
    #endif
    #ifndef PROTEUS_VSM
        digitalWrite(CWMotor, cw);
        digitalWrite(CCWMotor, ccw);
    #endif
    if (last != msg) {
        last = msg;
        DrawHudElement(msg, HUD::RotationDirection);
    }
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Exiting SpinRotor()\r\n");
    #endif
}

// Defines the speed to apply to the rotor
inline void SetRotorSpeed() {
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Entering SetRotorSpeed()\r\n");
    #endif
    static int lastSpeedValue = -1;
    if (!bSpeedModeAuto) {
        spdValue = map(analogRead(spdSetPotentiometer), 0, POTENTIOMETER_MAX, 0, 255);
        #ifndef PROTEUS_VSM
            analogWrite(PWMSpeedControl, spdValue);
        #endif
    } else {
        if (bMoveAntenna) {
            int rotationValue = abs(beamSet - beamDir),
                rotationDone = abs(beamDirStart - beamDir);
            if (10 >= rotationValue)
                spdValue = map(rotationValue, 0, 10, 20, 220);
            else if (10 >= rotationDone)
                spdValue = map(rotationDone, 0, 10, 20, 220);
            else
                spdValue = 255;
        } else {
            spdValue = 0;
        }
        #ifndef PROTEUS_VSM
            analogWrite(PWMSpeedControl, spdValue);
        #endif
    }
    #ifdef DEBUG_VERBOSE
        static int s = 0x7FFF;
        if (s != spdValue) {
            s = spdValue;
            DebugPrintf("Current rotor motor speed: %d\r\n", spdValue);
        }
    #endif
    int speed = map(spdValue,0,255,0,100);
    if (lastSpeedValue != speed) {
        lastSpeedValue = speed;
        DrawHudElement((void*)speed, HUD::CurrentSpeed);
        DrawHudElement((void*)spdValue, HUD::SpeedMeter);
    }
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("Exiting SetRotorSpeed()\r\n");
    #endif
}

// Checks buttons status and fires corresponding events
void CheckButtons() {
    // Keep track of buttons status, to avoid trigger ghosting/multi-fire
    static BUTTON_STATE buttonState[BUTTON_COUNT] = { BUTTON_STATE::Released };
    for (unsigned int c = 0; c < BUTTON_COUNT; c++) {
        uint8_t state = digitalRead(ButtonsMap[c].DigitalPin);
        // Trigger button event on release
        if ((HIGH == state) && (BUTTON_STATE::Released < buttonState[c])) {
            buttonState[c] = BUTTON_STATE::Released;
            (*ButtonsMap[c].EventFunction)();
        } else if ((LOW == state) && (BUTTON_STATE::Released == buttonState[c])) {
            buttonState[c] = BUTTON_STATE::Pressed;
        }
    }
}

// Initialize the pin configuration
void ConfigureIOPins() {
    pinMode(StartStopSwitch, INPUT);
    pinMode(UserActionSwitch, INPUT);
    pinMode(SpeedControlSwitch, INPUT);
    #ifndef PROTEUS_VSM
        pinMode(PWMSpeedControl, OUTPUT);
        pinMode(CWMotor, OUTPUT);
        pinMode(CCWMotor, OUTPUT);
        digitalWrite(CWMotor, LOW);
        digitalWrite(CCWMotor, LOW);
    #endif
    analogReference(DEFAULT);
}

// Multisampling for 12bit readings
inline int AnalogRead12Bits(uint8_t pin) {
    static uint16_t* history = ms_init(SGA);
/*    static int buffer[16] = { 0 };
    #ifndef DISABLE_MULTISTEP_SAMPLING
        for (int i = 0; i < 16; i++)
            buffer[i] = analogRead(pin);
    #else
        static int track = 16 - MULTIPLE_SAMPLING;
        static boolean bInitialized = false;
        if (!bInitialized) {
            bInitialized = true;
            for (int i = 0; i < (16 - MULTIPLE_SAMPLING); i++)
                buffer[i] = analogRead(pin);
        }
        for (int i = 0; i < MULTIPLE_SAMPLING; i++)
            buffer[track++] = analogRead(pin);
        if (15 < track)
            track -= 16;
    #endif

    int Result;*/
    for (int i = 1; i < 16; i++) {                                      // Read 16 times
        sga_filter(analogRead(pin), history);                         // Sum results
        //DebugPrintf("%d %d %d %d %d %d %d %d %d\r\n", history[0], history[1], history[2], history[3], history[4], history[5], history[6], history[7], history[8]);
    }
    return (history[4] << 2);                                           // Divide by 4 for 12 bit value;
}

// Arduino board bootstrap setup
void setup() {
    Serial.begin(9600);                                                 // 9600 baud rate
    #ifdef DEBUG_WAITONSERIAL
        while(!Serial);                                                 // For portability
    #endif

    #ifndef FASTADC
        // defines for setting and clearing register bits
        #define fastadc_cbit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
        #define fastadc_sbit(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

        // set prescale to 16 (1MHz)
        fastadc_sbit(ADCSRA, ADPS2);
        fastadc_cbit(ADCSRA, ADPS1);
        fastadc_cbit(ADCSRA, ADPS0);
    #endif

    InitializeDisplay();
    ConfigureIOPins();
    AutoManualToggle();
    BeamSetting();
    BeamDirection();
    bPreSetup = false;
}

// Main loop
void loop() {
    static boolean bUpdateAvailable = false;
    static int lastDir = NOREDRAW,
               lastSet = NOREDRAW;
    static const int* angles[4] = { &lastSet, &lastDir, &beamSet, &beamDir };
    #ifndef DISABLE_SKIPPING
        static int counter = -1;
        counter += 1;
        if (FRAME_SKIPS < counter)
            counter -= (1 + FRAME_SKIPS);
        if (FRAME_SKIPS == counter)
            bUpdateScreen = true;
    #endif

    BeamSetting();
    BeamDirection();
    CheckButtons();
    SetRotorSpeed();
    SpinRotor();
    #ifdef DEBUG_ULTRAVERBOSE
        DebugPrint("---------------------------- Cycling loop() START ----------------------------\r\n");
        DebugPrintf("RAW value of rotator potentiometer == %d\r\n", rawBeamDir);
        DebugPrintf("Value of the start/stop flag == %s\r\n", bMoveAntenna ? "Start" : "Stop");
        DebugPrintf("Value of the Auto/Manual flag == %s\r\n", bSpeedModeAuto ? "Automatic" : "Manual");
        DebugPrintf("Value of the User Action flag == %s\r\n", bChoosingNewAngle ? "Setting" : "Confirmed");
        DebugPrintf("Value of the BEAM direction == %d\r\n", beamDir);
        DebugPrintf("Value of the BEAM setting == %d\r\n", beamSet);
        DebugPrintf("Value of the throttle setting == %d\r\n", spdValue);
    #endif
    if ((lastDir != beamDir) || (lastSet != beamSet))
        bUpdateAvailable = true;
    if (bUpdateAvailable && bUpdateScreen) {
        bUpdateAvailable = false;
        bUpdateScreen = false;
        DrawBeamArrows(angles);
        lastDir = beamDir;
        lastSet = beamSet;
    }
    #ifdef DEBUG
        DrawHudElement(&rawBeamDir, HUD::RawRotorPotentiometer);
        #ifdef DEBUG_ULTRAVERBOSE
            DebugPrint("----------------------------- Cycling loop() END -----------------------------\r\n");
        #endif
        #ifdef DEBUG_SLEEP
            delay(1000);
        #endif
    #endif
}

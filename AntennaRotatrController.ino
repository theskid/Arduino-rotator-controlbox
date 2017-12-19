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

int beamDir = 0;                                                        // Actual beam direction
int beamSet = 0;                                                        // Beam directione to set
int spdValue = 0;                                                       // Rotation speed

boolean bMoveAntenna = false;                                           // Start Stop flag
boolean bSpeedModeAuto = true;                                          // Speed Mode Flag
boolean bChoosingNewAngle = false;                                      // User Action flag
boolean bPreSetup = true;                                               // Signal we're not out of the setup yet

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
void UserPrintAngle(int angle, const COLORS& color, const BHTYPE& type) {
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
    if (0 > angle)
    {
        angle += 360;
        la = COLORS::Red;
        bIsOver = true;
    }
    if (359 < angle)
    {
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

    if (lastla[type] != la)
    {
        lastla[type] = la;
        hb.color = la;
        DrawHudElement(&hb, HUD::BeamLeftArrow);
    }
    if (lastra[type] != ra)
    {
        lastra[type] = ra;
        hb.color = ra;
        DrawHudElement(&hb, HUD::BeamRightArrow);
    }

    if ((lastAngle[type] != angle) || (lastac[type] != color))
    {
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
    UserPrintAngle(beamDir, beamSet!=beamDir?Yellow:Green, BeamDIR);    // We defer the anti-flicker to UPA
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
    {
        // Update the angle and define the shortest route to it
        int r = ReadBeamSet();
        if (OVERLAP_TOLERANCE >= (360 - r))
            r = abs(beamDir - (r - 360)) < abs(beamDir - r) ? r - 360 : r;
        if (OVERLAP_TOLERANCE >= (r + 1))
            r = abs(beamDir - (r + 360)) < abs(beamDir - r) ? r + 360 : r;
        beamSet = r;
    }
    UserPrintAngle(beamSet, color, BeamSET);                            // We defer the anti-flicker to UPA
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

    if ((!bMoveAntenna) || (beamDir == beamSet)) {
        bMoveAntenna = false;
        msg = ("   ");
    } else if (bMoveAntenna && (beamDir < beamSet)) {
        cw = HIGH;
        msg = (" CW ");
    } else if (bMoveAntenna && (beamDir > beamSet)) {
        ccw = HIGH;
        msg = ("CCW");
    }
    #ifndef PROTEUS_VSM
    digitalWrite(CWMotor, cw);
    digitalWrite(CCWMotor, ccw);
    #endif
    if (last != msg)
    {
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
            int rotationValue = abs(beamSet - beamDir);
            if (10 < rotationValue)
                spdValue = 255;
            else
                spdValue = map(rotationValue, 0, 10, 20, 220);
        } else {
            spdValue = 0;
        }
        #ifndef PROTEUS_VSM
            analogWrite(PWMSpeedControl, spdValue);
        #endif
    }
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
        }
        else if ((LOW == state) && (BUTTON_STATE::Released == buttonState[c])) {
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
    static int buffer[16] = { 0 };
    static int track = 0;
    static boolean bInitialized = false;
    if (!bInitialized) {
        bInitialized = true;
        for (int i = 0; i < 15; i++)
            buffer[i] = analogRead(pin);
        track = 15;
    }
    buffer[track++] = analogRead(pin);
    if (15 < track)
        track -= 16;

    int Result = buffer[0];
    for (int i = 1; i < 16; i++)                                        // Read 16 times
        Result += buffer[i];                                            // Sum results
    return (Result >>= 2);                                              // Divide by 4 for 12 bit value;
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
    static int lastDir = NOREDRAW,
               lastSet = NOREDRAW;
    static const int* angles[4] = { &lastSet, &lastDir, &beamSet, &beamDir };

    CheckButtons();
    BeamSetting();
    BeamDirection();
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
    if ((lastDir != beamDir) || (lastSet != beamSet)) {
        DrawBeamArrows(angles);
        lastDir = beamDir;
        lastSet = beamSet;
    }
    #ifdef DEBUG
        DrawHudElement(&rawBeamDir, HUD::RawRotorPotentiometer);
        #ifdef DEBUG_ULTRAVERBOSE
            DebugPrint("----------------------------- Cycling loop() END -----------------------------\r\n");
        #endif
        #if defined(DEBUG_SLEEP)
            delay(1000);
        #endif
    #endif
}

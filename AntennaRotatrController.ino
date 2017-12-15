// Ham Antenna Rotator Device
// Version 1.00 (Rev. 2)
// 
// A SkyDubh production (http://www.skydubh.com)
// 
// Crew
// Diego Cioschi: concept and first code writing
// Bruno Passeri: code optimization and refactoring
//
// Dependencies:
// Font SevenSegmentFull.c (http://www.rinkydinkelectronics.com/r_fonts.php)
// UTFT Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)
// UTFT Geometry Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)

#include "main.h"                                                       // Main header
#include "Settings.h"                                                   // User settings take precedence (flags and debug mode)
#include "Displays.h"                                                   // Displays and initializations
#include "SerialPrint.h"                                                // Serial (and debug) print(f) helpers

/*** PIN DESIGNATION AND COMPONENTS SETTINGS *************/

const int StartStopSwitch = 12;
const int UserActionSwitch = 8;
const int SpeedControlSwitch = 13;
const int PWMSpeedControl = 9;
const int CWMotor = 10;
const int CCWMotor = 11;

const uint8_t spdSetPotentiometer = A0;
const uint8_t beamSetPotentiometer = A1;
const uint8_t rotatorSensor = A2;

#define POTENTIOMETER_MAX 1023                                          // Potentiometer range [0..x]

const int rotatorStart = 1847;
const int rotatorStop = 2245;

/*** GLOBAL VARIABLES ************************************/

#define FASTADC                                                         // Fast ADC for 12bit readings

#define MIN_AZIMUTH 0                                                   // N
#define MAX_AZIMUTH 359                                                 // N, 1° W

#define PI_OVER_180 0.01745329251994329576923690768489                  // Pi/180. Trigonometry is fun!

#define OVERLAP_SIZE 20                                                 // Overlap warning arrows

int beamDir = 0;                                                        // Actual beam direction
int beamSet = 0;                                                        // Beam directione to set
int spdValue = 0;                                                       // Rotation speed

boolean bMoveAntenna = false;                                           // Start Stop flag
boolean bSpeedModeAuto = true;                                          // Speed Mode Flag
boolean bChoosingNewAngle = false;                                      // User Action flag

/*** BUTTON MAPPING AND EVENT TRIGGERS *******************/

const BUTTON_MAP ButtonsMap[] = {
    { UserActionSwitch, UserSetConfirmToggle },
    { StartStopSwitch, StartStopToggle },
    { SpeedControlSwitch, AutoManualToggle },
};
#define BUTTON_COUNT sizeof ButtonsMap / sizeof ButtonsMap[0]

/*** EXTERNALLY DEFINED **********************************/

extern AREA speedMeter;                                                 // Speed meter area
extern COMPASS compass;                                                 // Compass basic coordinates
extern POINT overlapAlert;                                              // Overlap alert coordinates
extern UTFT* display;                                                   // Main display
extern UTFT_Geometry* geo;                                              // Geometric helper functions

/*******************************************************************
**** THERE BE DRAGONS **********************************************
*******************************************************************/

// Sensors reading helpers
#define ReadBeamDir() map(AnalogRead12Bits(rotatorSensor), rotatorStart, rotatorStop, MIN_AZIMUTH, MAX_AZIMUTH)
#define ReadBeamSet() map(analogRead(beamSetPotentiometer), 0, POTENTIOMETER_MAX, MIN_AZIMUTH, MAX_AZIMUTH)

// Arduino board bootstrap setup
void setup() {
    Serial.begin(9600);

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
    DrawInitialScreen();
    ConfigureIOPins();
    BeamSetting(true);
    BeamDirControl(true);
}

// Main loop
void loop() {
    DebugPrint("---------------------------- Cycling loop() START ----------------------------\n");
    DebugPrintf("RAW value of rotator potentiometer == %d\n", AnalogRead12Bits(rotatorSensor));
    DebugPrintf("Value of the start/stop flag == %s\n", bMoveAntenna ? "Start" : "Stop");
    DebugPrintf("Value of the Auto/Manual flag == %s\n", bSpeedModeAuto ? "Automatic" : "Manual");
    DebugPrintf("Value of the User Action flag == %s\n", bChoosingNewAngle ? "Setting" : "Confirmed");
    DebugPrintf("Value of the BEAM direction == %d\n", beamDir);
    DebugPrintf("Value of the BEAM setting == %d\n", beamSet);
    DebugPrintf("Value of the throttle setting == %d\n", spdValue);
    CheckButtons();
    StartStopAction();
    AutoManualAction();
    BeamSetting();
    BeamDirControl();
    DebugPrint("----------------------------- Cycling loop() END -----------------------------\n");
    #ifdef DEBUG
        delay(1000);
    #endif
}

// Print the angle
void UserPrintAngle(const int& x, const int& y, int angle, const COLORS& color, const BHTYPE& type) {
    static COLORS lastla[2] = { (COLORS)0 };                            // Prev. colors
    static COLORS lastra[2] = { (COLORS)0 };
    static boolean bWasLastOver = false;
    COLORS la = COLORS::Black;                                          // Left arrow
    COLORS ra = COLORS::Black;                                          // Right arrow
    boolean bIsOver = false;

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
    if (bIsOver != bWasLastOver) {
        bWasLastOver = bIsOver;
        UserPrint(/*overlapAlert.x*/RIGHT, overlapAlert.y, ("OVER"), bIsOver ? COLORS::Red : COLORS::Black);
    }

    // SSF font size: 32x50 pixels
    #define MIDARROW (50 >> 1)
    #define LOWY (MIDARROW + (OVERLAP_SIZE >> 1))
    #define HIGHY (MIDARROW - (OVERLAP_SIZE >> 1))
    #define RIGHTMOST (10 + OVERLAP_SIZE + (32 * 3))
    if (lastla[type] != la)
    {
        lastla[type] = la;
        display->setColor(la);
        geo->fillTriangle(x, y + MIDARROW, x + OVERLAP_SIZE, y + HIGHY, x + OVERLAP_SIZE, y + LOWY);
    }
    if (lastra[type] != ra)
    {
        lastra[type] = ra;
        display->setColor(ra);
        geo->fillTriangle(x + RIGHTMOST + OVERLAP_SIZE, y + MIDARROW, x + RIGHTMOST, y + LOWY, x + RIGHTMOST, y + HIGHY);
    }
    #undef RIGHTMOST
    #undef HIGHY
    #undef LOWY
    #undef MIDARROW

    char buff[4];                                                       // 3 digits + '\0'
    sprintf(buff, "%03d", angle);
    UserPrint(x + 5 + OVERLAP_SIZE, y, buff, color, SevenSegmentFull);
}

// Draw the beam
void DrawBeamHead(int oldAngle, int angle, const BHTYPE& type) {
    static boolean bInitialized = false;
    static int x2[360], y2[360], x3[360], y3[360], x4[360], y4[360];
    int h = (BHTYPE::BeamDIR == type ? 12 : 10);
    int w = 10;

    // For faster drawing and reduced computation, we pre-build a lookup tables for the heavy lifting computation
    if (!bInitialized) {
        bInitialized = true;
        int x2a, y2a, dx, dy;
        for (int a = 0; a < 360; a++) {
            x2[a] = (compass.radius * .9 * cos((a - 90) * PI_OVER_180)) + compass.X;
            y2[a] = (compass.radius * .9 * sin((a - 90) * PI_OVER_180)) + compass.Y;
            dx = compass.X + (w / 6) * (x2[a] - compass.X) / h;
            dy = compass.Y + (w / 6) * (y2[a] - compass.Y) / h;
            x2a = compass.X - dx;
            y2a = dy - compass.Y;
            x3[a] = y2a + dx;
            y3[a] = x2a + dy;
            x4[a] = dx - y2a;
            y4[a] = dy - x2a;
        }
    }

    // Ignore overlaps
    if (0 > oldAngle)   oldAngle += 360;
    if (359 < oldAngle) oldAngle -= 360;
    if (0 > angle)         angle += 360;
    if (359 < angle)       angle -= 360;

    #define colorDir COLORS::Red
    #define colorSet COLORS::Green

    // First pass erases the old one, second pass draws at the new angle
    for (int i = 0; i < 2; i++) {
        int a = (0 == i ? oldAngle : angle);
        int color = (0 == i ? COLORS::Black : (BHTYPE::BeamDIR == type ? colorDir : colorSet));
        switch (type) {
            case BHTYPE::BeamDIR: {
                display->setColor(color);
                geo->fillTriangle(x2[a], y2[a], x3[a], y3[a], x4[a], y4[a]);
                geo->fillTriangle(x3[a], y3[a], compass.X, compass.Y, x4[a], y4[a]);
                break;
            }
            case BHTYPE::BeamSET: {
                display->setColor(color);
                display->drawLine(x3[a], y3[a], compass.X, compass.Y);
                display->drawLine(compass.X, compass.Y, x4[a], y4[a]);
                display->drawLine(x4[a], y4[a], x2[a], y2[a]);
                display->drawLine(x2[a], y2[a], x3[a], y3[a]);
                display->drawLine(compass.X, compass.Y, x2[a], y2[a]);
                break;
            }
        }
    }

    // Always redraw the pivot
    display->setColor(colorDir);
    display->fillCircle(compass.X, compass.Y, 9);
}

// Update the azimuthal beam direction
inline void BeamDirControl(const bool& forceUpdate) {
    static int lastBDReading = 0;
    COLORS color = COLORS::Green;
    if (forceUpdate || (beamSet != beamDir)) {
        if (beamSet != beamDir)
            color = COLORS::Yellow;
        // Replace the old Azimut direction beam with the current direction
        int r = ReadBeamDir();
        if (r != lastBDReading) {
            beamDir = r;
            DrawBeamHead(lastBDReading, beamDir, BeamDIR);
            lastBDReading = beamDir;
        }
    }
    UserPrintAngle(0, 113, beamDir, color, BeamDIR);
}

// Update the azimuthal beam setting direction
inline void BeamSetting(const bool& forceUpdate) {
    static int lastBSReading = 0;
    COLORS color = COLORS::Green;
    if (forceUpdate || (bChoosingNewAngle)) {
        if (!forceUpdate)
            color = COLORS::Yellow;
        // Update the angle and define the shortest route to it
        int r = ReadBeamSet();
        if (r != lastBSReading) {
            beamSet = r;
            if (OVERLAP_TOLERANCE >= (360 - r))
                beamSet = abs(beamDir - (r - 360)) < abs(beamDir - r) ? r - 360 : r;
            if (OVERLAP_TOLERANCE >= (r + 1))
                beamSet = abs(beamDir - (r + 360)) < abs(beamDir - r) ? r + 360 : r;
            DrawBeamHead(lastBSReading, beamSet, BeamSET);
            lastBSReading = beamSet;
        }
    }
    UserPrintAngle(0, 213, beamSet, color, BeamSET);
}

// Toggles the Set/Confirm state [CB]
void UserSetConfirmToggle() {
    bChoosingNewAngle = !bChoosingNewAngle;
    DebugPrintf("UserActionSwitch has been pushed\nNew status of User Action flag == %s\n", bChoosingNewAngle ? "Setting" : "Confirmed");
}

// Toggles the Start/Stop state [CB]
void StartStopToggle() {
    bMoveAntenna = !bMoveAntenna;
    DebugPrintf("StartStopSwitch has been pushed\nNew status of the start/stop flag == %s\n", bMoveAntenna ? "Start" : "Stop");
}

// Toggles the Auto/Manual state [CB]
void AutoManualToggle() {
    bSpeedModeAuto = !bSpeedModeAuto;
    DebugPrintf("SpeedControlSwitch has been pushed\nNew status of the Auto/Manual flag == %s\n", bSpeedModeAuto ? "Automatic" : "Manual");
    UserPrint(RIGHT, 12, bSpeedModeAuto ? ("  Auto ") : ("Manual "), COLORS::Yellow);
 }

void StartStopAction() {
    static const char* msg = nullptr;
    static const char* last = nullptr;
    
    DebugPrint("Entering StartStopAction()\n");

    uint8_t cw = LOW;
    uint8_t ccw = LOW;

    if (bMoveAntenna) {
        if (beamDir == beamSet) {
            bMoveAntenna = false;
            msg = ("    ");
        } else if (beamDir < beamSet) {
            cw = HIGH;
            msg = (" CW ");
        } else if (beamDir > beamSet) {
            ccw = HIGH;
            msg = ("CCW ");
        }
    }
    digitalWrite(CWMotor, cw);
    digitalWrite(CCWMotor, ccw);
    if (last != msg)
    {
        last = msg;
        UserPrint(RIGHT, 25, msg, COLORS::Yellow);
    }
    DebugPrint("Exiting StartStopAction()\n");
}

// Draw the speed with a s-meter
inline void SpeedMeter(const int& speed) {
    #define PADDING 2
    int minX = speedMeter.tl.x + PADDING;
    int maxX = speedMeter.br.x - PADDING;
    int minY = speedMeter.br.y - PADDING;
    int maxY = speedMeter.tl.y + PADDING;
    #undef PADDING
    int mappedSpeed = map(speed, 0, 255, minY, maxY);
    display->setColor(COLORS::Black);
    display->fillRect(maxX, maxY, minX, mappedSpeed);
    display->setColor(COLORS::Red);
    display->fillRect(maxX, mappedSpeed, minX, minY);
}

void AutoManualAction() {
    DebugPrint("Entering AutoManualAction()\n");
    int rawSpdValue;
    if (!bSpeedModeAuto) {
        rawSpdValue = analogRead(spdSetPotentiometer);
        spdValue = map(rawSpdValue, 0, POTENTIOMETER_MAX, 0, 255);
        analogWrite(PWMSpeedControl, spdValue);
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
        analogWrite(PWMSpeedControl, spdValue);
    }
    display->setColor(COLORS::Yellow);
    display->setFont(BigFont);
    display->printNumI(spdValue, RIGHT, 38, 3, ' ');
    SpeedMeter(spdValue);
    DebugPrint("Exiting AutoManualAction()\n");
}

// Checks buttons status and fires corresponding events
void CheckButtons() {
    // Keep track of buttons status, to avoid trigger ghosting/multi-fire
    static BUTTON_STATE buttonState[BUTTON_COUNT] = { BUTTON_STATE::Released };
    for (int c = 0; c < BUTTON_COUNT; c++) {
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
    pinMode(PWMSpeedControl, OUTPUT);
    pinMode(CWMotor, OUTPUT);
    pinMode(CCWMotor, OUTPUT);
    digitalWrite(CWMotor, LOW);
    digitalWrite(CCWMotor, LOW);
    analogReference(DEFAULT);
}

// Draw the screen overlay
void DrawInitialScreen() {
    int dxOuter, dyOuter, dxinner, dyinner;
    display->setColor(0, 255, 0);
    display->drawCircle(compass.X, compass.Y, compass.radius);
    for (float i = 0; i < 360; i += 22.5) {
        display->setColor(255, 128, 0);
        dxOuter = compass.radius * cos((i - 90) * PI_OVER_180);
        dyOuter = compass.radius * sin((i - 90) * PI_OVER_180);
        dxinner = dxOuter * 0.97;
        dyinner = dyOuter * 0.97;
        display->drawLine(dxOuter + compass.X, dyOuter + compass.Y, dxinner + compass.X, dyinner + compass.Y);
        if (0 == (i - floor(i))) {
            dxinner = dxOuter * 0.92;
            dyinner = dyOuter * 0.92;
            display->drawLine(dxinner + compass.X, dyinner + compass.Y, dxOuter + compass.X, dyOuter + compass.Y);
        }
    }
    UserPrint(RIGHT, 12, bSpeedModeAuto ? ("  Auto ") : ("Manual "), COLORS::Yellow);
}

// Multisampling for 12bit readings
inline int AnalogRead12Bits(uint8_t pin) {
    int Result = 0;
    analogRead(pin);                                                    // Switch ADC
    for (int i = 0; i < 16; i++) {                                      // Read 16 times
        Result += analogRead(pin);                                      // Sum results
    }
    Result >>= 2;                                                       // Divide by 4 for 12 bit value
    return Result;
}

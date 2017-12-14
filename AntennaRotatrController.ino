// Antenna Rotator Controlbox
// Version 6 Rev. 2
//
// Dependencies:
// Font SevenSegmentFull.c (http://www.rinkydinkelectronics.com/r_fonts.php)
// UTFT Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)
// UTFT Geometry Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)

#include <UTFT.h>
#include <UTFT_Geometry.h>

#include "main.h"                                                       // Main header
#include "Settings.h"                                                   // User settings take precedence (flags and debug mode)
#include "Displays.h"                                                   // Displays and initializations
#include "SerialPrint.h"                                                // Serial (and debug) print(f) helpers

/*** PIN DESIGNATION *************************************/

const int StartStopSwitch = 12;
const int UserActionSwitch = 8;
const int SpeedControlSwitch = 13;
const int PWMSpeedControl = 9;
const int CWMotor = 10;
const int CCWMotor = 11;

const uint8_t spdSetPotentiometer = A0;
const uint8_t beamSetPotentiometer = A1;
const uint8_t rotatorSensor = A2;

/*** GENERAL CONFIGURATION *******************************/

#define FASTADC 1                                                       // Fast ADC for 12bit readings

#define MIN_AZIMUTH 0                                                   // N
#define MAX_AZIMUTH 359                                                 // N, 1° W
#define POTENTIOMETER_MAX 1023                                          // Potentiometer range [0..x]

const int rotatorStart = 1847;
const int rotatorStop = 2245;

/*** BUTTON MAPPING AND EVENT TRIGGERS *******************/

const BUTTON_MAP ButtonsMap[] = {
    { UserActionSwitch, UserSetConfirmToggle },
    { StartStopSwitch, StartStopToggle },
    { SpeedControlSwitch, AutoManualToggle },
};
#define BUTTON_COUNT sizeof ButtonsMap / sizeof ButtonsMap[0]

/*** GLOBAL VARIABLES ************************************/

int beamDir = 0;                                                        // Actual beam direction
int beamSet = 0;                                                        // Beam directione to set
int spdValue = 0;                                                       // Rotation speed

boolean bMoveAntenna = false;                                           // Start Stop flag
boolean bSpeedModeAuto = true;                                          // Speed Mode Flag
boolean bChoosingNewAngle = true;                                       // User Action flag

// Arduino board bootstrap setup
void setup() {
    Serial.begin(9600);

    #if FASTADC
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
    BeamSetting();
    bChoosingNewAngle = false;
    BeamSetting();
    BeamDirControl();
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

// (Re)draw the azimutal beam direction
void BeamDirControl() {
    static int lastBDReading = 0;
    COLORS color = COLORS::Green;
    if (beamSet != beamDir) {
        color = COLORS::Yellow;
        // Replace the old Azimut direction beam with the current direction
        int r = map(AnalogRead12Bits(rotatorSensor), rotatorStart, rotatorStop, MIN_AZIMUTH, MAX_AZIMUTH);
        if (r != lastBDReading) {
            beamDir = r;
            DrawBeamHead(lastBDReading, beamDir, BeamDIR);
            lastBDReading = beamDir;
        }
    }
    UserPrintAngle(0, 113, beamDir, color);
}

// (Re)draw the azimutal beam setting direction
void BeamSetting() {
    static int lastBSReading = 0;
    COLORS color = COLORS::Green;
    if (bChoosingNewAngle) {
        color = COLORS::Yellow;
        // Update the angle and define the shortest route to it
        int r = map(analogRead(beamSetPotentiometer), 0, POTENTIOMETER_MAX, MIN_AZIMUTH, MAX_AZIMUTH);
        if (r != lastBSReading)
        {
            beamSet = r;
            if (OVERLAP_TOLERANCE >= (360 - r))
                beamSet = abs(beamDir - (r - 360)) < abs(beamDir - r) ? r - 360 : r;
            if (OVERLAP_TOLERANCE >= (r + 1))
                beamSet = abs(beamDir - (r + 360)) < abs(beamDir - r) ? r + 360 : r;
            DrawBeamHead(lastBSReading, beamSet, BeamSET);
            lastBSReading = beamSet;
        }
    }
    UserPrintAngle(0, 213, beamSet, color);
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
}

void StartStopAction() {
    DebugPrint("Entering StartStopAction()\n");

    uint8_t cw = LOW;
    uint8_t ccw = LOW;
    char msg[5] = "    ";
    if (bMoveAntenna) {
        if (beamDir == beamSet) {
            bMoveAntenna = false;
        } else if (beamDir < beamSet) {
            cw = HIGH;
            strcat(&msg[1], "CW ");
        } else if (beamDir > beamSet) {
            ccw = HIGH;
            strcat(&msg[0], "CCW ");
        }
    }
    digitalWrite(CWMotor, cw);
    digitalWrite(CCWMotor, ccw);
    display->setColor(COLORS::Yellow);
    display->setFont(BigFont);
    display->print(msg, RIGHT, 25);
    DebugPrint("Exiting StartStopAction()\n");
}

void AutoManualAction() {
    DebugPrint("Entering AutoManualAction()\n");
    int rawSpdValue;
    if (!bSpeedModeAuto) {
        rawSpdValue = analogRead(spdSetPotentiometer);
        spdValue = map(rawSpdValue, 0, POTENTIOMETER_MAX, 0, 255);
        analogWrite(PWMSpeedControl, spdValue);
        display->setColor(COLORS::Yellow);
        display->setFont(BigFont);
        display->print(F("Manual "), RIGHT, 12);
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
        display->setColor(COLORS::Yellow);
        display->setFont(BigFont);
        display->print(F("  Auto "), RIGHT, 12); 
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

void UserPrint(int x, int y, String userData, COLORS COLOR) {
    display->setColor(COLOR);
    display->setFont(BigFont);
    display->print(userData, x, y);
}

// Draw the screen overlay
void DrawInitialScreen() {
    int dxOuter, dyOuter, dxinner, dyinner;
    display->setColor(0, 255, 0);
    display->drawCircle(X, Y, dm);
    for (float i = 0; i < 360; i += 22.5) {
        display->setColor(255, 128, 0);
        dxOuter = dm * cos((i - 90) * PIover180);
        dyOuter = dm * sin((i - 90) * PIover180);
        dxinner = dxOuter * 0.97;
        dyinner = dyOuter * 0.97;
        display->drawLine(dxOuter + X, dyOuter + Y, dxinner + X, dyinner + Y);
        if (0 == (i - floor(i))) {
            dxinner = dxOuter * 0.92;
            dyinner = dyOuter * 0.92;
            display->drawLine(dxinner + X, dyinner + Y, dxOuter + X, dyOuter + Y);
        }
    }
}

// Draw the beam
void DrawBeamHead(int oldAngle, int angle, const BHTYPE& type) {
    static boolean bInitialized = false;
    static float dist[360];
    static int dx[360], dy[360], x2[360], y2[360];
    if (0 > oldAngle)
        oldAngle += 360;
    if (359 < oldAngle)
        oldAngle -= 360;
    if (0 > angle)
        angle += 360;
    if (359 < angle)
        angle -= 360;
    int x2a, y2a, x3, y3, x4, y4;
    int h = (BHTYPE::BeamDIR == type ? 12 : 10);
    int w = 10;
    COLORS colorDir = COLORS::Red;
    COLORS colorSet = COLORS::Green;
    // For faster drawing and reduced computation, we pre-build a lookup tables for the heavy lifting computation
    if (!bInitialized) {
        bInitialized = true;
        for (int a = 0; a < 360; a++) {
            x2[a] = (dm * .9 * cos((a - 90) * PIover180)) + X;
            y2[a] = (dm * .9 * sin((a - 90) * PIover180)) + Y;
            dist[a] = sqrt((X - x2[a]) * (X - x2[a]) + (Y - y2[a]) * (Y - y2[a]));
            dx[a] = X + (w / 6) * (x2[a] - X) / h;
            dy[a] = Y + (w / 6) * (y2[a] - Y) / h;
        }
    }
    x2a = X - dx[oldAngle];
    y2a = dy[oldAngle] - Y;
    x3 = y2a + dx[oldAngle];
    y3 = x2a + dy[oldAngle];
    x4 = dx[oldAngle] - y2a;
    y4 = dy[oldAngle] - x2a;
    switch (type) {
        case BHTYPE::BeamDIR: {
            display->setColor(COLORS::Black);
            geo->fillTriangle(x2[oldAngle], y2[oldAngle], x3, y3, x4, y4);
            geo->fillTriangle(x3, y3, X, Y, x4, y4);
            x2a = X - dx[angle];
            y2a = dy[angle] - Y;
            x3 = y2a + dx[angle];
            y3 = x2a + dy[angle];
            x4 = dx[angle] - y2a;
            y4 = dy[angle] - x2a;
            display->setColor(colorDir);
            geo->fillTriangle(x2[angle], y2[angle], x3, y3, x4, y4);
            geo->fillTriangle(x3, y3, X, Y, x4, y4);
            break;
        }
        case BHTYPE::BeamSET: {
            display->setColor(COLORS::Black);
            display->drawLine(x3, y3, X, Y);
            display->drawLine(X, Y, x4, y4);
            display->drawLine(x4, y4, x2[oldAngle], y2[oldAngle]);
            display->drawLine(x2[oldAngle], y2[oldAngle], x3, y3);
            display->drawLine(X, Y, x2[oldAngle], y2[oldAngle]);
            x2a = X - dx[angle];
            y2a = dy[angle] - Y;
            x3 = y2a + dx[angle];
            y3 = x2a + dy[angle];
            x4 = dx[angle] - y2a;
            y4 = dy[angle] - x2a;
            display->setColor(colorSet);
            display->drawLine(x3, y3, X, Y);
            display->drawLine(X, Y, x4, y4);
            display->drawLine(x4, y4, x2[angle], y2[angle]);
            display->drawLine(x2[angle], y2[angle], x3, y3);
            display->drawLine(X, Y, x2[angle], y2[angle]);
            break;
        }
        default:
            DebugPrintf("Invalid beam type specified: %d\n", type);
            break;
    }
    display->setColor(colorDir);
    display->fillCircle(X, Y, 9);
}

// Print the angle
void UserPrintAngle (int x, int y, int angle, COLORS color) {
    // Overlaps
    if (0 > angle)
    {
        angle += 360;
        // Draw left overlap
    }
    if (359 < angle)
    {
        angle -= 360;
        // Draw right overlap
    }
    char buff[4];                                                      // 3 digit + null string terminator (\0)
    sprintf(buff, "%03d", angle);
    display->setColor(color);
    display->setFont(SevenSegmentFull);
    display->print(buff, x, y);
}

// Multisampling for 12bit readings
int AnalogRead12Bits(uint8_t pin) {
    int Result = 0;
    analogRead(pin);                                                    // Switch ADC
    for (int i = 0; i < 16; i++) {                                      // Read 16 times
        Result += analogRead(pin);                                      // Sum results
    }
    Result >>= 2;                                                       // Divide by 4 for 12 bit value
    return Result;
}

// Draw the speed with a s-meter
void SpeedMeter(const int& speed) {
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

// Overlap signial drawing helper function
/*void OverlapWarning () {        
/*    int minX = overWarnSig.tl.x;
    int maxX = overWarnSig.br.x;
    int minY = overWarnSig.br.y;
    int maxY = overWarnSig.tl.y;
    int x = minX;
    int width = (minY - maxY);
    int mid = maxY + (width >> 1);
    if (overWarn != 0) {
        display->setColor (red);
        display->setFont(BigFont);
        display->print(F("OVER"), minX+25, maxY+2);
        if (overWarn > 0) {
            x = maxX;
            width = -width;
        }
        geo->fillTriangle(x+width, minY, x+width, maxY, x, mid);
    } else {
        display->setColor (black);
        display->fillRect(maxX,maxY,minX,minY);
    }* /
}*/
/*
**  Serial print helpers with debug and printf support
*/

#include "Settings.h"
#include "Displays.h"

AREA speedMeter;
COMPASS compass;

UTFT* display = nullptr;
UTFT_Geometry* geo = nullptr;

void UserPrint(const int& x, const int& y, const char *userData, const COLORS& color, uint8_t* font) {
    display->setColor(color);
    display->setFont(font);
    display->print(userData, x, y);
}
inline void UserPrint(const int& x, const int& y, const __FlashStringHelper *userData, const COLORS& color, uint8_t* font) {
    UserPrint(x, y, (const char *)userData, color, font);
}

inline void SetupLayout() {
    // Initialize the screen
    display->InitLCD(LANDSCAPE);
    display->clrScr();
    display->setFont(BigFont);

    // Draw the reference display
    UserPrint(0, 12, ("ANTENNA ROTATOR"), COLORS::Orange);
    UserPrint(40, 36, ("CONTROLLER"), COLORS::Orange);
    display->drawLine(compass.X+(compass.radius-10), compass.Y, compass.X+(compass.radius+10), compass.Y);
    display->drawLine(compass.X-(compass.radius-10), compass.Y, compass.X-(compass.radius+10), compass.Y);
    display->drawLine(compass.X, compass.Y-(compass.radius-10), compass.X, compass.Y-(compass.radius+10));
    display->drawLine(compass.X, compass.Y+(compass.radius-10), compass.X, compass.Y+(compass.radius+10));
    display->drawCircle(compass.X, compass.Y, compass.radius);
    UserPrint(0, 75, ("BEAM DIR"), COLORS::Red);
    UserPrint(0, 175, ("BEAM SET"), COLORS::Red);
    UserPrint(0, 290, (QRZ ": " NAME), COLORS::White);
    UserPrint((compass.X-8), (compass.Y-(compass.radius+27)), ("N"), COLORS::Red);
    UserPrint((compass.X-8), (compass.Y+(compass.radius+15)), ("S"), COLORS::Red);
    UserPrint((compass.X+(compass.radius+13)), (compass.Y-7), ("E"), COLORS::Red);
    UserPrint((compass.X-(compass.radius+30)), (compass.Y-7), ("W"), COLORS::Red);
    display->setColor(COLORS::Green);
    display->drawRect(speedMeter.tl.x,speedMeter.tl.y,speedMeter.br.x,speedMeter.br.y);
}

#ifdef TFT_HVGA_480x320
// Initializes the TFTLCD 3.2 HVGA 480x320 display shield
inline void InitializeDisplayHVGA480x320() {
    // Create the display objects
    display = new UTFT(ILI9481, 38, 39, 40, 41);
    geo = new UTFT_Geometry(display);

    compass = { 285, 160, 120 };
    speedMeter = { { 445, 58 }, { 470, 310 } };
    //overWarnSig = { { 320, 290 }, { 434, 310 } };

    SetupLayout();
}
#endif
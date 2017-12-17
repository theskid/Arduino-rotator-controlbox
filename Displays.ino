/*
**  Serial print helpers with debug and printf support
*/

#include "Settings.h"
#include "Displays.h"

#if defined(TFT_HVGA_480x320)
    #define WIDTH 480
    #define HEIGHT 320
    UTFT display(ILI9481, 38, 39, 40, 41);
#elif defined(ADAFRUIT_ILI9341_S5) || defined(PROTEUS_VSM)
    #define WIDTH 320
    #define HEIGHT 240
    UTFT display(ILI9341_S5P, MOSI, SCK, 10 /*DC*/, NOTINUSE, 9 /*CS*/);
#endif

UTFT_Geometry geo(&display);

AREA speedMeter;
COMPASS compass;
POINT overlapAlert;

#if (480 > WIDTH)
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
uint8_t* _main = SmallFont;
uint8_t* _angles = BigFont;
#else
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];
uint8_t* _main = BigFont;
uint8_t* _angles = SevenSegNumFont;
#endif

void UserPrint(const int& x, const int& y, const char *userData, const COLORS& color, const UI_FONT& font) {
    display.setColor(color);
    switch (font) {
        case UI_FONT::Main: display.setFont(_main); break;
        case UI_FONT::Angles: display.setFont(_angles); break;
    };
    display.print(userData, x, y);
}
inline void UserPrint(const int& x, const int& y, const __FlashStringHelper *userData, const COLORS& color, const UI_FONT& font) {
    UserPrint(x, y, (const char *)userData, color, font);
}

void InitializeDisplay() {
    // Initialize the screen
    display.InitLCD(LANDSCAPE);
    display.clrScr();

    // Compass 60% 50% 33%
    compass = { int(WIDTH * 0.6), HEIGHT >> 1, int(HEIGHT * 0.3333333333333333333333333333) };
    speedMeter = { { WIDTH - 35, int(HEIGHT * 0.18) }, { WIDTH - 10, compass.Y + compass.radius } };
    overlapAlert = { 345, 292 };    // Right align

    // Draw the reference display
    UserPrint(0, 12, ("ANTENNA ROTATOR"), COLORS::Orange);
    UserPrint(40, 36, ("CONTROLLER"), COLORS::Orange);
    display.drawLine(compass.X+(compass.radius-10), compass.Y, compass.X+(compass.radius+10), compass.Y);
    display.drawLine(compass.X-(compass.radius-10), compass.Y, compass.X-(compass.radius+10), compass.Y);
    display.drawLine(compass.X, compass.Y-(compass.radius-10), compass.X, compass.Y-(compass.radius+10));
    display.drawLine(compass.X, compass.Y+(compass.radius-10), compass.X, compass.Y+(compass.radius+10));
    //display.drawCircle(compass.X, compass.Y, compass.radius);
    UserPrint(0, 75, ("BEAM DIR"), COLORS::Red);
    UserPrint(0, 175, ("BEAM SET"), COLORS::Red);
    UserPrint(0, 290, (QRZ ": " NAME), COLORS::White);
    UserPrint((compass.X-8), (compass.Y-(compass.radius+27)), ("N"), COLORS::Red);
    UserPrint((compass.X-8), (compass.Y+(compass.radius+15)), ("S"), COLORS::Red);
    UserPrint((compass.X+(compass.radius+13)), (compass.Y-7), ("E"), COLORS::Red);
    UserPrint((compass.X-(compass.radius+30)), (compass.Y-7), ("W"), COLORS::Red);
    display.setColor(COLORS::Green);
    display.drawRect(speedMeter.tl.x,speedMeter.tl.y,speedMeter.br.x,speedMeter.br.y);
}
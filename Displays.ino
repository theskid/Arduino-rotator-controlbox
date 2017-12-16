/*
**  Serial print helpers with debug and printf support
*/

#include "Settings.h"
#include "Displays.h"

AREA speedMeter;
COMPASS compass;
POINT overlapAlert;

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
    // Initialize the geometry helper
    geo = new UTFT_Geometry(display);

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

/*
    *** 8 & 16 bit ***
    Model, RS, WR, CS, RST[, ALE]
    RS: Register select
    WR: Write pin
    CS: Chip select
    RST: Reset
    ALE: (Optional) 16bit shields, Latch signal pin

    *** SPI ***
    Model, SDA, SCL, CS, RST[, RS]
    SDA: Serial data
    SCL: Serial clock
    CS: Chip select
    RST: Reset
    RS: (Optional) 5pin serial modules, Register select
*/

// TFTLCD 3.2 HVGA 480x320 display shield
#if defined(TFT_HVGA_480x320)
void InitializeDisplayHVGA480x320() {
    display = new UTFT(ILI9481, 38, 39, 40, 41);

    compass = { 285, 160, 120 };
    speedMeter = { { 445, 58 }, { 470, compass.Y + compass.radius } };
    overlapAlert = { 345, 292 };

    SetupLayout();
}
#endif

// Real or emulated Adafruit ILI9341 SPI 5 pins
#if defined(ADAFRUIT_ILI9341_S5) || defined(PROTEUS_VSM)
void InitializeDisplayAdafruitILI9341_S5() {
    // MOSI, SCK, <IOx CS Pin | Ada CS>, NOTINUSE, <IOx WR/D/C Pin | Ada DC>
    display = new UTFT(ILI9341_S5P, MOSI, SCK, 10, NOTINUSE, 9);
    compass = { 285, 160, 120 };
    speedMeter = { { 445, 58 }, { 470, compass.Y + compass.radius } };
    overlapAlert = { 345, 292 };

    SetupLayout();
}
#endif
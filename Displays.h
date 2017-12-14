/*
**  Serial print helpers with debug and printf support
*/
#pragma once

// Font declarations
extern uint8_t BigFont[];
extern uint8_t SmallFont[];
extern uint8_t SevenSegmentFull[];

AREA speedMeter;
//COMPASS compass;
int X, Y, dm;

UTFT* display = nullptr;
UTFT_Geometry* geo = nullptr;

#ifdef TFT_HVGA_480x320
// Initializes the TFTLCD 3.2 HVGA 480x320 display shield
inline void InitializeDisplayHVGA480x320() {
    // Create the display objects
    display = new UTFT(ILI9481, 38, 39, 40, 41);
    geo = new UTFT_Geometry(display);

    X = 285;
    Y = 160;
    dm = 120;
    speedMeter = { { 445, 58 }, { 470, 310 } };
    //overWarnSig = { { 320, 290 }, { 434, 310 } };
    // Initialize the screen
    display->InitLCD(LANDSCAPE);
    display->clrScr();
    display->setFont(BigFont);

    // Draw the reference display
    UserPrint(0, 12, "ANTENNA ROTATOR", COLORS::Orange);
    UserPrint(40, 36, "CONTROLLER", COLORS::Orange);
    display->drawLine(X+(dm-10), Y, X+(dm+10), Y);
    display->drawLine(X-(dm-10), Y, X-(dm+10), Y);
    display->drawLine(X, Y-(dm-10), X, Y-(dm+10));
    display->drawLine(X, Y+(dm-10), X, Y+(dm+10));
    display->drawCircle(X, Y, dm);
    UserPrint(0, 75, "BEAM DIR", COLORS::Red);
    UserPrint(0, 175, "BEAM SET", COLORS::Red);
    UserPrint(0, 290, QRZ + " : " + NAME, COLORS::White);
    UserPrint((X-8), (Y-(dm+27)), "N", COLORS::Red);
    UserPrint((X-8), (Y+(dm+15)), "S", COLORS::Red);
    UserPrint((X+(dm+13)), (Y-7), "E", COLORS::Red);
    UserPrint((X-(dm+30)), (Y-7), "W", COLORS::Red);
    display->setColor(COLORS::Green);
    display->drawRect(speedMeter.tl.x,speedMeter.tl.y,speedMeter.br.x,speedMeter.br.y);
}
#define InitializeDisplay InitializeDisplayHVGA480x320
#endif
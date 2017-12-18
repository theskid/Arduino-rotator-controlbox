/*
**  Serial print helpers with debug and printf support
*/

#include "Settings.h"
#include "Global.h"
#include "Displays.h"
#include "SerialPrint.h"

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

typedef struct {
    uint8_t* data;
    byte w;
    byte h;
} FONT_DATA;

#if (480 > WIDTH)
    #define MFS SmallFont
    #define AFS BigFont
    extern uint8_t MFS[];
    extern uint8_t AFS[];
    const FONT_DATA _main = { MFS, 8, 12 };
    const FONT_DATA _angles = { AFS, 16, 16 };
    #define OVERLAP_SIZE 6
#else
    #define MFS BigFont
    #define AFS SevenSegNumFont
    extern uint8_t MFS[];
    extern uint8_t AFS[];
    const FONT_DATA _main = { &MFS[0], 16, 16 };
    const FONT_DATA _angles = { &AFS[0], 32, 50 };
    #define OVERLAP_SIZE 20
#endif
#undef MFS
#undef AFS

#define DISPLAY_MARGIN (_main.h >> 1)

// Compass 60% 50% 33%
COMPASS compass = { int(WIDTH * 0.6), HEIGHT >> 1, int(HEIGHT * 0.3333333333333333333333333333) };
AREA speedMeter = {
    { WIDTH - DISPLAY_MARGIN - (_main.w << 1) - (_main.w >> 1), int(3 * _main.h) + 2 + DISPLAY_MARGIN },
    { WIDTH - DISPLAY_MARGIN - 4 - _main.w + (_main.w >> 1), HEIGHT - _main.h - 2 - DISPLAY_MARGIN }
};

void UserPrint(const int& x, const int& y, const char *userData, const COLORS& color, const UI_FONT& font) {
    display.setColor(color);
    switch (font) {
        case UI_FONT::Main: display.setFont(_main.data); break;
        case UI_FONT::Angles: display.setFont(_angles.data); break;
    }
    display.print(userData, x, y);
}
/*inline void UserPrint(const int& x, const int& y, const __FlashStringHelper *userData, const COLORS& color, const UI_FONT& font) {
    // Load strings from flash
    //UserPrint(x, y, (const char*)userData, color, font);
}*/

void DrawHudElement(const void* data, const HUD& hud) {
    switch (hud) {
        case HUD::AutoManual: {
            UserPrint(WIDTH - (_main.w * strlen((const char*)data)) - DISPLAY_MARGIN, DISPLAY_MARGIN, (const char*)data, COLORS::Yellow);
            break;
        }
        case HUD::RotationDirection: {
            int r = WIDTH - (_main.w * 3) - DISPLAY_MARGIN;
            if (4 == strlen((const char*)data))
                r -= (_main.w >> 1);
            UserPrint(r, _main.h + DISPLAY_MARGIN, (const char*)data, COLORS::Yellow);
            break;
        }
        case HUD::OverlapAlert: {
            UserPrint(WIDTH - (_main.w << 2) - DISPLAY_MARGIN, HEIGHT - _main.h - DISPLAY_MARGIN, (const char*)data, COLORS::Red);
            break;
        }
        case HUD::CurrentSpeed: {
            char speed[5] = { 0 };
            const char* mask = " %d ";

            int x = WIDTH - (_main.w * 3) - DISPLAY_MARGIN, y = (_main.h << 1) + DISPLAY_MARGIN;
            if (100 == (int)data) {
                mask = "%d";
            }
            else if (9 < (int)data) {
                mask = " %d ";
                x -= (_main.w >> 1);
            }
            sprintf(speed, mask, (int)data);
            UserPrint(x, y, speed, COLORS::Yellow);
            break;
        }
        case HUD::SpeedMeter: {
            static int lastMap = -1;
            #define PADDING 2
            const int minX = speedMeter.tl.x + PADDING;
            const int maxX = speedMeter.br.x - PADDING;
            const int minY = speedMeter.br.y - PADDING;
            const int maxY = speedMeter.tl.y + PADDING;
            #undef PADDING
            int mappedSpeed = map((int)data, 0, 255, minY, maxY);
            if (lastMap != mappedSpeed) {
                lastMap = mappedSpeed;
                display.setColor(COLORS::Black);
                display.fillRect(maxX, maxY, minX, mappedSpeed);
                display.setColor(COLORS::Red);
                display.fillRect(maxX, mappedSpeed, minX, minY);
            }
            break;
        }
        case HUD::BeamAngle:
        case HUD::BeamLeftArrow:
        case HUD::BeamRightArrow: {
            HUD_BEAM* hb = (HUD_BEAM*)data;
            display.setColor(hb->color);
            int x = DISPLAY_MARGIN, y = ((_main.h << 1) + (HEIGHT * (BHTYPE::BeamDIR == hb->type ? 0.22 : 0.55)));
            if (HUD::BeamAngle != hud) {
                #define MIDARROW (_angles.h >> 1)
                #define LOWY (MIDARROW + (OVERLAP_SIZE >> 1))
                #define HIGHY (MIDARROW - (OVERLAP_SIZE >> 1))
                #define RIGHTMOST (10 + OVERLAP_SIZE + (_angles.w * 3))
            }
            if (HUD::BeamLeftArrow == hud) {
                geo.fillTriangle(x, y + MIDARROW, x + OVERLAP_SIZE, y + HIGHY, x + OVERLAP_SIZE, y + LOWY);
            } else if (HUD::BeamRightArrow == hud) {
                geo.fillTriangle(x + RIGHTMOST + OVERLAP_SIZE, y + MIDARROW, x + RIGHTMOST, y + LOWY, x + RIGHTMOST, y + HIGHY);
            } else {
                UserPrint(x + 5 + OVERLAP_SIZE, y, hb->angle, hb->color, UI_FONT::Angles);
            }
            break;
        }
    }
}

void InitializeDisplay() {
    // Initialize the screen
    display.InitLCD(LANDSCAPE);
    display.clrScr();

    // Draw the reference display
    #define NESW_OFFSET 5                                               // How many pixels the notches get out from the compass
    UserPrint(DISPLAY_MARGIN, DISPLAY_MARGIN, ("ANTENNA ROTATOR"), COLORS::Orange);
    UserPrint(DISPLAY_MARGIN + (2.5 * _main.w), DISPLAY_MARGIN + (_main.h * 1.25), ("CONTROLLER"), COLORS::Orange);
    #define BEAMS_CENTER (((OVERLAP_SIZE << 1) + 11 + (_angles.w * 3) - (_main.w << 3)) >> 1)
    UserPrint(DISPLAY_MARGIN + BEAMS_CENTER, int(HEIGHT * 0.22), ("BEAM DIR"), COLORS::Red);
    UserPrint(DISPLAY_MARGIN + BEAMS_CENTER, int(HEIGHT * 0.55), ("BEAM SET"), COLORS::Red);
    UserPrint(DISPLAY_MARGIN, HEIGHT-_main.h-DISPLAY_MARGIN, (QRZ ": " NAME), COLORS::White);
    UserPrint((compass.X-((_main.w>>1)-2)), (compass.Y-(compass.radius+NESW_OFFSET+_main.h)), ("N"), COLORS::Red);
    UserPrint((compass.X-((_main.w>>1)-2)), (compass.Y+(compass.radius+NESW_OFFSET+2)), ("S"), COLORS::Red);
    UserPrint((compass.X+(compass.radius+NESW_OFFSET)+2), (compass.Y-(_main.h>>1)), ("E"), COLORS::Red);
    UserPrint((compass.X-(compass.radius+NESW_OFFSET+_main.w)+2), (compass.Y-(_main.h>>1)), ("W"), COLORS::Red);
    display.setColor(COLORS::Green);
    display.drawRect(speedMeter.tl.x,speedMeter.tl.y,speedMeter.br.x,speedMeter.br.y);
    display.drawCircle(compass.X, compass.Y, compass.radius);
    int dxOuter, dyOuter, step = -1;
    display.setColor(255, 128, 0);
    for (float i = 0; i < 360; i += 11.25) {
        if (7 < ++step)
            step = 0;
        dxOuter = (compass.radius + (0 < step ? 0 : NESW_OFFSET)) * cos((i - 90) * PI_OVER_180);
        dyOuter = (compass.radius + (0 < step ? 0 : NESW_OFFSET)) * sin((i - 90) * PI_OVER_180);
        float mod = 0.97;                                               // 22.5°
        if (0 == step)                                                  // 90°
            mod = 0.82;
        else if (0 == (step % 2))                                       // 45°
            mod = 0.92;
        
        display.drawLine(dxOuter + compass.X, dyOuter + compass.Y, (dxOuter * mod) + compass.X, (dyOuter * mod) + compass.Y);
    }
}

/*
**  Serial print helpers with debug and printf support
*/
#pragma once

#include <UTFT.h>
#include <UTFT_Geometry.h>

typedef struct {
    int x;
    int y;
} POINT;                                                                // Point coordinates

typedef struct {
    POINT tl;                                                           // Top left corner
    POINT br;                                                           // Bottom right corner
} AREA;                                                                 // Area coordinates

typedef struct {
    int X;
    int Y;
    int radius;
} COMPASS;

typedef enum {
    Main = 1,
    Angles = 2
} UI_FONT;

typedef enum {
    Black   = 0x0000,
    Blue    = 0x001F,
    Red     = 0xF800,
    Green   = 0x07E0,
    Cyan    = 0x07FF,
    Magenta = 0xF81F,
    Yellow  = 0xFFE0,
    White   = 0xFFFF,
    Orange  = 0xF400,
} COLORS;                                                               // UTFT Display module color constants

typedef struct {
    BHTYPE type;
    const char* angle;
    COLORS color;
} HUD_BEAM;

typedef enum {
    RotationDirection = 1,
    CurrentSpeed = 2,
    AutoManual = 3,
    OverlapAlert = 4,
    SpeedMeter = 5,
    BeamAngle = 6,
    BeamLeftArrow = 7,
    BeamRightArrow = 8,
} HUD;

#define NOREDRAW 0x7FFE                                                 // Initial state of lastBeam/lastSet

void InitializeDisplay();
void DrawHudElement(const void* data, const HUD& hud);
void DrawBeamArrows(const int* angles[4]);
void UserPrint(const int& x, const int& y, const char *userData, const COLORS& color, const UI_FONT& font = UI_FONT::Main);
//inline void UserPrint(const int& x, const int& y, const __FlashStringHelper* userData, const COLORS& color, const UI_FONT& font = UI_FONT::Main);

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

#ifndef ENUM_COLORS
typedef enum {
    Black   = 0x0000,
    Blue    = 0x001F,
    Red     = 0xF800,
    Green   = 0x07E0,
    Cyan    = 0x07FF,
    Magenta = 0xF81F,
    Yellow  = 0xFFE0,
    White   = 0xFFFF,
    Orange  = 0xF400
} COLORS;                                                               // UTFT Display module color constants
#define ENUM_COLORS
#endif

void InitializeDisplay();
void UserPrint(const int& x, const int& y, const char *userData, const COLORS& color, const UI_FONT& font = UI_FONT::Main);
inline void UserPrint(const int& x, const int& y, const __FlashStringHelper* userData, const COLORS& color, const UI_FONT& font = UI_FONT::Main);
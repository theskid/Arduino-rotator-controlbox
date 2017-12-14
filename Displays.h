/*
**  Serial print helpers with debug and printf support
*/
#pragma once

#include <UTFT.h>
#include <UTFT_Geometry.h>

// Font declarations
extern uint8_t BigFont[];
extern uint8_t SmallFont[];
extern uint8_t SevenSegmentFull[];

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

#ifdef TFT_HVGA_480x320
#define InitializeDisplay InitializeDisplayHVGA480x320
#endif

void InitializeDisplay();
void UserPrint(const int& x, const int& y, const __FlashStringHelper *userData, const COLORS& color);    
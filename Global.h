/*
**  Main header
*/
#pragma once

typedef struct {
    int DigitalPin;
    void (*EventFunction)();
} BUTTON_MAP;                                                           // Button callbacks map

typedef enum {
    BeamDIR = 1,
    BeamSET = 0,
} BHTYPE;                                                               // Arrow beam for current and planned directions

typedef enum {
    Released = 0,
    Pressed = 1,
    Held = 2,                                                           // Currently unused
} BUTTON_STATE;                                                         // Button states

/* Debug settings expansion */
#if ((!defined(DEBUG)) && (defined(DEBUG_VERBOSE) || defined(DEBUG_ULTRAVERBOSE) || defined(DEBUG_SLEEP)))
    #define DEBUG
#endif
#if ((!defined(DEBUG_VERBOSE)) && defined(DEBUG_ULTRAVERBOSE))
    #define DEBUG_VERBOSE
#endif

/* Frame skips expansion */
#if ((!defined(FRAME_SKIPS)) || (0 == FRAME_SKIPS))
#define DISABLE_SKIPPING
#endif
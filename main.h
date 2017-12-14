/*
**  Main header
*/

/*** STRUCTS ***********************************/

typedef struct {
    int x;
    int y;
} POINT;                                                                // Point coordinates

typedef struct {
    POINT tl;                                                           // Top left corner
    POINT br;                                                           // Bottom right corner
} AREA;                                                                 // Area coordinates

typedef struct {
    int DigitalPin;
    void (*EventFunction)();
} BUTTON_MAP;                                                           // Button callbacks map

/*** ENUMS *************************************/

typedef enum {
    BeamDIR = 1,
    BeamSET = 2,
} BHTYPE;                                                               // Arrow beam for current and planned directions

typedef enum {
    Released = 0,
    Pressed = 1,
    Held = 2,                                                           // Currently unused
} BUTTON_STATE;                                                         // Button states

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

/*** CONSTANTS *********************************/

const float PIover180 = 3.1415926535897932384626433832795 / 180;        // Trigonometry is fun!

/*** FUNCTION DECLARATIONS *********************/

inline void ConfigureIOPins();                                          // Pins initialization
inline void DrawInitialScreen();                                        // Initial screen overlay
void UserPrint(int x, int y, String userData, COLORS color);            // String printer helper function
void DrawBeamHead(int oldAngle, int angle, const BHTYPE& type);         // Beam drawing helper function
inline void UserPrintAngle(int x, int y, int userAngle, COLORS color);  // Angle drawing through SevenSegmentFull font
inline void CheckButtons();                                             // Button tracking
void StartStopToggle();                                                 // Start/Stop button event toggle
inline void StartStopAction();                                          // Start/Stop in-loop actions
void AutoManualToggle();                                                // Auto/Manual button event toggle
inline void AutoManualAction();                                         // Auto/Manual in-loop actions
void UserSetConfirmToggle();                                            // Set/Confirm button event toggle
inline void BeamSetting(const bool& forceUpdate = false);               // Azimut setting potentiometer read
inline void BeamDirControl(const bool& forceUpdate = false);            // Azimut rotor potentiometer read
inline int AnalogRead12Bits(uint8_t pin);                               // 12-bits oversampled analogread 
void SpeedMeter(const int& speed);                                      // Speedmeter drawing helper function

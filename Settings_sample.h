/*
** Configuration sample, rename to Settings.h and change your parameters accordingly
*/

// Your QRZ
#define QRZ "QRZ"

// Your Name
#define NAME "Name"

//  The amount of degrees overlap allowance. Must be a number between 0 and 90.
//
//  "OVERLAP_TOLERANCE 5" indicates that the rotor turn can overlap 5 degrees in
//  both directions [-5..364] instead of the standard [0..359].
//
#define OVERLAP_TOLERANCE 15

//  Start rotation of the potentiometer partition mounted on the rotor motor.
//
//  "ROTATION_BEGIN 1200" indicates that the rotor turn starts where the potentiometer 
//  displays, in debug mode, a raw value equal to 1200.
//
#define ROTATION_BEGIN 0

//  End rotation of the potentiometer partition mounted on the rotor motor.
//
//  "ROTATION_END 1400" indicates that the rotor turn end where the potentiometer 
//  displays, in debug mode, a raw value equal to 1400.
//
#define ROTATION_END 4092


//  IMPORTANT!
//
//  The display model to initialize. Uncomment the one the matches your display shield.
//
//#define PROTEUS_VSM                       1       /* For the emulator, ILI9341 S shield */
//#define TFT_HVGA_480x320                  1       /* ILI9481 */
//#define ADAFRUIT_ILI9341_S5               1       /* ILI9341 Shield, SPI 5 pins */



/*
**  HIC SUNT DRACONES – THERE BE DRAGONS!
**
**  Only go further if you know what you're doing!
*/



//  Pin configuration
//
//  If you didn't follow the original schematics, update the pin configuration
//  to match your setup.
//
#define BUTTON_START_STOP                   12      /* Start/Stop */
#define BUTTON_CHANGE_PLANNED_DIRECTION      8      /* Setting/Confirmed */
#define BUTTON_AUTO_MANUAL                  13      /* Auto/Manual */
#define PWM_SPEED_CONTROL                    9      /* Manual speed value */
#define MOTOR_CLOCKWISE_SIGNAL              10      /* Spin the rotor clockwise */
#define MOTOR_COUNTERCLOCKWISE_SIGNAL       11      /* Spin the rotor counter clockwise */
#define POTENTIOMETER_MANUAL_SPEED          A0      /* Manually control the speed you want applied to the rotor */
#define POTENTIOMETER_PLANNED_DIRECTION     A1      /* Select the new angle for the antenna beam */
#define POTENTIOMETER_ROTOR_SENSOR          A2      /* Detect the current rotor position (angle) */

// (Un)comment to enable/disable debug mode.

#define DEBUG                                       /* Minimal debug and setup informations */
//#define DEBUG_VERBOSE                             /* Basic debug informations */
//#define DEBUG_ULTRAVERBOSE                        /* Open the floods */
//#define DEBUG_SLEEP                               /* Wait 1 second after every loop */
//#define DEBUG_WAITONSERIAL                        /* Wait for USB Serial to be connected while booting to avoid board resets */

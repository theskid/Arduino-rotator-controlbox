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

//  IMPORTANT!
//
//  The display model to initialize. Uncomment the one the matches your display shield.
//
//#define PROTEUS_VSM                   1   /* For the emulator, ILI9341 S shield */
//#define TFT_HVGA_480x320              1   /* ILI9481 */
//#define ADAFRUIT_ILI9341_S5           1   /* ILI9341 Shield, SPI 5 pins */

/*
**  THERE BE DRAGONS
**
**  Only go further if you know what you're doing!
*/

// (Un)comment to enable/disable debug mode.
//#define DEBUG                             /* Minimal debug informations and wait-on-serial */
//#define DEBUG_VERBOSE                     /* Basic debug informations */
//#define DEBUG_ULTRAVERBOSE                /* Open the floods */
//#define DEBUG_SLEEP                       /* Wait 1 second after every loop */
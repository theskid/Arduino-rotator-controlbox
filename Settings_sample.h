/*
** Configuration sample, rename to Settings.h and change your parameters accordingly
*/

// Your QRZ
#define QRZ "QRZ"

// Your Name
#define NAME "Name"

//  The amount of degrees overlap allowance. Must be a number between 0 and 90
//
//  "OVERLAP_TOLERANCE 5" indicates that the rotor turn can overlap 5 degrees in
//  both directions [-5..364] instead of the standard [0..359]
//
#define OVERLAP_TOLERANCE 15

//  IMPORTANT!
//
//  The display model to initialize. Uncomment the one the matches your display shield
//
//#define TFT_HVGA_480x320


/*
**  THERE BE DRAGONS
**
**  Only go further if you know what you're doing
*/

// (Un)comment to enable/disable debug mode
//#define DEBUG
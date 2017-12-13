// Configuration sample, rename to Settings.h and change your parameters accordingly
//

// Your QRZ
const String QRZ = "QRZ";

// Your Name
const String NAME = "Name";

// This number represents the amount of degrees to be performed
// This value is an integer and has a range of 0 to 90
// "const int overLapTollerance = 5;" indicates that the rotor stroke can be 5 degrees less than 0 and more than 5 compared to 359
const int overlapTolerance = 15;

// The type of display in your possession
const DISPLAY_TYPE DisplayType = DISPLAY_TYPE::TFT_HVGA_480x320;

// (Un)comment to enable/disable debug mode
//#define DEBUG
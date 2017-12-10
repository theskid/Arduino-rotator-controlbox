// Antenna Rotator Controlbox
// Version 6 Rev. 1
//
// Dependencies:
// Font SevenSegmentFull.c  (http://www.rinkydinkelectronics.com/r_fonts.php)
// UTFT Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)
// UTFT Geometry Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)

#include <UTFT.h>           
#include <UTFT_Geometry.h>  

#define FASTADC 1

/*#################### USER Configuration ####################
#                                                            #
#  type of display used                                      #
#  DT value rappreset the display that you are using         #
#  DT == 1  : 3.2 480x320 TFTLCD Shild                       #
#                                                            #
############################################################*/

const int DT = 1;               // Display used
const String QRZ = "IU6CRH";    // Your QRZ
const String NAME = "Diego";    // Your Name
#define DEBUG                 // (Un)comment to enable/disable debug mode
const int minAzimut = 0;
const int maxAzimut = 359;
const int rotatorStart = 1847;
const int rotatorStop = 2245;
const int zumEncoder = 4;

// ################# END USER Configuration #################*/

// ################# PIN Designation ################//
const int StartStopSwitch = 12;
const int UserActionSwitch = 8;
const int SpeedControlSwitch = 13;
const int ManualSpeedCtrl = 9;
const int CWMotor = 10;
const int CCWMotor = 11;
const uint8_t spdSetPotentiometer = A0;
const uint8_t beamSetPotentiometer = A1;
const uint8_t rotatorSensor = A2;

extern uint8_t BigFont[];
extern uint8_t SmallFont[];
extern uint8_t SevenSegmentFull[];

typedef enum {
  black   = 0x0000,
  blue    = 0x001F,
  red     = 0xF800,
  green   = 0x07E0,
  cyan    = 0x07FF,
  magenta = 0xF81F,
  yellow  = 0xFFE0,
  white   = 0xFFFF,
  orange  = 0xF400
} Colors;

typedef enum {
  BeamDIR = 1,
  BeamSET = 0
} HeadType;

typedef enum {
  Create = 1,
  Delete = 0
} Action;

typedef enum {
  Start     = 1,
  Stop      = 0,
  Auto      = 1,
  Manual    = 0,
  Setting   = 1,
  Confirmed = 0
} PINflag;

int X, Y, dm;

inline void InitializeDisplay(int displayType); 						        // inizializzazione del display generica
inline void ConfigureIOPins();                  						        // Inizializazione dei pins
inline void DrawInitialScreen();                						        // disegno preliminare dello schermo
void UserPrint (int x, int y, String userData, Colors COLOR);       // scrittura delle stringhe
void DrawBeamHead(int angle, HeadType headStyle, Action toDo);      // disegno delle lancette
void UserPrintAngle (int x, int y, int userAngle, Colors COLOR);    // print degli angoli con font SevenSegmentFull
inline boolean isPushed(int botton);            						        // rilevazione della pressione dei pulsanti
inline void StartStopToggle();                   						        // cambia la flag alla pressione del pulsante start/stop
inline void StartStopAction();                  						        // effettua le azioni da compiere a seconda del flag start/stop
inline void AutoManualToggle();                   						      // cambia la flag alla pressione del pulsante auto/manual
inline void AutoManualAction();                        					  	// effettua le azioni da compiere a seconda del flag auto/manual
inline void UserSetConfirmToggle();               						      // cambia la flag alla pressione del pulsante user set/confirm
void BeamSetting();                                      						// effettua la lettura del potenziometro per il settaggio dell'azimut
inline void BeamDirControl();                                       // effettua la lettura del potenziometro sotto il rotore dell'azimut
int Read12bit(uint8_t pin);                                         // 12-bits oversampled analogread 

const float PIover180 = 3.1415926535897932384626433832795 / 180;

UTFT utftDisplay(ILI9481, 38, 39, 40, 41);
UTFT_Geometry geo(&utftDisplay);

int beamDir = 1;                                                    // Actual beam direction
int beamSet = 1;                                                    // Beam directione to set
int spdValue = 1;                                                   // Rotation speed

PINflag StartStopFlag = Stop;
PINflag SpeedModeFlag = Manual;
PINflag UserActionFlag = Setting;

#ifdef DEBUG
  void DebugPrintInt(const unsigned char* expr, const int& pValue)
  {
    char buffer[512] = { 0 };
    sprintf(buffer, expr, pValue);
    Serial.print(buffer);
  }
  inline void DebugPrintMessage(const char* expr)
  {
    Serial.print(expr);
  }
#else
  #define DebugPrintInt(...) (void(0))
  #define DebugPrintMessage(...) (void(0))
#endif

void setup() {
  Serial.begin(9600);

  #if FASTADC
    // defines for setting and clearing register bits
    #define fastadc_cbit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
    #define fastadc_sbit(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

    // set prescale to 16 (1MHz)
    fastadc_sbit(ADCSRA, ADPS2);
    fastadc_cbit(ADCSRA, ADPS1);
    fastadc_cbit(ADCSRA, ADPS0);
  #endif

  InitializeDisplay(DT);
  DrawInitialScreen();
  ConfigureIOPins();
  BeamSetting();
  UserActionFlag = Confirmed;
  BeamSetting();
  BeamDirControl();
}

void loop() {
  DebugPrintMessage("------------------------------- Cycling loop() START -------------------------------\n");
  DebugPrintInt("RAW value of rotator potentiometer == %d\n", Read12bit(rotatorSensor));
  DebugPrintInt("Value of the start/stop flag == %d\n", StartStopFlag);
  DebugPrintInt("Value of the Auto/Manual flag == %d\n", SpeedModeFlag);
  DebugPrintInt("Value of the User Action flag == %d\n", UserActionFlag);
  DebugPrintInt("Value of the BEAM direction == %d\n", beamDir);
  DebugPrintInt("Value of the BEAM setting == %d\n", beamSet);
  DebugPrintInt("Value of the throttle setting == %d\n", spdValue);
  if (isPushed(StartStopSwitch)) {
    DebugPrintInt("Satus of the start/stop flag == %d\nStartStopSwitch has been pushed\n", StartStopFlag);
    StartStopToggle();
    DebugPrintInt("New satus of the start/stop flag == %d\n", StartStopFlag);
  }
  DebugPrintMessage("Entering StartStopAction()\n");
  StartStopAction();
  DebugPrintMessage("Exiting StartStopAction()\n");
  if (isPushed(SpeedControlSwitch)) {
    DebugPrintInt("Satus of the Auto/Manual flag == %d\nSpeedControlSwitch has been pushed\n", SpeedModeFlag);
    AutoManualToggle();
    DebugPrintInt("New satus of the Auto/Manual flag == %d\n", SpeedModeFlag);
  }
  DebugPrintMessage("Entering AutoManualAction()\n");
  AutoManualAction();
  DebugPrintMessage("Exiting AutoManualAction()\n");
  if (isPushed(UserActionSwitch)) {
    DebugPrintInt("UserActionSwitch has been pushed\nStatus of UserActionFlag == %d\n", UserActionFlag);
    UserSetConfirmToggle();
    DebugPrintInt("New status of UserActionFlag == %d\n", UserActionFlag);
  }
  BeamSetting();
  BeamDirControl();
  DebugPrintMessage("-------------------------------- Cycling loop() END --------------------------------\n");
  #ifdef DEBUG
    delay(1000);
  #endif
}

void BeamDirControl() {
  int rawAngle;
  Colors color = green;
  if (beamSet != beamDir) {
    DrawBeamHead(beamDir, BeamDIR, Delete);                 // Cancella la lancetta di direzione al vecchio azimut  
    rawAngle = Read12bit(rotatorSensor);                    // Leggi il potenziometro sotto al rotore
    beamDir = map(rawAngle, rotatorStart, rotatorStop, minAzimut, maxAzimut);   // Normalizza la letura del potenziometro rawAngle
    DrawBeamHead(beamDir, BeamDIR, Create);                 // Disegna la lancetta dell'azimut all'attuale posizione
    color = yellow;
  }
  UserPrintAngle(0, 113, beamDir, color);
}

void BeamSetting() {
  int rawAngle;
  Colors color = green;
  if (UserActionFlag == Setting) {
    DrawBeamHead(beamSet, BeamSET, Delete);                 // cancella la lancetta di settaggio al vecchio azimut
    rawAngle = analogRead(beamSetPotentiometer);            // leggi il potenziometro di settaggio 
    beamSet = map(rawAngle, 0, 1023, minAzimut, maxAzimut); // normalizza la lettura del potenziometro rawAngle
    DrawBeamHead(beamSet, BeamSET, Create);                 // Disegna la lancetta all'azimut corrispondente
    color = yellow;
  }
  UserPrintAngle(0, 213, beamSet, color);                   // conferma il settaggio
}

void UserSetConfirmToggle() {
  UserActionFlag = (UserActionFlag == Setting ? Confirmed : Setting);
}

void StartStopToggle() {
  StartStopFlag = (StartStopFlag == Stop ? Start : Stop);
}

void StartStopAction() {
  uint8_t cw = LOW;
  uint8_t ccw = LOW;
  char msg[5] = "    ";
  if ((!StartStopFlag) || (beamDir == beamSet)){
    StartStopFlag = Stop;
  }
  else if ((beamDir < beamSet) && (StartStopFlag)){
    cw = HIGH;
    msg[1] = 'C'; msg[2] = 'W';
  }
  else if ((beamDir > beamSet) && (StartStopFlag)){
    ccw = HIGH;
    msg[0] = 'C'; msg[1] = 'C'; msg[2] = 'W';
  }
  digitalWrite(CWMotor, cw);
  digitalWrite(CCWMotor, ccw);
  utftDisplay.setColor(yellow);
  utftDisplay.setFont(BigFont);
  utftDisplay.print(msg, RIGHT, 25);
}

void AutoManualToggle() {
  SpeedModeFlag = (SpeedModeFlag == Manual ? Auto : Manual);
}

void AutoManualAction() {
  int rawSpdValue;
  if (!SpeedModeFlag) {
    if (StartStopFlag == Stop) {
      rawSpdValue = analogRead(spdSetPotentiometer);
      spdValue = map(rawSpdValue, 0, 1023, 0, 255);
      analogWrite(ManualSpeedCtrl, spdValue);
    } else {
      spdValue = 0;
      analogWrite(ManualSpeedCtrl, spdValue);
    }
    utftDisplay.setColor(yellow);
    utftDisplay.setFont(BigFont);
    utftDisplay.print("Manual ", RIGHT, 12);
  } else {
    if (StartStopFlag == Start) {
      int rotationValue = (abs(beamSet - beamDir))*4;
      if (rotationValue <= 10) {
        spdValue = map(rotationValue, 0, 360, 0, 255); 
      }
      analogWrite(ManualSpeedCtrl, spdValue);
    } else {
      spdValue = 0;
      analogWrite(ManualSpeedCtrl, spdValue);
    }
    utftDisplay.setColor(yellow);
    utftDisplay.setFont(BigFont);
    utftDisplay.print("  Auto ", RIGHT, 12); 
  }
}

boolean isPushed(int button) {
  boolean ret = (LOW == digitalRead(button));
  delay(100);
  return ret;
}

void InitializeDisplayHVGA480x320() {
  X = 290;
  Y = 160;
  dm = 120;
  utftDisplay.InitLCD(LANDSCAPE);
  utftDisplay.clrScr();
  utftDisplay.setFont(BigFont);
  UserPrint(0, 12, "ANTENNA ROTATOR", orange);
  UserPrint(40, 36, "CONTROLLER", orange);
  utftDisplay.drawLine(X+(dm-10), Y, X+(dm+10), Y);
  utftDisplay.drawLine(X-(dm-10), Y, X-(dm+10), Y);
  utftDisplay.drawLine(X, Y-(dm-10), X, Y-(dm+10));
  utftDisplay.drawLine(X, Y+(dm-10), X, Y+(dm+10));
  utftDisplay.drawCircle(X, Y, dm);
  UserPrint(0, 75, "BEAM DIR", red);
  UserPrint(0, 175, "BEAM SET", red);
  UserPrint(0, 290, QRZ + " : " + NAME, white);
  UserPrint((X-8), (Y-(dm+27)), "N", red);
  UserPrint((X-8), (Y+(dm+15)), "S", red);
  UserPrint((X+(dm+13)), (Y-7), "E", red);
  UserPrint((X-(dm+30)), (Y-7), "W", red);
}

void InitializeDisplay(int displayNumber) {
  if (1 == displayNumber) {
    InitializeDisplayHVGA480x320();
  }
}

void ConfigureIOPins() {
  pinMode(StartStopSwitch, INPUT);
  pinMode(UserActionSwitch, INPUT);
  pinMode(SpeedControlSwitch, INPUT);
  pinMode(ManualSpeedCtrl, OUTPUT);
  pinMode(CWMotor, OUTPUT);
  pinMode(CCWMotor, OUTPUT);
  digitalWrite(CWMotor, LOW);
  digitalWrite(CCWMotor, LOW);
  analogReference(DEFAULT);
}

void UserPrint(int x, int y, String userData, Colors COLOR) {
  utftDisplay.setColor(COLOR);
  utftDisplay.setFont(BigFont);
  utftDisplay.print(userData, x, y);
}

void DrawInitialScreen() {
  int dxOuter, dyOuter, dxinner, dyinner;
  utftDisplay.setColor(0, 255, 0);
  utftDisplay.drawCircle(X, Y, dm);
  for (float i = 0; i < 360; i += 22.5) {
    utftDisplay.setColor(255, 128, 0);
    dxOuter = dm * cos((i - 90) * PIover180);
    dyOuter = dm * sin((i - 90) * PIover180);
    dxinner = dxOuter * 0.97;
    dyinner = dyOuter * 0.97;
    utftDisplay.drawLine(dxOuter + X, dyOuter + Y, dxinner + X, dyinner + Y);
    if (0 == (i - floor(i))) {
      dxinner = dxOuter * 0.92;
      dyinner = dyOuter * 0.92;
      utftDisplay.drawLine(dxinner + X, dyinner + Y, dxOuter + X, dyOuter + Y);
    }
  }
}

void DrawBeamHead(int angle, HeadType headStyle, Action toDo) {
  static boolean initialized = false;
  static float dist[360];
  static int dx[360], dy[360], x2[360], y2[360];
  int x2a, y2a, x3, y3, x4, y4;
  int h = (headStyle ? 12 : 10);
  int w = 10;
  Colors colorDir = red;
  Colors colorSet = green;

  // For faster drawing and reduced computation, we pre-build a lookup tables for the heavy lifting computation
  if (!initialized) {
    initialized = true;
    for (int a = 0; a < 360; a++)
    {
      x2[a] = (dm * .9 * cos((a - 90) * PIover180)) + X;
      y2[a] = (dm * .9 * sin((a - 90) * PIover180)) + Y;
      dist[a] = sqrt((X - x2[a]) * (X - x2[a]) + (Y - y2[a]) * (Y - y2[a]));
      dx[a] = X + (w / 6) * (x2[a] - X) / h;
      dy[a] = Y + (w / 6) * (y2[a] - Y) / h;
    }
  }

  x2a = X - dx[angle];
  y2a = dy[angle] - Y;
  x3 = y2a + dx[angle];
  y3 = x2a + dy[angle];
  x4 = dx[angle] - y2a;
  y4 = dy[angle] - x2a;

  if (toDo) {
    if (headStyle) {
      utftDisplay.setColor(colorDir);
      geo.fillTriangle(x2[angle], y2[angle], x3, y3, x4, y4);
      geo.drawTriangle(x2[angle], y2[angle], x3, y3, x4, y4);
      geo.fillTriangle(x3, y3, X, Y, x4, y4);
      geo.drawTriangle(x3, y3, X, Y, x4, y4);
    } else {
      utftDisplay.setColor(black);
      geo.fillTriangle(x2[angle], y2[angle], x3, y3, x4, y4);
      geo.fillTriangle(x3, y3, X, Y, x4, y4);
      utftDisplay.setColor(colorSet);
      utftDisplay.drawLine(x3, y3, X, Y);
      utftDisplay.drawLine(X, Y, x4, y4);
      utftDisplay.drawLine(x4, y4, x2[angle], y2[angle]);
      utftDisplay.drawLine(x2[angle], y2[angle], x3, y3);
      utftDisplay.drawLine(X, Y, x2[angle], y2[angle]);
    }
  } else {
    utftDisplay.setColor(black);
    geo.fillTriangle(x2[angle], y2[angle], x3, y3, x4, y4);
    geo.drawTriangle(x2[angle], y2[angle], x3, y3, x4, y4);
    geo.fillTriangle(x3, y3, X, Y, x4, y4);
    geo.drawTriangle(x3, y3, X, Y, x4, y4);
  }
  utftDisplay.setColor(colorDir);
  utftDisplay.fillCircle(X, Y, 9);
}

void UserPrintAngle (int x, int y, int userAngle, Colors COLOR) {
  char angle[4];                      // 3 digit + null string terminator (\0)
  sprintf(angle, "%03d", userAngle);
  utftDisplay.setColor(COLOR);
  utftDisplay.setFont(SevenSegmentFull);
  utftDisplay.print(angle, x, y);
}

int Read12bit(uint8_t pin) {
  int Result = 0;
  analogRead(pin);                    // Switch ADC
  for (int i = 0; i < 16; i++) {      // Read 16 times
    Result += analogRead(pin);        // Sum results
  }
  Result >>= 2;                       // Divide by 4 for 12 bit value
  return Result;
}

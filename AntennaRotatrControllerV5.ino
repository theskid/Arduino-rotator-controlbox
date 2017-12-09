#define FASTADC 1
//=======================================================//
#ifndef cbi                                              //
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))      //
#endif                                                   // defines for setting and 
#ifndef sbi                                              // clearing register bits
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))       //
#endif                                                   //
//=======================================================//
#define FormatData(x) strcpy_P(dataBuffer, PSTR(x))

#include <UTFT.h>           // UTFT Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)
#include <UTFT_Geometry.h>  // UTFT Geometry Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)

/*#################### USER Configuration ####################/
  /#                                                           #/
  /# type of display used                                      #/
  /# DT value rappreset the display that you are using         #/
  /# DT == 1  : 3.2 480x320 TFTLCD Shild                       #/
  /#                                                           #/
  /############################################################*/

const int DT = 1;             // Display used
const String QRZ = "IU6CRH";  // Your QRZ
const String NAME = "Diego";  // Your Name
const int DEBUG = 0;          // Debug flag true == print debug value on serial output
const bool SETUP = true;
const int minAzimut = 0;
const int maxAzimut = 359;
const int rotatorStart = 308;
const int rotatorStop = 611;
const int zumEncoder = 4;

// ################# END USER Configuration #################*/

// ################# Digital PIN Designation ################//
int StartStopSwitch = 12;
int UserActionSwitch = 8;
int SpeedControlSwitch = 13;
int ManualSpeedCtrl = 9;
int CWMotor = 10;
int CCWMotor = 11;

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

inline void InitializeDisplay(int displayType); 						// inizializzazione del display generica
inline void InitializeDisplayHVGA480x320();     						// inizializzazione specifica per il display da 3.2 pollici 480x320 HVGA TFT LCD
inline void ConfigureIOPins();                  						// Inizializazione dei pins
inline void DrawInitialScreen();                						// disegno preliminare dello schermo
inline void UserPrint (int x, int y, String userData, Colors COLOR);    // scrittura delle stringhe
inline void DrawBeamHead(int angle, HeadType headStyle, Action toDo);   // disegno delle lancette
inline void UserPrintAngle (int x, int y, int userAngle, Colors COLOR); // print degli angoli con font SevenSegmentFull
inline boolean isPushed(int botton);            						// rilevazione della pressione dei pulsanti
inline void StartStopChangeStatus();            						// cambia la flag alla pressione del pulsante start/stop
inline void StartStopAction();                  						// effettua le azioni da compiere a seconda del flag start/stop
inline void AutoManualChangeStatus();           						// cambia la flag alla pressione del pulsante auto/manual
inline void AutoManualAction();                 						// effettua le azioni da compiere a seconda del flag auto/manual
inline void UserSetConfirmChangeStatus();       						// cambia la flag alla pressione del pulsante user set/confirm
inline void BeamSetting();                      						// effettua la lettura del potenziometro per il settaggio dell'azimut
inline void BeamDirControl();                               // effettua la lettura del potenziometro sotto il rotore dell'azimut
//inline int Read12bit(uint8_t pin);                          // 12-bits oversampled analogread 
inline int Read12bit();                          // 12-bits oversampled analogread 

UTFT utftDisplay(ILI9481, 38, 39, 40, 41);
UTFT_Geometry geo(&utftDisplay);

int beamDir = 1;    // Actual beam direction
int beamSet = 1;    // Beam directione to set
int spdValue = 1;   // Rotation speed

PINflag StartStopFlag = Stop;
PINflag SpeedModeFlag = Manual;
PINflag UserActionFlag = Setting;


void setup() {
  Serial.begin(9600);
#if FASTADC
  // set prescale to 16 (1MHz)
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
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
  if (SETUP) {
   Serial.print(analogRead(A2));
   Serial.print("\n");
  }
  if (DEBUG > 0) {
    Serial.print("--------------------------------Ciclyng loop() START ----------------------------------\n");
    Serial.print("Value of the start/stop flag == ");
    Serial.print(StartStopFlag);
    Serial.print("\n");
    Serial.print("Value of the Auto/Manual flag == ");
    Serial.print(SpeedModeFlag);
    Serial.print("\n");
    Serial.print("Value of the User Action flag == ");
    Serial.print(UserActionFlag);
    Serial.print("\n");
    Serial.print("Value of the BEAM direction == ");
    Serial.print(beamDir);
    Serial.print("\n");
    Serial.print("Value of the BEAM setting == ");
    Serial.print(beamSet);
    Serial.print("\n");
    Serial.print("Value of the throttle setting == ");
    Serial.print(spdValue);
    Serial.print("\n");
  }
  if (isPushed(StartStopSwitch)) {
    if (DEBUG > 0) {
      Serial.print("Satus of the start/stop flag == ");
      Serial.print(StartStopFlag);
      Serial.print("\n");
      Serial.print("StartStopSwitch has been pushed");
      Serial.print("\n");
    }
    StartStopChangeStatus();
    if (DEBUG > 0) {
      Serial.print("New satus of the start/stop flag == ");
      Serial.print(StartStopFlag);
      Serial.print("\n");
    }
  }
  if (DEBUG > 0) {
    Serial.print("Entering StartStopAction()");
    Serial.print("\n");
  }
  StartStopAction();
  if (DEBUG > 0) {
    Serial.print("Exiting StartStopAction()");
    Serial.print("\n");
  }
  if (isPushed(SpeedControlSwitch)) {
    if (DEBUG > 0) {
      Serial.print("Satus of the Auto/Manual flag == ");
      Serial.print(SpeedModeFlag);
      Serial.print("\n");
      Serial.print("SpeedControlSwitch has been pushed");
      Serial.print("\n");
    }
    AutoManualChangeStatus();
    if (DEBUG > 0) {
      Serial.print("New satus of the Auto/Manual flag == ");
      Serial.print(SpeedModeFlag);
      Serial.print("\n");
    }
  }
  if (DEBUG > 0) {
    Serial.print("Entering AutoManualAction()");
    Serial.print("\n");
  }  
  AutoManualAction();
  if (DEBUG > 0) {
    Serial.print("Exiting AutoManualAction()");
    Serial.print("\n");
  }
  if (isPushed(UserActionSwitch)) {
	  if (DEBUG > 0) {
		  Serial.print("UserActionSwitch has been pushed");
		  Serial.print("\n");
		  Serial.print("Satus of UserActionFlag == ");
		  Serial.print(UserActionFlag);
		  Serial.print("\n");      
	  }
    UserSetConfirmChangeStatus();
	  if (DEBUG > 0) {
		Serial.print("New status of UserActionFlag == ");
		Serial.print(UserActionFlag);
		Serial.print("\n");      
	  }
  }
  BeamSetting();
  BeamDirControl();
  if (DEBUG > 0) {
    Serial.print("--------------------------------Ciclyng loop() END ----------------------------------\n");
    delay(1000);
  }
}

void BeamDirControl() {
  float rawAngle;
  if (beamSet != beamDir) {
    DrawBeamHead(beamDir, BeamDIR, Delete);                                   //   cancella la llancetta di direzione al vecchio azimut  
    rawAngle = Read12bit();                                                //   leggi il potenziometro sotto al rotore
    beamDir = map(rawAngle, rotatorStart, rotatorStop, minAzimut, maxAzimut); //   normalizza la letura del potenziometro rawAngle
    DrawBeamHead(beamDir, BeamDIR, Create);     //   Disegna la lancetta all'azimut all'attuale posizione
    UserPrintAngle(0, 113, beamDir, yellow);    //   Scrivi l'azimut corrispondente in giallo
  } else {                                      // Altrimenti
    UserPrintAngle(0, 113, beamDir, green);     //   conferma il l'arrivo
  }
}

void StartStopChangeStatus() {
  if (StartStopFlag == Stop) {
    StartStopFlag = Start;
  } else if (StartStopFlag == Start) {
    StartStopFlag = Stop;
  }
}

void AutoManualChangeStatus() {
  if (SpeedModeFlag == Manual ) {
    SpeedModeFlag = Auto;
  } else if (SpeedModeFlag == Auto ) {
    SpeedModeFlag = Manual;
  }
}

void UserSetConfirmChangeStatus() {
  if (UserActionFlag == Setting ) {
    UserActionFlag = Confirmed;
  } else if (UserActionFlag == Confirmed ) {
    UserActionFlag = Setting;
  }
}

void BeamSetting() {
  int rawAngle;
  if (UserActionFlag == Setting) {                          // Controlla se e' permesso il settaggio
    DrawBeamHead(beamSet, BeamSET, Delete);                 //   cancella la llancetta di settaggio al vecchio azimut
    rawAngle = analogRead(A1);                              //   leggi il potenziometro
    beamSet = map(rawAngle, 0, 1023, minAzimut, maxAzimut); //   normalizza la letura del potenziometrorawAngle
    DrawBeamHead(beamSet, BeamSET, Create);                 //   Disegna la lancetta all'azimut corrispondente
    UserPrintAngle(0, 213, beamSet, yellow);                //   Scrivi l'azimut corrispondente in giallo
  } else {                                                  // Altrimenti
    UserPrintAngle(0, 213, beamSet, green);                 //   conferma il settaggio
  }
}

void StartStopAction() {
  if (!StartStopFlag) {
    digitalWrite(CWMotor, LOW);
    digitalWrite(CCWMotor, LOW);
    StartStopFlag = 0;
    utftDisplay.setColor(yellow);
    utftDisplay.setFont(BigFont);
    utftDisplay.print("    ", RIGHT, 25);
  } else {
    if ((beamDir < beamSet) && (StartStopFlag)) {
      digitalWrite(CWMotor, HIGH);
      digitalWrite(CCWMotor, LOW);      
      utftDisplay.setColor(yellow);
      utftDisplay.setFont(BigFont);
      utftDisplay.print(" CW ", RIGHT, 25);
    }
    if ((beamDir > beamSet) && (StartStopFlag)) {
      digitalWrite(CWMotor, LOW);
      digitalWrite(CCWMotor, HIGH);
      utftDisplay.setColor(yellow);
      utftDisplay.setFont(BigFont);
      utftDisplay.print("CCW ", RIGHT, 25);
    }
    if (beamDir == beamSet) {
      digitalWrite(CWMotor, LOW);
      digitalWrite(CCWMotor, LOW);
      StartStopFlag = Stop;
      utftDisplay.setColor(yellow);
      utftDisplay.setFont(BigFont);
      utftDisplay.print("    ", RIGHT, 25);
    }
  }
}

void AutoManualAction() {
  int rawSpdValue;
  if (!SpeedModeFlag) {
    if (StartStopFlag == Stop) {
      rawSpdValue = analogRead(A0);
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

boolean isPushed(int botton) {
  int timeDelay = 100;
  if (digitalRead(botton)) {
    delay(timeDelay);
    return false;
  }
  delay(timeDelay);
  return true;
}

void InitializeDisplay(int displayNumber) {
  if (displayNumber == 1) {
    InitializeDisplayHVGA480x320();
  }
}

void InitializeDisplayHVGA480x320() {
  X = 320;
  Y = 160;
  dm = 130;
  utftDisplay.InitLCD();
  utftDisplay.InitLCD(LANDSCAPE);
  utftDisplay.clrScr();
  utftDisplay.setFont(BigFont);
  UserPrint(0, 12, "ANTENNA ROTATOR", orange);
  UserPrint(40, 36, "CONTROLLER", orange);
  utftDisplay.drawLine(440, 160, 460, 160);
  utftDisplay.drawLine(180, 160, 200, 160);
  utftDisplay.drawLine(320, 20, 320, 40);
  utftDisplay.drawLine(320, 280, 320, 300);
  utftDisplay.drawCircle(320, 160, 130);
  UserPrint(0, 75, "BEAM DIR", red);
  UserPrint(0, 175, "BEAM SET", red);
  UserPrint(0, 290, QRZ + " : " + NAME, white);
  UserPrint((X - 8), (Y - 157), "N", red);
  UserPrint((X - 8), (Y + 145), "S", red);
  UserPrint((X + 141), (Y - 7), "E", red);
  UserPrint((X - 160), (Y - 7), "W", red);
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

void UserPrint (int x, int y, String userData, Colors COLOR) {
  utftDisplay.setColor(COLOR);
  utftDisplay.setFont(BigFont);
  utftDisplay.print(userData, x, y);
}

void DrawInitialScreen() {
  int dxOuter, dyOuter, dxinner, dyinner;
  utftDisplay.setColor(0, 255, 0);
  utftDisplay.drawCircle(X, Y, dm);
  for (float i = 0; i < 360; i = i + 22.5 )
  {
    utftDisplay.setColor(255, 128, 0);
    dxOuter = dm * cos((i - 90) * 3.14 / 180);
    dyOuter = dm * sin((i - 90) * 3.14 / 180);
    dxinner = dxOuter * 0.97;
    dyinner = dyOuter * 0.97;
    utftDisplay.drawLine(dxOuter + X, dyOuter + Y, dxinner + X, dyinner + Y);
  }
  for (float i = 0; i < 360; i = i + 45 )
  {
    utftDisplay.setColor(255, 128, 0);
    dxOuter = dm * cos((i - 90) * 3.14 / 180);
    dyOuter = dm * sin((i - 90) * 3.14 / 180);
    dxinner = dxOuter * 0.92;
    dyinner = dyOuter * 0.92;
    utftDisplay.drawLine(dxinner + X, dyinner + Y, dxOuter + X, dyOuter + Y);
  }
}

void DrawBeamHead(int angle, HeadType headStyle, Action toDo) {
  Colors colorDir = red;
  Colors colorSet = green;
  float dist;
  int dx, dy, x2a, y2a, x2, y2, x3, y3, x4, y4, h, w;
  if (headStyle) {
    h = 12;
    w = 10;
  } else {
    h = 10;
    w = 10;
  }
  x2 = (dm * .9 * cos((angle - 90) * 3.14 / 180)) + X; // calculate X position
  y2 = (dm * .9 * sin((angle - 90) * 3.14 / 180)) + Y; // calculate Y position
  dist = sqrt((X - x2) * (X - x2) + (Y - y2) * (Y - y2));
  dx = X + (w / 6) * (x2 - X) / h;
  dy = Y + (w / 6) * (y2 - Y) / h;
  x2a = X - dx;
  y2a = dy - Y;
  x3 = y2a + dx;
  y3 = x2a + dy;
  x4 = dx - y2a;
  y4 = dy - x2a;
  if (toDo) {
    if (headStyle) {
      utftDisplay.setColor(colorDir);
      geo.fillTriangle(x2, y2, x3, y3, x4, y4);
      geo.drawTriangle(x2, y2, x3, y3, x4, y4);
      geo.fillTriangle(x3, y3, X, Y, x4, y4);
      geo.drawTriangle(x3, y3, X, Y, x4, y4);
    } else {
      utftDisplay.setColor(black);
      geo.fillTriangle(x2, y2, x3, y3, x4, y4);
      geo.fillTriangle(x3, y3, X, Y, x4, y4);
      utftDisplay.setColor(colorSet);
      utftDisplay.drawLine(x3, y3, X, Y);
      utftDisplay.drawLine(X, Y, x4, y4);
      utftDisplay.drawLine(x4, y4, x2, y2);
      utftDisplay.drawLine(x2, y2, x3, y3);
      utftDisplay.drawLine(X, Y, x2, y2);
    }
  } else {
    utftDisplay.setColor(black);
    geo.fillTriangle(x2, y2, x3, y3, x4, y4);
    geo.drawTriangle(x2, y2, x3, y3, x4, y4);
    geo.fillTriangle(x3, y3, X, Y, x4, y4);
    geo.drawTriangle(x3, y3, X, Y, x4, y4);
  }
  utftDisplay.setColor(colorDir);
  utftDisplay.fillCircle(320, 160, 9);
}

void UserPrintAngle (int x, int y, int userAngle, Colors COLOR) {
  char dataBuffer[60];
  char formattedDataBuffer[3];
  sprintf(formattedDataBuffer, FormatData("%03d"), userAngle);
  utftDisplay.setColor(COLOR);
  utftDisplay.setFont(SevenSegmentFull);
  utftDisplay.print(formattedDataBuffer, x, y);
}

int Read12bit(){
  int Result = 0;
  analogRead(A2);                      //Switch ADC
  for (int i = 0; i < 16; i++){         //Read 16 times
    Result = Result + analogRead(A2);  //Sum results
  }
  Result = (int)Result / 4;             //Divide by 4 for 12 bit value
  return (Result);
}


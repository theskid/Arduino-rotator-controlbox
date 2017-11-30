#include <UTFT.h>           // UTFT Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)
#include <UTFT_Geometry.h>  // UTFT Geometry Library from Henning Karlsen (http://www.rinkydinkelectronics.com/library.php)
#include <EEPROM.h>

#define address 0x1E                    // 0011110b, I2C 7bit address of HMC5883
#define EEPROM_ModeStatus_Location 10   // The starting address of the EEPROM where the data is Stored (10.11.12.13)
#define EEPROMSetpointLocation 14
#define EEPROM_ScaleMax_Location 18
#define EEPROMInflightLocation 22
#define EEPROMMagDeclinationDegreeLocation 26
#define EEPROMMagDeclinationMinuteLocation 30
#define EEPROMMagDeclinationSignLocation 34
#define FormatData(x) strcpy_P(dataBuffer, PSTR(x))

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
//const bool DEBUG = false;         // Debug flag true == print debug value on serial output
const bool DEBUG = true;         // Debug flag true == print debug value on serial output
const int minAzimut = 0;
const int maxAzimut = 359;

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

inline void InitializeDisplay(int displayType); // inizializzazione del display generica
inline void InitializeDisplayHVGA480x320();     // inizializzazione specifica per il display da 3.2 pollici 480x320 HVGA TFT LCD
inline void ConfigureIOPins();                  // Inizializazione dei pins
inline void DrawInitialScreen();                // disegno preliminare dello schermo
inline void UserPrint (int x, int y, String userData, Colors COLOR);    // scrittura delle stringhe
inline void DrawBeamHead(int angle, HeadType headStyle, Action toDo);   // disegno delle lancette
inline void UserPrintAngle (int x, int y, int userAngle, Colors COLOR); // print degli angoli con font SevenSegmentFull
inline boolean isPushed(int botton);            // rilevazione della pressione dei pulsanti
inline void StartStopChangeStatus();            // cambia la flag alla pressione del pulsante start/stop
inline void StartStopAction();                  // effettua le azioni da compiere a seconda del flag start/stop
inline void AutoManualChangeStatus();           // cambia la flag alla pressione del pulsante auto/manual
inline void AutoManualAction();                 // effettua le azioni da compiere a seconda del flag auto/manual
inline void UserSetConfirmChangeStatus();       // cambia la flag alla pressione del pulsante user set/confirm
inline void BeamSetting();                      // effettua la lettura del potenziometro per il settaggio dell'azimut
inline void SerialDebug (String output);

UTFT utftDisplay(ILI9481, 38, 39, 40, 41);
UTFT_Geometry geo(&utftDisplay);

int beamDir = 0;    // Actual beam direction
int beamSet = 0;    // Beam directione to set
int spdValue = 0;   // Rotation speed

PINflag StartStopFlag = Stop;
PINflag SpeedModeFlag = Auto;
PINflag UserActionFlag = Confirmed;


void setup() {
  Serial.begin(9600);
  InitializeDisplay(DT);
  DrawInitialScreen();
  ConfigureIOPins();
  //DrawBeamHead(beamSet,BeamSET,Create);
  //DrawBeamHead(beamDir,BeamDIR,Create);
  //UserPrintAngle(0,213,beamSet,green);
  //UserPrintAngle(0,113,beamDir,green);
}

void loop() {
  if (DEBUG) {
    Serial.print("--------------------------------Ciclyng loop() START ----------------------------------\n");
    Serial.print("Value of the start/stop flag == ");
    Serial.print(StartStopFlag);
    Serial.print("\n");
    Serial.print("Value of the Auto/Manual flag == ");
    Serial.print(SpeedModeFlag);
    Serial.print("\n");
  }
  if (isPushed(StartStopSwitch)) {
    if (DEBUG) {
      Serial.print("Satus of the start/stop flag == ");
      Serial.print(StartStopFlag);
      Serial.print("\n");
      Serial.print("StartStopSwitch has been pushed");
      Serial.print("\n");
    }
    StartStopChangeStatus();
    if (DEBUG) {
      Serial.print("New satus of the start/stop flag == ");
      Serial.print(StartStopFlag);
      Serial.print("\n");
    }
  }
  StartStopAction();
  if (isPushed(SpeedControlSwitch)) {
    if (DEBUG) {
      Serial.print("Satus of the Auto/Manual flag == ");
      Serial.print(SpeedModeFlag);
      Serial.print("\n");
      Serial.print("SpeedControlSwitch has been pushed");
      Serial.print("\n");
    }
    AutoManualChangeStatus();
    if (DEBUG) {
      Serial.print("New satus of the Auto/Manual flag == ");
      Serial.print(SpeedModeFlag);
      Serial.print("\n");
    }
  }
  AutoManualAction();
  //  if (isPushed(UserActionSwitch)) {
  //    UserSetConfirmChangeStatus();
  //    SerialDebug ("UserActionSwitch has been pushed");
  //  }
  //  BeamSetting();
  // BeaDirControl();
  if (DEBUG) {
    Serial.print("--------------------------------Ciclyng loop() END ----------------------------------\n");
    delay(1000);
  }
}

void SerialDebug (String output) {
  if (DEBUG) {
    Serial.print(output);
    Serial.print("\n");
  }
}

void StartStopChangeStatus() {
  if (DEBUG) {
    Serial.print("Ingresso alla StartStopChangeStatus()\n");
    Serial.print("Valore d'ingresso della Start/Stop Falg == ");
    Serial.print(StartStopFlag);
    Serial.print("\n");
  }
  if (StartStopFlag == Stop) {
    StartStopFlag = Start;
    if (DEBUG) {
      Serial.print("Nuovo valore della Start/Stop Falg == ");
      Serial.print(StartStopFlag);
      Serial.print("\n");
    }
  } else if (StartStopFlag == Start) {
    StartStopFlag = Stop;
    if (DEBUG) {
      Serial.print("Nuovo valore della Start/Stop Falg == ");
      Serial.print(StartStopFlag);
      Serial.print("\n");
    }
  }
  if (DEBUG) {
    Serial.print("Uscita dalla StartStopChangeStatus()\n");
    Serial.print("Valore d'uscita della Start/Stop Falg == ");
    Serial.print(StartStopFlag);
    Serial.print("\n");
  }
}

void AutoManualChangeStatus() {
  if (DEBUG) {
    Serial.print("Ingresso alla AutoManualChangeStatus()\n");
    Serial.print("Valore d'ingresso della Auto/Manual Falg == ");
    Serial.print(SpeedModeFlag);
    Serial.print("\n");
  }
  if (SpeedModeFlag == Manual ) {
    SpeedModeFlag = Auto;
    if (DEBUG) {
      Serial.print("Nuovo valore della  Auto/Manual Falg == ");
      Serial.print(SpeedModeFlag);
      Serial.print("\n");
    }
  } else if (SpeedModeFlag == Auto ) {
    SpeedModeFlag = Manual;
    if (DEBUG) {
      Serial.print("Nuovo valore della Auto/Manual Falg == ");
      Serial.print(SpeedModeFlag);
      Serial.print("\n");
    }
  }
  if (DEBUG) {
    Serial.print("Uscita dalla AutoManualChangeStatus()\n");
    Serial.print("Valore d'uscita della Auto/Manual Falg == ");
    Serial.print(SpeedModeFlag);
    Serial.print("\n");
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
  if (UserActionFlag == Setting) {              // Controlla se e' permesso il settaggio
    DrawBeamHead(beamSet, BeamSET, Delete);     //   cancella la llancetta di settaggio al vecchio azimut
    int rawAngle = analogRead(A1);              //   leggi il potenziometro
    beamSet = map(rawAngle, 0, 1023, 0 , 360);  //   normalizza la letura del potenziometro
    DrawBeamHead(beamSet, BeamSET, Create);     //   Disegna la lancetta all'azimut corrispondente
    UserPrintAngle(0, 213, beamSet, yellow);    //   Scrivi l'azimut corrispondente in giallo
  } else {                                      // Altrimenti
    UserPrintAngle(0, 213, beamSet, green);     //   conferma il settaggio
  }
}

void StartStopAction() {
  if (DEBUG) {
    Serial.print("Ingresso alla StartStopAction()########################\n");
    Serial.print("Valore in ingresso della Start/Stop Falg == ");
    Serial.print(StartStopFlag);
    Serial.print("\n");
  }
  if (!StartStopFlag) {
    digitalWrite(CWMotor, LOW);
    digitalWrite(CCWMotor, LOW);
    StartStopFlag = Stop;
    UserPrint(455, 25, "      ", black);
    if (DEBUG) {
      Serial.print("Finding rotation stopped\n");
      Serial.print("Start/Stop Flag == ");
      Serial.print(StartStopFlag);
      Serial.print("\n");
      Serial.print("CWMotor value == LOW\n");
      Serial.print("CCWMotor value == LOW\n");
    }
  } else {
    if ((beamDir < beamSet) && (StartStopFlag)) {
      digitalWrite(CWMotor, HIGH);
      digitalWrite(CCWMotor, LOW);
      UserPrint(455, 25, "  CW ", yellow);
      if (DEBUG) {
        Serial.print("Finding rotation CW\n");
        Serial.print("Start/Stop Flag == ");
        Serial.print(StartStopFlag);
        Serial.print("\n");
        Serial.print("CWMotor value == HIGH\n");
        Serial.print("CCWMotor value == LOW\n");
      }
    }
    if ((beamDir > beamSet) && (StartStopFlag)) {
      digitalWrite(CWMotor, LOW);
      digitalWrite(CCWMotor, HIGH);
      UserPrint(455, 25, " CCW ", yellow);
      if (DEBUG) {
        Serial.print("Finding rotation CCW\n");
        Serial.print("Start/Stop Flag == ");
        Serial.print(StartStopFlag);
        Serial.print("\n");
        Serial.print("CWMotor value == LOW\n");
        Serial.print("CCWMotor value == HIGH\n");
      }
    }
    if (beamDir == beamSet) {
      digitalWrite(CWMotor, LOW);
      digitalWrite(CCWMotor, LOW);
      StartStopFlag = Stop;
      UserPrint(455, 25, "      ", black);
      if (DEBUG) {
        Serial.print("Beam has reached set point\n");
        Serial.print("Start/Stop Flag == ");
        Serial.print(StartStopFlag);
        Serial.print("\n");
        Serial.print("CWMotor value == LOW\n");
        Serial.print("CCWMotor value == LOW\n");
      }
    }
  }
  if (DEBUG) {
    Serial.print("Uscita dalla StartStopAction()\n");
  }
}

void AutoManualAction() {
  if (DEBUG) {
    Serial.print("Ingresso alla AutoManualAction()########################\n");
    Serial.print("Valore in ingresso della Auto/Manual Falg == ");
    Serial.print(SpeedModeFlag);
    Serial.print("\n");
  }
  if (SpeedModeFlag == Manual) {
    if (DEBUG) {
      Serial.print("With Auto/Manual Falg == Manual \n");
    }
    if (StartStopFlag == Stop) {
      if (DEBUG) {
        Serial.print("Rotation is stopped\n");
      }
      spdValue = analogRead(A0);
      if (DEBUG) {
        Serial.print("RAW value letta dal potenziometro\n == ");
        Serial.print(spdValue);
        Serial.print("\n");
      }
      spdValue = map(spdValue, 0, 1023, minAzimut, maxAzimut);
      if (DEBUG) {
        Serial.print("Value normalizzata nel range nim max azimut == ");
        Serial.print(spdValue);
        Serial.print("\n");
      }
      analogWrite(ManualSpeedCtrl, spdValue);
    } else {
      spdValue = 0;
                 analogWrite(ManualSpeedCtrl, spdValue);
    }
    if (DEBUG) {
      Serial.print("Pin ");
      Serial.print(ManualSpeedCtrl);
      Serial.print("Impostato a ");
      Serial.print(spdValue);
      Serial.print("\n");
    }
    UserPrint(400, 12, "Manual", yellow);
  } else {
    if (DEBUG) {
      Serial.print("With Auto/Manual Falg == Auto \n");
    }
    if (StartStopFlag == Start) {
      if (DEBUG) {
        Serial.print("Rotation in progress\n");
      }
      /* Irrespective of the Direction the difference in value needs to
         be considered for PWM, Stoping is based on Cw/CCW outputs.
         Use Serial Print to check the value of rotationValue variable */
      int rotationValue = abs(beamSet - beamDir);
      int scaleRotationValue =  rotationValue * 4;
      //int scaleMaxValue = scaleMax *4;
      int scaleMaxValue = 1023;
      spdValue = map(scaleRotationValue, 0, scaleMaxValue, minAzimut, maxAzimut); //The Scaling needs to be fine tuned based on the Test.
      analogWrite(ManualSpeedCtrl, spdValue);
      if (DEBUG) {
      Serial.print("RAW value letta in base alla rotazione == ");
      Serial.print(rotationValue);
      Serial.print("\n");
      Serial.print("Value normalizzata nel range nim max azimut == ");
      Serial.print(spdValue);
      Serial.print("\n");
    }
    } else {
      spdValue = 0;
      analogWrite(ManualSpeedCtrl, spdValue);
    }
    
    UserPrint(400, 12, "Auto  ", yellow);
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


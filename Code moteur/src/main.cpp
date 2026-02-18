// GRO400 - Exemple d'utilisation du OpenRB avec un moteur Dynamixel sous Platform.IO.
// Basé sur l'exemple de Position Control.
// Opère un moteur (à définir par la variable DXL_ID - 1 par défaut) en position en le faisant passer
// d'une position en pulsations (1000) à une autre en degrés (5.7) et vice-versa à chaque
// seconde.
// Écrit la position en cours en pulsations à la console série (accessible par DEBUG_SERIAL).
// N'oubliez-pas de configurer votre port série pour cette console à 115200 bauds.

#include <Dynamixel2Arduino.h>
#include <Servo.h>
#define ouvert 50
#define fermer 0

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

Servo myServo;
const int servoPin = 5;
 
// TODO: À changer selon l'ID de votre moteur :
const uint8_t DXL_ID_DH1007 = 1; 
const uint8_t DXL_ID_DH2028 = 2; 
const uint8_t DXL_ID_DH2020 = 3;
float deg1 = 0;
float deg2 = 0;
float deg3 = 0;


const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void Set_target_angle(){

  float position1 = dxl.getPresentPosition(DXL_ID_DH2028, UNIT_DEGREE);
  float position2 = dxl.getPresentPosition(DXL_ID_DH1007, UNIT_DEGREE);
  float position3 = dxl.getPresentPosition(DXL_ID_DH2020, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_DH2028, deg1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_DH1007, deg2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_DH2020, deg3, UNIT_DEGREE);

  while (abs(deg1 - position1) > 1.0 && abs(deg2 - position2) > 1.0 && abs(deg3 - position3)> 1.0)
  {
    position1 = dxl.getPresentPosition(DXL_ID_DH2028, UNIT_DEGREE);
    position2 = dxl.getPresentPosition(DXL_ID_DH1007, UNIT_DEGREE);
    position3 = dxl.getPresentPosition(DXL_ID_DH2020, UNIT_DEGREE);
  }
  //appel de gripper voir comment il faut se rendre
  delay(5000);
}
void gripper(){
  //ferme la pince
  myServo.write(fermer);
  delay(1000);
  //ouvre la pince
  myServo.write(ouvert);
  delay(1000);
}
void lecture(){

  String message = DEBUG_SERIAL.readStringUntil('\n');
  float a1, a2, a3;
  a1 = DEBUG_SERIAL.parseFloat();
  a2 = DEBUG_SERIAL.parseFloat();
  a3 = DEBUG_SERIAL.parseFloat();

  // Extraction des 3 valeurs
  // Conversion Radian -> Degré (si Python envoie des Radians)
  deg1 = ((a1 * 180.0) / PI);
  deg2 = ((a2 * 180.0) / PI);
  deg3 = ((a3 * 180.0) / PI);

  //rend la valeur négative en valeur positive
  if (deg1<0)
  {
    deg1 = 360-deg1;
  }
  if (deg2<0)
  {
    deg2 = 360-deg2;
  }
  if (deg3<0)
  {
    deg3 = 360-deg3;
  }
  DEBUG_SERIAL.println("lecture fait");
}

void setup() {
  // put your setup code here, to run once:
  delay(2000);    // Délai additionnel pour avoir le temps de lire les messages sur la console.
  DEBUG_SERIAL.println("Starting position control ...");
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL); // On attend que la communication série pour les messages soit prête.

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);

  if (dxl.getLastLibErrCode()) {
    DEBUG_SERIAL.println("Could not init serial port!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  if (!dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION)) {
    DEBUG_SERIAL.println("Could not set protocol version!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  // Get DYNAMIXEL information
  bool ping_M1 = dxl.ping(DXL_ID_DH1007);
  bool ping_M2 = dxl.ping(DXL_ID_DH2028);
  bool ping_M3 = dxl.ping(DXL_ID_DH2020);

  if (!ping_M1) {
    DEBUG_SERIAL.println("Could not ping motor1!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());

    return;
  }
  if (!ping_M2) {
    DEBUG_SERIAL.println("Could not ping motor2!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());

    return;
  }
  if (!ping_M3) {
    DEBUG_SERIAL.println("Could not ping motor3!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());

    return;
  }

  //Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_DH2020);
  dxl.setOperatingMode(DXL_ID_DH2020, OP_POSITION);
  dxl.torqueOn(DXL_ID_DH2020);

  dxl.torqueOff(DXL_ID_DH2028);
  dxl.setOperatingMode(DXL_ID_DH2028, OP_POSITION);
  dxl.torqueOn(DXL_ID_DH2028);

  dxl.torqueOff(DXL_ID_DH1007);
  dxl.setOperatingMode(DXL_ID_DH1007, OP_POSITION);
  dxl.torqueOn(DXL_ID_DH1007);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_DH2020, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_DH2028, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_DH1007, 30);

  DEBUG_SERIAL.println("Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  dxl.setGoalPosition(DXL_ID_DH2020, 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_DH2028, 0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_DH1007, 216, UNIT_DEGREE);
  myServo.attach(servoPin);
}

void loop() {
  // Vérifie si Python a envoyé des données sur le port USB
  if (DEBUG_SERIAL.available() > 0) {
    lecture();

  }
  //test
  delay(2000);
  // Envoi immédiat aux moteurs
  Serial.println(dxl.getPresentPosition(DXL_ID_DH1007,UNIT_DEGREE));// max 216 degree min 19 degree
  delay(2000);
  dxl.setGoalPosition(DXL_ID_DH1007,19,UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_DH2028, 100, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID_DH2020, 0, UNIT_DEGREE);
}

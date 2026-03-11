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
const uint8_t Joint_1 = 2; 
const uint8_t Joint_2 = 1;
const uint8_t Joint_3 = 3;

float deg1 = 0;
float deg2 = 0;
float deg3 = 0;
float v1 = 30;
float v2 = 30;
float v3 = 30;
bool home = 0;
bool lineaire = 0;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;
void get_new_velo(){
  String msg = DEBUG_SERIAL.readStringUntil('\n');

  if(msg.startsWith("#Lineaire,")){
    //#Lineaire,deg1~V1 deg2_V2-deg3
    int commaIndex = msg.indexOf(',');
    int spaceIndex = msg.indexOf(' ');
    int tiretIndex = msg.indexOf('-');
    int starIndex = msg.indexOf('*');
    
    if (commaIndex != -1 && starIndex != -1)
    {
      String value_V1 = msg.substring(commaIndex + 1, spaceIndex);
      String value_V2 = msg.substring(spaceIndex + 1, tiretIndex);
      String value_V3 = msg.substring(tiretIndex + 1, starIndex);

      v1 = value_V1.toFloat()/1.374;
      v2 = value_V2.toFloat()/1.374;
      v3 = value_V3.toFloat()/1.374;
    }
  }
}
void Set_target_angle(){

  float position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
  float position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
  float position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_1, deg1, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_2, deg2, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_3, deg3, UNIT_DEGREE);

  while (abs(deg1 - position1) <= 1.0 && abs(deg2 - position2) <= 1.0 && abs(deg3 - position3) <= 1.0)
  {
    position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
    position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
    position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
    if(lineaire == true){
      get_new_velo();
    }
    dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, v1);
    dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, v2);
    dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, v3);
  }
  //appel de gripper voir comment il faut se rendre
  //delay(5000);
}
void set_target_speed(){

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
  float a1, a2, a3;
  String msg = DEBUG_SERIAL.readStringUntil('\n');
  if(msg.startsWith("#Joint,")){
    lineaire = false;
    int commaIndex = msg.indexOf(',');
    int zigIndex = msg.indexOf('~');
    int spaceIndex = msg.indexOf(' ');
    int starIndex = msg.indexOf('*');

    if (commaIndex != -1 && starIndex != -1)
    {
      String value_A1 = msg.substring(commaIndex + 1, zigIndex);
      String value_A2 = msg.substring(zigIndex + 1, spaceIndex);
      String value_A3 = msg.substring(spaceIndex + 1, starIndex);
      a1 = value_A1.toFloat();
      a2 = value_A2.toFloat();
      a3 = value_A3.toFloat();
      v1 = 30;
      v2 = 30;
      v3 = 30;

      // ici tu utilises a1 a2 a3
      deg1 = ((a1 * 180.0) / PI)+45;
      deg2 = ((a2 * 180.0) / PI)+27;
      if(home){
          deg3 = ((a3 * 180.0) / PI) + 360;
        }
      else{
        deg3 = ((a3 * 180.0) / PI);
      }
      DEBUG_SERIAL.print(deg1);
      DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(deg2);
      DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.println(deg3);      
    }
  }
  if(msg.startsWith("#Lineaire,")){
    lineaire = true;
    //#Lineaire,deg1~V1 deg2_V2-deg3
    int commaIndex = msg.indexOf(',');
    int zigIndex = msg.indexOf('~');
    int spaceIndex = msg.indexOf(' ');
    int UnderIndex = msg.indexOf('_');
    int tiretIndex = msg.indexOf('-');
    int exclaIndex = msg.indexOf('!');
    int starIndex = msg.indexOf('*');
    
    if (commaIndex != -1 && starIndex != -1)
    {
      String value_A1 = msg.substring(commaIndex + 1, zigIndex);
      String value_V1 = msg.substring(zigIndex + 1, spaceIndex);
      String value_A2 = msg.substring(spaceIndex + 1, UnderIndex);
      String value_V2 = msg.substring(UnderIndex + 1, tiretIndex);
      String value_A3 = msg.substring(tiretIndex + 1, exclaIndex);
      String value_V3 = msg.substring(exclaIndex + 1, starIndex);

      a1 = value_A1.toFloat();
      a2 = value_A2.toFloat();
      a3 = value_A3.toFloat();
      v1 = value_V1.toFloat()/1.374;
      v2 = value_V2.toFloat()/1.374;
      v3 = value_V3.toFloat()/1.374;
      
      // ici tu utilises a1 a2 a3
      deg1 = ((a1 * 180.0) / PI)+45;
      deg2 = ((a2 * 180.0) / PI)+27;
      if(home){
          deg3 = ((a3 * 180.0) / PI) + 360;
        }
      else{
        deg3 = ((a3 * 180.0) / PI);
      }
      DEBUG_SERIAL.print(deg1);
      DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.print(deg2);
      DEBUG_SERIAL.print(" ");
      DEBUG_SERIAL.println(deg3);      
    }
  }
}
void go_home(){
  if (home == true){
    dxl.setGoalPosition(Joint_3, 270, UNIT_DEGREE);
  }
  else{
    dxl.setGoalPosition(Joint_3, -90, UNIT_DEGREE);
  }
  dxl.setGoalPosition(Joint_1, 45, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_2, 117, UNIT_DEGREE);
}

void setup() {
  // put your setup code here, to run once:
  delay(2000);    // Délai additionnel pour avoir le temps de lire les messages sur la console.
  DEBUG_SERIAL.println("Starting position control ...");
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  //while(!DEBUG_SERIAL); // On attend que la communication série pour les messages soit prête.

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
  //bool ping_M2 = dxl.ping(Joint_1);
  bool ping_M1 = dxl.ping(Joint_2);
  //bool ping_M3 = dxl.ping(Joint_3);

  if (!ping_M1) {
    DEBUG_SERIAL.println("Could not ping motor1!");
    DEBUG_SERIAL.print("Last error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());

    return;
  }
  /*if (!ping_M2) {
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

  dxl.torqueOff(Joint_1);
  dxl.setOperatingMode(Joint_1, OP_EXTENDED_POSITION);
  dxl.torqueOn(Joint_1);*/

  dxl.torqueOff(Joint_2);
  dxl.setOperatingMode(Joint_2, OP_EXTENDED_POSITION);
  dxl.torqueOn(Joint_2);
  
  /*dxl.torqueOff(Joint_3);
  dxl.setOperatingMode(Joint_3, OP_EXTENDED_POSITION);
  dxl.torqueOn(Joint_3);
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, 15);*/
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, 50);
  //dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, 15);
  
  DEBUG_SERIAL.println("Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());

  /*if (dxl.getPresentPosition(Joint_3,UNIT_DEGREE)>= 230){
    dxl.setGoalPosition(Joint_3, 270, UNIT_DEGREE);
    home = true;
  }
  else{
    dxl.setGoalPosition(Joint_3, -90, UNIT_DEGREE);
    home = false;
  }
  dxl.setGoalPosition(Joint_1, 45, UNIT_DEGREE);*/
  dxl.setGoalPosition(Joint_2, 0, UNIT_DEGREE);//*/
  myServo.attach(servoPin);
  delay(2000);
}

void loop() {
  // Vérifie si Python a envoyé des données sur le port USB
  //Serial.println(home);
  /*if (DEBUG_SERIAL.available()) {
    lecture();
    Set_target_angle();
  }*/
  dxl.setGoalPosition(Joint_2, 270, UNIT_DEGREE);
  delay(500);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, 300);

  // Envoi immédiat aux moteurs
  /*Serial.println(dxl.getPresentPosition(DXL_ID_DH2028,UNIT_DEGREE));// max 216 degree min 19 degree
  Serial.println(dxl.getPresentPosition(DXL_ID_DH1007,UNIT_DEGREE));
  Serial.println(dxl.getPresentPosition(DXL_ID_DH2020,UNIT_DEGREE));
  delay(2000);//*/
}

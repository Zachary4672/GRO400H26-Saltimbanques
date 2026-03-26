// GRO400 - Exemple d'utilisation du OpenRB avec un moteur Dynamixel sous Platform.IO.
// Basé sur l'exemple de Position Control.
// Opère un moteur (à définir par la variable DXL_ID - 1 par défaut) en position en le faisant passer
// d'une position en pulsations (1000) à une autre en degrés (5.7) et vice-versa à chaque
// seconde.
// Écrit la position en cours en pulsations à la console série (accessible par DEBUG_SERIAL).
// N'oubliez-pas de configurer votre port série pour cette console à 115200 bauds.

#include <Dynamixel2Arduino.h>
#include <Servo.h>
//L'angle envoyé au moteurs pour ouvrir et fermer la pince
#define ouvert 20
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
// servomoteur pour la pince et poignet
Servo myServo;
const int servoPin = 5;
 
// TODO: À changer selon l'ID de votre moteur :
const uint8_t Joint_1 = 2; 
const uint8_t Joint_2 = 1;
const uint8_t Joint_3 = 3;

// variable global pour set la vitesse en unité dynamixel et la position en deg
float deg1 = 0;
float deg2 = 0;
float deg3 = 0;
int v1 = 30;
int v2 = 30;
int v3 = 30;
// variable selon ou est le moteur pour empècher le déplacement du mauvais côter
bool home = 0;
bool lineaire = 0;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void go_home(){
  float position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
  float position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
  float position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);

  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, 15);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, 15);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, 15);
  if (home == true){
    dxl.setGoalPosition(Joint_3, 270, UNIT_DEGREE);
    deg3 = 270;
  }
  else{
    dxl.setGoalPosition(Joint_3, -90, UNIT_DEGREE);
    deg3 = -90;
  }
  dxl.setGoalPosition(Joint_1, 45, UNIT_DEGREE);
  deg1 = 45;
  dxl.setGoalPosition(Joint_2, 90, UNIT_DEGREE);
  deg2 = 90;

  while (abs(deg1 - position1) > 1.0 || abs(deg2 - position2) > 1.0 || abs(deg3 - position3) > 1.0)
  {
    position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
    position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
    position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
  }
}
void get_new_velo(){
  String msg = DEBUG_SERIAL.readStringUntil('\n');

  if(msg.startsWith("#Lineaire,")){
    //#Lineaire,deg1~V1 deg2_V2-deg3*
    // le message est séparé par des caractère spéciaux afin de connaitre quel est la première valeur et ainsi de suite

    // défini la position de tout les caractéres dans le string envoyé du code python
    int commaIndex = msg.indexOf(',');
    int zigIndex = msg.indexOf('~');
    int spaceIndex = msg.indexOf(' ');
    int UnderIndex = msg.indexOf('_');
    int tiretIndex = msg.indexOf('-');
    int exclaIndex = msg.indexOf('!');
    int starIndex = msg.indexOf('*');
    
    //regarde si le message est valide en vérifiant s'il y a pas d'erreur de position
    if (commaIndex != -1 && starIndex != -1)
    {
      //utilise seulement les valeurs de vitesses vu qu'on connait déjà la position final
      String value_V1 = msg.substring(zigIndex + 1, spaceIndex);
      String value_V2 = msg.substring(UnderIndex + 1, tiretIndex);
      String value_V3 = msg.substring(exclaIndex + 1, starIndex);
      
      //redifinit les valeurs envoyé par python(rad/s) en valeur utile pour les dynamixels 
      v1 = abs(value_V1.toFloat()*9.549/0.229);
      v2 = abs(value_V2.toFloat()*9.549/0.229);
      v3 = abs(value_V3.toFloat()*9.549/0.229);
      if (v1==0)
      {
        v1 = 1;
      }
      if (v2==0)
      {
        v2 = 1;
      }
      if (v3==0)
      {
        v3 = 1;
      }
    }
  }
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, v1);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, v2);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, v3);
  dxl.setGoalPosition(Joint_1, deg1, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_2, deg2, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_3, deg3, UNIT_DEGREE);
}
void Set_target_angle(){
  //définit la position de départ des moteurs
  float position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
  float position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
  float position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
  // définit l'angle à qu'on veut atteindre
  dxl.setGoalPosition(Joint_1, deg1, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_2, deg2, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_3, deg3, UNIT_DEGREE);

  
  //vérifie quand les 3 moteurs ont atteint leurs positions finale 
  while (abs(deg1 - position1) > 7.0 || abs(deg2 - position2) > 7.0 || abs(deg3 - position3)> 7.0)
  {
    position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
    position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
    position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);

    if(lineaire == true){
      //si on est en déplacement linéaire change la vitesse de chaque moteur
      get_new_velo();
      //renvoie les position en RAD au code de cinématique et lui dit qu'Il bouge encore
      if (home)
      {
        String message = "Working "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3-360)*-DEG_TO_RAD);
        DEBUG_SERIAL.println(message); 
      }
      else{
        String message = "Working "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3)*-DEG_TO_RAD);
        DEBUG_SERIAL.println(message); 
      }
      //change les vitesses des 3 joints
      get_new_velo();
    }
  }
  // envoie au code python qu'il a fini et atteint la objectif
  if(lineaire == false){
    position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
    position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
    position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
    if (home)
    {
      String mess = "angle "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3-360)*-DEG_TO_RAD);
      DEBUG_SERIAL.println(mess);
      //DEBUG_SERIAL.println((position3-360)*DEG_TO_RAD); 
    }
    else{
      String mess = "angle "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3)*-DEG_TO_RAD);
      DEBUG_SERIAL.println(mess);
      //DEBUG_SERIAL.println((position3)*DEG_TO_RAD); a
    }
  }
  else{
    DEBUG_SERIAL.println("Doneline a b c");
  }
  //appel de gripper voir comment il faut se rendre
  //delay(5000);
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
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, 15);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, 15);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, 15);

      // ici tu utilises a1 a2 a3
      deg1 = ((a1 * 180.0) / PI)+45;
      deg2 = ((a2 * 180.0) / PI);
      if(home){
          deg3 = ((a3 * 180.0) / PI) + 360;
        }
      else{
        deg3 = ((a3 * 180.0) / PI);
      }     
    }
  }
  if(msg.startsWith("#Lineaire,")){
    lineaire = true;
    //#Lineaire,deg1~V1 deg2_V2-deg3
    int commaIndex = msg.indexOf(',');
    int zigIndex = msg.indexOf('~');
    int spaceIndex = msg.indexOf(' ');
    int UnderIndex = msg.indexOf('_');
    int hyperIndex = msg.indexOf('&');
    int exclaIndex = msg.indexOf('!');
    int starIndex = msg.indexOf('*');
    
    if (commaIndex != -1 && starIndex != -1)
    {
      String value_A1 = msg.substring(commaIndex + 1, zigIndex);
      String value_V1 = msg.substring(zigIndex + 1, spaceIndex);
      String value_A2 = msg.substring(spaceIndex + 1, UnderIndex);
      String value_V2 = msg.substring(UnderIndex + 1, hyperIndex);
      String value_A3 = msg.substring(hyperIndex + 1, exclaIndex);
      String value_V3 = msg.substring(exclaIndex + 1, starIndex);

      a1 = value_A1.toFloat();
      a2 = value_A2.toFloat();
      a3 = value_A3.toFloat();

      v1 = abs(value_V1.toFloat()*9.549/0.229);
      v2 = abs(value_V2.toFloat()*9.549/0.229);
      v3 = abs(value_V3.toFloat()*9.549/0.229);
      if (v1==0)
      {
        v1 = 1;
      }
      if (v2==0)
      {
        v2 = 1;
      }
      if (v3==0)
      {
        v3 = 1;
      }
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, v1);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, v2);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, v3);
      // ici tu utilises a1 a2 a3
      deg1 = ((a1 * 180.0) / PI)+45;
      deg2 = ((a2 * 180.0) / PI);
      if(home){
          deg3 = ((a3 * 180.0) / PI) + 360;
        }
      else{
        deg3 = ((a3 * 180.0) / PI);
      }   
    }
  }
  if(msg.startsWith("#LineaireReverse,")){
    lineaire = true;
    //#Lineaire,deg1~V1 deg2_V2-deg3
    int commaIndex = msg.indexOf(',');
    int zigIndex = msg.indexOf('~');
    int spaceIndex = msg.indexOf(' ');
    int UnderIndex = msg.indexOf('_');
    int hyperIndex = msg.indexOf('&');
    int exclaIndex = msg.indexOf('!');
    int starIndex = msg.indexOf('*');
    
    if (commaIndex != -1 && starIndex != -1)
    {
      String value_A1 = msg.substring(commaIndex + 1, zigIndex);
      String value_V1 = msg.substring(zigIndex + 1, spaceIndex);
      String value_A2 = msg.substring(spaceIndex + 1, UnderIndex);
      String value_V2 = msg.substring(UnderIndex + 1, hyperIndex);
      String value_A3 = msg.substring(hyperIndex + 1, exclaIndex);
      String value_V3 = msg.substring(exclaIndex + 1, starIndex);

      a1 = value_A1.toFloat();
      a2 = value_A2.toFloat();
      a3 = value_A3.toFloat();

      v1 = abs(value_V1.toFloat()*9.549/0.229);
      v2 = abs(value_V2.toFloat()*9.549/0.229);
      v3 = abs(value_V3.toFloat()*9.549/0.229);
      if (v1==0)
      {
        v1 = 1;
      }
      if (v2==0)
      {
        v2 = 1;
      }
      if (v3==0)
      {
        v3 = 1;
      }
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, v1);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, v2);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, v3);
      // ici tu utilises a1 a2 a3
      deg1 = ((a1 * 180.0) / PI)+45;
      deg2 = ((a2 * 180.0) / PI);
      if(home){ 
          deg3 = ((a3 * 180.0) / PI) + 360;
        }
      else{
        deg3 = ((a3 * 180.0) / PI);
      }   
    }
    float position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
    float position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
    float position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
    if (home)
    {
      String mess = "angle "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3-360)*-DEG_TO_RAD);
      DEBUG_SERIAL.println(mess);
      //DEBUG_SERIAL.println((position3-360)*DEG_TO_RAD); 
    }
    else{
      String mess = "angle "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3)*-DEG_TO_RAD);
      DEBUG_SERIAL.println(mess);
      //DEBUG_SERIAL.println((position3)*DEG_TO_RAD); a
    }
  }
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
  
  bool ping_M1 = dxl.ping(Joint_2);
  bool ping_M2 = dxl.ping(Joint_1);
  bool ping_M3 = dxl.ping(Joint_3);

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

  dxl.torqueOff(Joint_1);
  dxl.setOperatingMode(Joint_1, OP_EXTENDED_POSITION);
  dxl.torqueOn(Joint_1);

  dxl.torqueOff(Joint_2);
  dxl.setOperatingMode(Joint_2, OP_EXTENDED_POSITION);
  dxl.torqueOn(Joint_2);
  
  dxl.torqueOff(Joint_3);
  dxl.setOperatingMode(Joint_3, OP_EXTENDED_POSITION);
  dxl.torqueOn(Joint_3);
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, 15);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, 15);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, 15);
  
  DEBUG_SERIAL.println("Setup done.");
  DEBUG_SERIAL.print("Last error code: ");
  DEBUG_SERIAL.println(dxl.getLastLibErrCode());

  if (dxl.getPresentPosition(Joint_3,UNIT_DEGREE)>= 230){
    dxl.setGoalPosition(Joint_3, 270, UNIT_DEGREE);
    home = true;
  }
  else{
    dxl.setGoalPosition(Joint_3, -90, UNIT_DEGREE);
    home = false;
  }
  dxl.setGoalPosition(Joint_1, 45, UNIT_DEGREE);
  dxl.setGoalPosition(Joint_2, 90, UNIT_DEGREE);//*/
  myServo.attach(servoPin);
  delay(2000);
}

void loop() {
  // Vérifie si Python a envoyé des données sur le port USB
  //Serial.println(home);
  if (DEBUG_SERIAL.available()) {
    lecture();
    Set_target_angle();
  }
}

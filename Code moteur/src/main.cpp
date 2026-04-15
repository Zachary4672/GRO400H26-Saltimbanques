// GRO400 - Exemple d'utilisation du OpenRB avec un moteur Dynamixel sous Platform.IO.
// Basé sur l'exemple de Position Control.
// Opère un moteur (à définir par la variable DXL_ID - 1 par défaut) en position en le faisant passer
// d'une position en pulsations (1000) à une autre en degrés (5.7) et vice-versa à chaque
// seconde.
// Écrit la position en cours en pulsations à la console série (accessible par DEBUG_SERIAL).
// N'oubliez-pas de configurer votre port série pour cette console à 115200 bauds.

#include <Dynamixel2Arduino.h>
#include <Servo.h>
#include <math.h>
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
Servo Pince;
Servo Poignet;
const int servoPoignet = 5;
const int servoPince = 6;
 
// TODO: À changer selon l'ID de votre moteur :
const uint8_t Joint_1 = 2; 
const uint8_t Joint_2 = 1;
const uint8_t Joint_3 = 3;

const byte buttonPin = 2;

// variable global pour set la vitesse en unité dynamixel et la position en deg
float deg1 = 0;
float deg2 = 0;
float deg3 = 0;
float deg4 = 0;
int v1 = 15;
int v2 = 15;
int v3 = 15;
// variable selon ou est le moteur pour empècher le déplacement du mauvais côter
bool lineaire = 0;
bool interuption = 0;

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void monInterruption();

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
      v1 = abs(value_V1.toFloat())*9.549/0.229;
      v2 = abs(value_V2.toFloat())*9.549/0.229;
      v3 = abs(value_V3.toFloat())*9.549/0.229;
      //si la vitesse est à 0 on la met à 1 pour éviter que le moteur aille à pleine vitesse
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
  //change la vitesse de chaque moteur
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, v1);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, v2);
  dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, v3);
  
  //faut redéfinir la position voulu pour chauqe moteurs afin que la vitesse puisse changer en cours de route
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
  
  //vérifie quand les 3 moteurs ont atteint leurs positions finale et répète la boucle jusqu'à ce que la contrainte soit respecté ou que le bouton d'urgence soit pas appuyé
  while ((abs(deg1 - position1) > 4.0 || abs(deg2 - position2) > 4.0 || abs(deg3 - position3)> 4.0) && !interuption)
  {
    position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
    position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
    position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);

    if(lineaire == true){
      //si on est en déplacement linéaire change la vitesse de chaque moteur
      //renvoie les position en RAD au code de cinématique et lui dit qu'Il bouge encore
      String message = "Working "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3-360)*-DEG_TO_RAD);
      DEBUG_SERIAL.println(message);
      //change les vitesses des 3 joints
      get_new_velo();
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

  while (dxl.getPresentPosition(Joint_1,UNIT_DEGREE)>=225)
  {
    dxl.torqueOff(Joint_1);
    if(dxl.getPresentPosition(Joint_1,UNIT_DEGREE)<=0)
    {
      dxl.reboot(Joint_1);
      dxl.torqueOff(Joint_1);
      dxl.setOperatingMode(Joint_1, OP_EXTENDED_POSITION);
      dxl.torqueOn(Joint_1);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, 15);
    }
  }
  dxl.setGoalPosition(Joint_1, 45, UNIT_DEGREE);

  while (dxl.getPresentPosition(Joint_2,UNIT_DEGREE)>=270)
  {
    dxl.torqueOff(Joint_2);
    if(dxl.getPresentPosition(Joint_2,UNIT_DEGREE)>360)
    {
      dxl.reboot(Joint_2);
      dxl.torqueOff(Joint_2);
      dxl.setOperatingMode(Joint_2, OP_EXTENDED_POSITION);
      dxl.torqueOn(Joint_2);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, 15);
    }
  }
  
  dxl.setGoalPosition(Joint_2, 70, UNIT_DEGREE);

  while (dxl.getPresentPosition(Joint_3,UNIT_DEGREE)>=0 && dxl.getPresentPosition(Joint_3,UNIT_DEGREE)<=180)
  {
    dxl.torqueOff(Joint_3);
    if(dxl.getPresentPosition(Joint_3,UNIT_DEGREE)<=0)
    {
      dxl.reboot(Joint_3);
      dxl.torqueOff(Joint_3);
      dxl.setOperatingMode(Joint_3, OP_EXTENDED_POSITION);
      dxl.torqueOn(Joint_3);
      dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, 15);
    }
  }
  dxl.setGoalPosition(Joint_3, 300, UNIT_DEGREE);
  Poignet.attach(servoPoignet);
  Pince.attach(servoPince);
  Pince.write(ouvert);
  Poignet.write(90);

  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), monInterruption, FALLING);
  delay(2000);
}

void loop() {
  float position1, position2, position3;
  interuption = 0;
  
  //Attend que le code Python a envoyé des données
  if (DEBUG_SERIAL.available()) {

    //Valeurs pour inscrire les angles dans une variable local
    float a1, a2, a3, a4;
    
    //enregistre le message envoyé par python dans un string jusqu'à ce qu'il rencontre la fin du message (\n)
    String msg = DEBUG_SERIAL.readStringUntil('\n');
    
    // Si le message commence par #Joint donc le robot doit faire un mouvement en joint et rentre dans la boucle
    if(msg.startsWith("#Joint,")){
      lineaire = false; // définit que le robot bouge en joint

      // donne des valeurs d'index pour chaque symbole utilisé pour séparer les différentes valeurs dans le message envoyé par python
      //le message = #Joint,deg1~deg2 deg3/deg4*
      int commaIndex = msg.indexOf(',');
      int zigIndex = msg.indexOf('~');
      int spaceIndex = msg.indexOf(' ');
      int slashIndex = msg.indexOf('/');
      int starIndex = msg.indexOf('*');

      //Vérifie si le message est valide en vérifiant s'il y a pas d'erreur de position des symboles et si c'est le cas alors on peut extraire les valeurs du message
      if (commaIndex != -1 && starIndex != -1)
      {
        //mets les valeurs des angles selon leurs position dans le message
        String value_A1 = msg.substring(commaIndex + 1, zigIndex);
        String value_A2 = msg.substring(zigIndex + 1, spaceIndex);
        String value_A3 = msg.substring(spaceIndex + 1, slashIndex);
        String value_A4 = msg.substring(slashIndex + 1, starIndex);

        //mets les valeurs des angle dans des variable temporaire
        a1 = value_A1.toFloat();
        a2 = value_A2.toFloat();
        a3 = value_A3.toFloat();
        a4 = value_A4.toFloat();

        //définit les vitesse de chaque moteur à 15 pour le mouvment en joint
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, 15);
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, 15);
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, 15);

        // Redéfinit les variable utilisé, deg1 deg2 deg3 pour les utilisé dans set_target_angle
        deg1 = ((a1 * 180.0) / PI) + 45;
        deg2 = ((a2 * 180.0) / PI);
        deg3 = ((a3 * 180.0) / PI) + 360;

        //commence le mouvement en joint vers la position voulu
        
        Set_target_angle();

        //déplace le poignet avant de faire le déplacement linéaire pour que le poignet soit dans la bonne position pendant le déplacement linéaire
        if(a4>180)
        {
          deg4 = abs(a4) - 180;
        }
        deg4 = abs(a4);

        Poignet.write(deg4);

        //renvoie la position actuel des moteur en RAD au code de cinématique et lui dit que le mouvement est fini
        position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
        position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
        position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE); 
        String mess = "DoneJoint "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3-360)*-DEG_TO_RAD);
        DEBUG_SERIAL.println(mess);
      }
    }
    // Si le message commence par #Lineaire donc le robot doit faire un mouvement linéaire et rentre dans la boucle
    if(msg.startsWith("#Lineaire,")){
      lineaire = true; // Garde en mémoire que le robot bouge linéairement

      //Index chaque symbole utilisé pour séparer les différentes valeurs dans le message envoyé par python
      //message = #Lineaire,deg1~V1 deg2_V2-deg3/deg4*
      int commaIndex = msg.indexOf(',');
      int zigIndex = msg.indexOf('~');
      int spaceIndex = msg.indexOf(' ');
      int UnderIndex = msg.indexOf('_');
      int hyperIndex = msg.indexOf('&');
      int exclaIndex = msg.indexOf('!');
      int starIndex = msg.indexOf('*');

      //Vérifie si le message est valide en vérifiant s'il y a pas d'erreur de position des symboles et si c'est le cas alors on peut extraire les valeurs du message
      if (commaIndex != -1 && starIndex != -1)
      {
        //Mets les valeurs des angles selon leurs position dans le message
        String value_A1 = msg.substring(commaIndex + 1, zigIndex);
        String value_V1 = msg.substring(zigIndex + 1, spaceIndex);
        String value_A2 = msg.substring(spaceIndex + 1, UnderIndex);
        String value_V2 = msg.substring(UnderIndex + 1, hyperIndex);
        String value_A3 = msg.substring(hyperIndex + 1, exclaIndex);
        String value_V3 = msg.substring(exclaIndex + 1, starIndex);


        //mets les valeurs des angle dans des variable temporaire
        a1 = value_A1.toFloat();
        a2 = value_A2.toFloat();
        a3 = value_A3.toFloat();

        //Définti les vitesses de chaque moteur en redéfinissant les variable v1 v2 v3 pour les utilisé dans set_target_angle
        v1 = abs(value_V1.toFloat()*9.549/0.229);
        v2 = abs(value_V2.toFloat()*9.549/0.229);
        v3 = abs(value_V3.toFloat()*9.549/0.229);

        //si la vitesse est à 0 on la met à 1 pour éviter que le moteur aille à pleine vitesse
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

        // redéfinit la vitesse de chaque moteur
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, v1);
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, v2);
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, v3);

        //Définit les variable deg1 deg2 deg3 pour les utilisé dans set_target_angle et deg_poignet pour le poignet
        deg1 = ((a1 * 180.0) / PI) + 45;
        deg2 = ((a2 * 180.0) / PI);
        deg3 = ((a3 * 180.0) / PI) + 360;

        //commence le mouvement linéaire vers la position voulu
        Set_target_angle();

        //renvoie la position actuel des moteur en RAD au code de cinématique et lui dit que le mouvement est fini
        position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
        position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
        position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE); 
        String mess = "Doneline "+String((position1-45)*DEG_TO_RAD)+" "+String((position2)*-DEG_TO_RAD)+" "+String((position3-360)*-DEG_TO_RAD);
        DEBUG_SERIAL.println(mess);
      }
    }

    if(msg.startsWith("#LineaireReverse,")){
      lineaire = true;// Garde en mémoire que le robot bouge linéairement

      //Index chaque symbole utilisé pour séparer les différentes valeurs dans le message envoyé par python
      //#LineaireReverse,deg1~V1 deg2_V2-deg3*
      int commaIndex = msg.indexOf(',');
      int zigIndex = msg.indexOf('~');
      int spaceIndex = msg.indexOf(' ');
      int UnderIndex = msg.indexOf('_');
      int hyperIndex = msg.indexOf('&');
      int exclaIndex = msg.indexOf('!');
      int starIndex = msg.indexOf('*');
      
      //Vérifie si le message est valide en vérifiant s'il y a pas d'erreur de position des symboles et si c'est le cas alors on peut extraire les valeurs du message
      if (commaIndex != -1 && starIndex != -1)
      {
        //Mets les valeurs des angles selon leurs position dans le message
        String value_A1 = msg.substring(commaIndex + 1, zigIndex);
        String value_V1 = msg.substring(zigIndex + 1, spaceIndex);
        String value_A2 = msg.substring(spaceIndex + 1, UnderIndex);
        String value_V2 = msg.substring(UnderIndex + 1, hyperIndex);
        String value_A3 = msg.substring(hyperIndex + 1, exclaIndex);
        String value_V3 = msg.substring(exclaIndex + 1, starIndex);

        //mets les valeurs des angle dans des variable temporaire
        a1 = value_A1.toFloat();
        a2 = value_A2.toFloat();
        a3 = value_A3.toFloat();
        
        //mets les valeurs des vitesses dans des variable temporaire
        v1 = abs(value_V1.toFloat()*9.549/0.229);
        v2 = abs(value_V2.toFloat()*9.549/0.229);
        v3 = abs(value_V3.toFloat()*9.549/0.229);

        //confirme que la vitesse n'est pas à 0 pour éviter que le moteur aille à pleine vitesse
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

        // redéfinit la vitesse de chaque moteur
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_1, v1);
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_2, v2);
        dxl.writeControlTableItem(PROFILE_VELOCITY, Joint_3, v3);

        // transformé les angles de radian en degré et les redéfinit pour les utilisé dans set_target_angle
        deg1 = ((a1 * 180.0) / PI) + 45;
        deg2 = ((a2 * 180.0) / PI);
        deg3 = ((a3 * 180.0) / PI) + 360;

        //commence le mouvement linéaire vers la position voulu
        Set_target_angle();

        //repositionne le poignet à 90 degré une fois le mouvement linéaire inverse fini
        Poignet.write(90);

        //Renvoie un message disant que le mouvement est fini
        DEBUG_SERIAL.println("DoneLineReverse");
      }
    }
    //Regardes si le message commence par #Pince pour savoir s'il doit fermer ou ouvrir la pince 
    if (msg.startsWith("#Pince,"))
    {
      //Index chaque symbole utilisé pour séparer les différentes valeurs dans le message envoyé par python
      int commaIndex = msg.indexOf(',');
      int starIndex = msg.indexOf('*');
      
      //Vérifie si le message est valide en vérifiant s'il y a pas d'erreur de position des symboles et si c'est le cas alors on peut extraire les valeurs du message
      if (commaIndex != -1 && starIndex != -1){

        //Mets les valeurs des angles selon leurs position dans le message
        String value_A1 = msg.substring(commaIndex + 1, starIndex);

        //transfoirme la valeur de string en int pour savoir si on doit ouvrir ou fermer la pince
        int closing = value_A1.toInt();

        //si la valeur est à 1 alors on ferme la pince sinon on l'ouvre
        if (closing == 1)
        {
          Pince.write(fermer);
          
        }
        else{
          Pince.write(ouvert);
        }
      }
    }
  }
}

void monInterruption(){
  // boutton urgence pour arreter le mouvement du robot
  float position1, position2, position3;
  bool erreur = 1;
  //reste dans la boucle temops que le robot n'est pas prêt à bouger à nouveau
  while(erreur == 1){
    position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
    position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
    position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
    dxl.setGoalPosition(Joint_3, position1, UNIT_DEGREE);
    dxl.setGoalPosition(Joint_3, position2, UNIT_DEGREE);
    dxl.setGoalPosition(Joint_3, position3, UNIT_DEGREE);
    Pince.write(ouvert);

    if(DEBUG_SERIAL.available()){
      String msg = DEBUG_SERIAL.readStringUntil('\n');
      //attend le message de reset pour éteindre le torque des moteur afin de pouvoir les bouger à la main
      if(msg.startsWith("#Reset"))
      {
        dxl.torqueOff(Joint_1);
        dxl.torqueOff(Joint_2);
        dxl.torqueOff(Joint_3);
      }
      //une fois que le robot est bien positionner attend le message #Good pour réactiver c'est moteur
      if(msg.startsWith("#Good"))
      {
        position1 = dxl.getPresentPosition(Joint_1, UNIT_DEGREE);
        position2 = dxl.getPresentPosition(Joint_2, UNIT_DEGREE);
        position3 = dxl.getPresentPosition(Joint_3, UNIT_DEGREE);
        dxl.setGoalPosition(Joint_3, position1, UNIT_DEGREE);
        dxl.setGoalPosition(Joint_3, position2, UNIT_DEGREE);
        dxl.setGoalPosition(Joint_3, position3, UNIT_DEGREE);
        dxl.torqueOn(Joint_1);
        dxl.torqueOn(Joint_2);
        dxl.torqueOn(Joint_3);
        interuption = 1;
        erreur = 0;
      }
    }
  }
}
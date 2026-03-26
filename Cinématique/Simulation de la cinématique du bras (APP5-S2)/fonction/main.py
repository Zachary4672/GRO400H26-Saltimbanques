import bras_robot
import affichage
import serial
import time
import donnees
 
PORT = "COM9" #Ce port doit être défini par l'utilisateur
BAUD = 115200
MemoryMessage = None
turn = 0
 
#Fonction: connect_serial: connection du programme avec l'arduino
def connect_serial():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)  # timeout=0 => non bloquant
        time.sleep(2)  # OpenRB reboot
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Connexion établie avec l'OpenRB-150")
        return ser
    except serial.SerialException as e:
        print(f"Erreur : impossible d'ouvrir {PORT} -> {e}")
        return None
   
#Fonction: envoyer_angles(ser, a1, b1, a2, b2, a3, b3): Envoie des angles à l'arduino pour un prochain mouvement
def envoyer_angles(ser, a1, b1, a2, b2, a3, b3):
    global turn
    if directive == "Joint":
        message = "#"+ directive + f",{a1:.4f}~{a2:.4f} {a3:.4f}*\n"
    elif directive == "Lineaire" and turn == 0:
        message = "#" + directive + f",{a1:.4f}~{b1:.4f} {a2:.4f}_{b2:.4f}&{a3:.4f}!{b3:.4f}*\n"
    elif directive == "Lineaire" and turn == 1:
        message = "#" + "LineaireReverse" + f",{a1:.4f}~{b1:.4f} {a2:.4f}_{b2:.4f}&{a3:.4f}!{b3:.4f}*\n"
    ser.write(message.encode("utf-8"))
 
 
 
 
#Fonction: lire_responses permet de recevoir les informations du buffer "ser" et vérifier les réponses
def lire_latest_ligne_complete(ser, last_line = None):
 

    # Lire tout ce qui est disponible
    data = ser.read(ser.in_waiting or 0)
    if not data:
        return last_line

    # Accumuler dans un buffer local (à garder entre appels)
    text = data.decode("utf-8", errors="replace")
    lines = text.splitlines()
    if lines:
        return lines[-1]   # dernière ligne reçue
    return last_line
 
 
 
 
# --- Main ---
MemJ1 = 0
Parts = ["", "", "", ""]
MemJ2 = 0
MemJ3 = 0
J1 = 0
J2 = 0
J3 = 0
J4 = 0
V1 = 0
V2 = 0
V3 = 0
directive = "Joint" #Directive is set to joint as a base value
ser = connect_serial()
if ser is None:
    raise SystemExit("Port série indisponible.")
 

while True:
    try:
        MemoryMessage = None
        Parts[0] = "Turn"
        if turn == 0:
            directive = "Joint"
            bras_robot.Calculate(1, 0.0, 0.0, 0.0)

            ParamInput = True
            tol = 0.1
            if tol < abs(J1 - bras_robot.angles[0]) or tol < abs(J2 - bras_robot.angles[1]) or tol < abs(J3 - bras_robot.angles[2]):
                J1 = bras_robot.angles[0]
                J2 = bras_robot.angles[1]
                J3 = bras_robot.angles[2]
                envoyer_angles(ser,J1, V1, -J2, V2, -J3, V3)
                ParamInput = False






        if ParamInput == False:


            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)

                if MemoryMessage != None:
                    Parts = MemoryMessage.split()
                    if len(Parts) != 4:
                        MemoryMessage = None
                        continue
            directive = "Lineaire"

            if turn == 0:
                bras_robot.Calculate(2, 0.0, 0.0, 0.0)
            else:
                bras_robot.Calculate(1, 0.0, 0.0, 0.0)
            

            while Parts[0] != "Doneline":
                bras_robot.Calculate(0, float(Parts[1]), float(Parts[2]), float(Parts[3]))
                

                if tol < abs(J1 - bras_robot.angles[0]) or tol < abs(J2 - bras_robot.angles[1]) or tol < abs(J3 - bras_robot.angles[2] or turn == 1):
                    V1 = bras_robot.vitesse[0]
                    V2 = bras_robot.vitesse[1]
                    V3 = bras_robot.vitesse[2]
                    MemJ1 = bras_robot.angles[0]
                    MemJ2 = bras_robot.angles[1]
                    MemJ3 = bras_robot.angles[2]
                    print(ser,MemJ1, V1, -MemJ2, V2, -MemJ3, V3)
                    envoyer_angles(ser, MemJ1, V1, -MemJ2, V2, -MemJ3, V3)
                Test = False
                while Test == False:
                    MemoryMessage = lire_latest_ligne_complete(ser)
                    if MemoryMessage != None:
                        Parts = MemoryMessage.split()
                  
                        if len(Parts) != 4:
                            MemoryMessage = None
                            continue
                        Test = True

                time.sleep(0.02)
            if Parts[0] == "Doneline":
                print("Done")
                turn = 1

 
 
    except (serial.SerialException, OSError) as e:
        print(f"Connexion série perdue: {e}")
        try:
            ser.close()
        except Exception:
            pass
        # tentative simple de reconnexion
        ser = None
        while ser is None:
            time.sleep(1)
            ser = connect_serial()
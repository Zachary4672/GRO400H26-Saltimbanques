import bras_robot
import affichage
import serial
import time
 
PORT = "COM3" #Ce port doit être défini par l'utilisateur
BAUD = 115200


#Fonction: connect_serial: connection du programme avec l'arduino
def connect_serial():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0)  # timeout=0 => non bloquant
        time.sleep(2)  # OpenRB reboot
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Connexion établie avec l'OpenRB-150")
        return ser
    except serial.SerialException as e:
        print(f"Erreur : impossible d'ouvrir {PORT} -> {e}")
        return None
    
#Fonction: envoyer_angles(ser, a1, a2, a3): Envoie des angles à l'arduino pour un prochain mouvement
def envoyer_angles(ser, a1, a2, a3):
    message = f"{a1:.2f},{a2:.2f},{a3:.2f}\n"
    ser.write(message.encode("utf-8"))


#Fonction: lire_responses permet de recevoir les informations du buffer "ser" et vérifier les réponses
def lire_reponses(ser, max_lines=10):
    lines = []
    for _ in range(max_lines):
        if ser.in_waiting <= 0:
            break
        line = ser.readline()  # non bloquant car timeout=0
        if not line:
            break
        try:
            lines.append(line.decode("utf-8", errors="replace").strip())
        except Exception:
            lines.append(str(line))
    return lines
 
# --- Main ---

ser = connect_serial()
if ser is None:
    raise SystemExit("Port série indisponible.")

period = 0.1  # 100 ms
next_t = time.perf_counter()

while True:
    try:
        bras_robot.Calculate()
        affichage.Draw()
        # Exemple : envoyer des positions
        J1 = bras_robot.angles[0]
        J2 = bras_robot.angles[1]
        J3 = bras_robot.angles[2]
        envoyer_angles(ser,J1, J2, J3)

        # Lire et afficher toutes les réponses dispo
        #for rep in lire_reponses(ser):
            #print(f"OpenRB dit : {rep}")

        #Timer avant le prochain try
        next_t += period
        time.sleep(max(0, next_t - time.perf_counter()))

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

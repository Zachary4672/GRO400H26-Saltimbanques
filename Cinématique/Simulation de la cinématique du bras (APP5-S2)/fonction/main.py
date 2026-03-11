import bras_robot
import affichage
import serial
import time
 
PORT = "COM9" #Ce port doit être défini par l'utilisateur
BAUD = 115200
MemoryMessage = ""
 
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
 
 
    message = "#"+ directive + f",{a1:.2f}~{a2:.2f} {a3:.2f}*\n"
    ser.write(message.encode("utf-8"))
 
 
 
 
#Fonction: lire_responses permet de recevoir les informations du buffer "ser" et vérifier les réponses
def lire_premiere_ligne_complete(ser, rxbuf=bytearray()):
 
    n = ser.in_waiting
    if n:
        rxbuf += ser.read(n)
 
    pos = rxbuf.find(b"\n")
    if pos == -1:
        return None, rxbuf
 
    raw = rxbuf[:pos]
    del rxbuf[:pos+1]
 
    raw = raw.rstrip(b"\r")
    return raw.decode("ascii", errors="replace").strip(), rxbuf
 
 
 
 
# --- Main ---
J1 = 0
J2 = 0
J3 = 0
rxbuf = bytearray()
directive = "Joint" #Directive is set to joint as a base value
ser = connect_serial()
if ser is None:
    raise SystemExit("Port série indisponible.")
 
period = 0.1  # 100 ms
next_t = time.perf_counter()
 
while True:
    try:
 
        bras_robot.Calculate()
        #affichage.Draw()
 
        if J1 != bras_robot.angles[0] or J2 != bras_robot.angles[1] or J3 != bras_robot.angles[2]:
            J1 = bras_robot.angles[0]
            J2 = bras_robot.angles[1]
            J3 = bras_robot.angles[2]
 
            envoyer_angles(ser,J1, -J2, -J3)
 
        #Timer avant le prochain try
        next_t += period
        time.sleep(max(0, next_t - time.perf_counter()))
 
        MemoryMessage, rxbuf = lire_premiere_ligne_complete(ser, rxbuf)
        if MemoryMessage is not None:
            print("RX:", MemoryMessage)
            print(J1, -J2, -J3)
 
 
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
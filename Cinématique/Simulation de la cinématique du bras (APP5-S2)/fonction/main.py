import bras_robot
import affichage
import serial
import time
 
PORT = "COM9"
BAUD = 115200
MemoryMessage = None


# TRAJECTOIRE

trajectory = [
    (0.0, 0.0, 150.0),   # HOME
    (100.0, 0.0, 120.0), # Approche pick
    (100.0, 0.0, 50.0),  # PICK
    (100.0, 0.0, 120.0), # Retrait pick
    (0.0, 100.0, 120.0), # Approche drop
    (0.0, 100.0, 50.0),  # DROP
    (0.0, 100.0, 120.0), # Retrait drop
    (0.0, 0.0, 150.0),   # HOME
]

traj_index = 0


# Connexion série

def connect_serial():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Connexion établie avec l'OpenRB-150")
        return ser
    except serial.SerialException as e:
        print(f"Erreur : impossible d'ouvrir {PORT} -> {e}")
        return None
   

# Envoi angles

def envoyer_angles(ser, a1, b1, a2, b2, a3, b3, a4, b4):

    if directive == "Joint":
        message = "#" + directive + f",{a1:.2f}~{a2:.2f} {a3:.2f} {a4:.2f}*\n"
    else:
        message = "#" + directive + f",{a1:.2f}~{b1:.2f} {a2:.2f}_{b2:.2f}-{a3:.2f}!{b3:.2f}|{a4:.2f}:{b4:.2f}*\n"

    ser.write(message.encode("utf-8"))
 

# Lecture série

def lire_latest_ligne_complete(ser, last_line = None):

    data = ser.read(ser.in_waiting or 0)
    if not data:
        return last_line

    text = data.decode("utf-8", errors="replace")
    lines = text.splitlines()

    if lines:
        return lines[-1]

    return last_line
 

# MAIN

J1 = 0
J2 = 0
J3 = 0
J4 = 0

V1 = 0
V2 = 0
V3 = 0
V4 = 0

directive = "Joint"

ser = connect_serial()
if ser is None:
    raise SystemExit("Port série indisponible.")
 
while True:
    try:
       
        directive = "Joint"

        
        # TRAJECTOIRE
        
        if traj_index >= len(trajectory):
            traj_index = 0

        x, y, z = trajectory[traj_index]

        bras_robot.Calculate(True, x, y, z)
 
        ParamInput = True
 
        if (J1 != bras_robot.angles[0] or
            J2 != bras_robot.angles[1] or
            J3 != bras_robot.angles[2] or
            J4 != bras_robot.angles[3]):

            J1 = bras_robot.angles[0]
            J2 = bras_robot.angles[1]
            J3 = bras_robot.angles[2]
            J4 = bras_robot.angles[3]

            envoyer_angles(ser, J1, V1, -J2, V2, -J3, V3, J4, V4)
            ParamInput = False
 
        if ParamInput == False:
 
            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)
 
            if MemoryMessage != None:
                Parts = MemoryMessage.split()
           
            while Parts[0] != "Done":
                directive = "Lineaire"

                bras_robot.Calculate(False,
                                     float(Parts[1]),
                                     float(Parts[2]),
                                     float(Parts[3]))
 
                if (J1 != bras_robot.angles[0] or
                    J2 != bras_robot.angles[1] or
                    J3 != bras_robot.angles[2] or
                    J4 != bras_robot.angles[3]):

                    J1 = bras_robot.angles[0]
                    J2 = bras_robot.angles[1]
                    J3 = bras_robot.angles[2]
                    J4 = bras_robot.angles[3]

                    V1 = bras_robot.vitesse[0]
                    V2 = bras_robot.vitesse[1]
                    V3 = bras_robot.vitesse[2]
                    V4 = 0  # ou vitesse[3] si implémenté

                envoyer_angles(ser, J1, V1, -J2, V2, -J3, V3, J4, V4)

                MemoryMessage = lire_latest_ligne_complete(ser)
                Parts = MemoryMessage.split()

                time.sleep(0.02)

            
            traj_index += 1

            # Pick / Drop
            if traj_index == 2:
                print("FERMER PINCE")

            if traj_index == 5:
                print("OUVRIR PINCE")

            MemoryMessage = None
 
    except (serial.SerialException, OSError) as e:
        print(f"Connexion série perdue: {e}")
        try:
            ser.close()
        except Exception:
            pass

        ser = None
        while ser is None:
            time.sleep(1)
            ser = connect_serial()
import serial
import time
import donnees
import bras_robot
import json
from pick_and_place import generate_trajectory, rouge, bleu, jaune

PORT  = "/dev/ttyACM0"  # à adapter selon votre système
BAUD  = 115200


# Position scan
X_SCAN = donnees.Donnees.x_scan
Y_SCAN = donnees.Donnees.y_scan
Z_SCAN = donnees.Donnees.z_scan
LastMessage = None
turn = 0
directive = "Joint"

# Série


def connect_serial():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print("Connexion établie avec l'OpenRB-150")
        return ser
    except serial.SerialException as e:
        print(f"Erreur série : {e}")
        return None
    

def writingJSON(x_pos, y_pos, z_pos):
    filename = "donnees.json"
    data = {
        "x" : x_pos,
        "y" : y_pos,
        "z" : z_pos
    }
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)
    

#def envoyer_joint(ser, j1, j2, j3, j4=0):
    #msg = f"#Joint,{j1:.4f}~{j2:.4f} {j3:.4f} {j4:.4f}*\n"
    #ser.write(msg.encode("utf-8"))


#def envoyer_lineaire(ser, j1, v1, j2, v2, j3, v3, j4=0):
   # msg = f"#Lineaire,{j1:.4f}~{v1:.4f} {j2:.4f}_{v2:.4f}&{j3:.4f}!{v3:.4f}*\n"
   # ser.write(msg.encode("utf-8"))


#def envoyer_reverse(ser, j1, v1, j2, v2, j3, v3, j4=0):
    #msg = f"#LineaireReverse,{j1:.4f}~{v1:.4f} {j2:.4f}_{v2:.4f}&{j3:.4f}!{v3:.4f}*\n"
    #ser.write(msg.encode("utf-8"))


#def envoyer_pince(ser, fermer=True):
    #msg = f"#Pince,1*\n" if fermer else f"#Pince,0*\n"
    #ser.write(msg.encode("utf-8"))

#Fonction: envoyer_angles(ser, a1, b1, a2, b2, a3, b3): Envoie des angles à l'arduino pour un prochain mouvement
def envoyer_angles(ser, a1, b1, a2, b2, a3, b3, a4):
    global turn, directive
    if directive == "Joint":
        message = "#"+ directive + f",{a1:.4f}~{a2:.4f} {a3:.4f}<{a4:.4f}*\n"
    elif directive == "Lineaire" and turn == 0:
        message = "#" + directive + f",{a1:.4f}~{b1:.4f} {a2:.4f}_{b2:.4f}&{a3:.4f}!{b3:.4f}*\n"
    elif directive == "LineaireReverse":
        message = "#" + "LineaireReverse" + f",{a1:.4f}~{b1:.4f} {a2:.4f}_{b2:.4f}&{a3:.4f}!{b3:.4f}*\n"
    elif directive == "Pince1":
        message = f"#Pince,1*\n"
    elif directive == "Pince0":
        message = f"#Pince,0*\n"
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

#def attendre_doneline(ser, timeout=15):
    #debut = time.time()
    #done_recu = False
   # while time.time() - debut < timeout:
       # ligne = ser.readline().decode("utf-8", errors="replace").strip()
        #if not ligne:
        #    continue
        #if ligne.startswith("Working"):
        #    continue
        #if ligne.startswith("Doneline"):
        #    done_recu = True
        #    continue
        #if ligne.startswith("Donejoint"):
        #    done_recu = True
        #    continue
        #if done_recu:
        #    parts = ligne.split()
        #    if len(parts) >= 4:
        #        return float(parts[1]), float(parts[2]), float(parts[3])
    #return None


# Aller à une position cartésienne (Joint)


def aller_a(ser, x, y, z, fA1=0.0, fA2=0.0, fA3=0.0):
    global directive

    writingJSON(x, y, z)
    bras_robot.Calculate(1, fA1, fA2, fA3, turn)
    j1, j2, j3, angle = bras_robot.angles

    directive = "Joint"

    envoyer_angles(ser, j1,0, -j2,0, -j3,0, angle)

    Parts = [None]
    MemoryMessage = None
    while Parts[0] != "DoneJoint":
        while MemoryMessage == None:
            MemoryMessage = lire_latest_ligne_complete(ser)
            if MemoryMessage != None:
                Parts = MemoryMessage.split()
                if len(Parts) != 4:
                    MemoryMessage = None
                    continue
    result = Parts

    print(f"  Position atteinte : j1={result[0]:.3f} j2={result[1]:.3f} j3={result[2]:.3f}")
    return result[0], result[1], result[2], angle

def detecter_pilules():
    # Simuler la détection de pilules
    # TODO : remplacer par la vraie détection caméra
    return {
        "pilules": [
            {"x": 0.2, "y": 0.01, "angle": 0, "couleur": "rouge"},
            {"x": 0.2, "y": 0.0, "angle": 45, "couleur": "bleu"},
            {"x": 0.2, "y": -0.01, "angle": -30, "couleur": "jaune"},
        ]   
    }


# Exécuter un point de trajectoire

def executer_point(ser, pt, fA1, fA2, fA3):
    x, y, z, angle, type_mvt, pince, couleur = pt
    global directive
    writingJSON(x, y, z)

    bras_robot.Calculate(1, fA1, fA2, fA3, 1)
    j1, j2, j3, j4 = bras_robot.angles

    if type_mvt == 0:       # Joint
        directive = "Joint"
        envoyer_angles(ser, j1, 0, -j2, 0, -j3, 0, j4)

        Parts = [None]
        MemoryMessage = None
        while Parts[0] != "DoneJoint":
            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)
                if MemoryMessage != None:
                    Parts = MemoryMessage.split()
                    if len(Parts) != 4:
                        MemoryMessage = None
                        continue
        

    elif type_mvt == 1:     # Lineaire
        Move_lineaire("DoneJoint")

    else:                   # Reverse
        Move_ReverseLineaire("DoneLine")

    if pince == 1:
        print(f"  → FERMER pince")
        directive = "Pince1"
        envoyer_angles(ser, 0,0,0,0,0,0,0)
        time.sleep(0.8)
        Move_ReverseLineaire("DoneLine")
    else:
        print(f"  → OUVRIR pince")
        directive = "Pince0"
        envoyer_angles(ser, 0,0,0,0,0,0,0)
        time.sleep(0.8)
        Move_ReverseLineaire("DoneLine")

    #if result:
        #return result[0], result[1], result[2]
    return



def Move_lineaire(LastMessage):


    global directive, turn
    turn = 0
    Parts = [None]
    MemoryMessage = None
    if LastMessage == "DoneJoint":
           
        directive = "Lineaire"


        bras_robot.Calculate(1, 0.0, 0.0, 0.0, 0)


        J1 = bras_robot.angles[0]
        J2 = bras_robot.angles[1]
        J3 = bras_robot.angles[2]


        bras_robot.Calculate(2, 0.0, 0.0, 0.0, 0)


        PickJ1 = bras_robot.angles[0]
        PickJ2 = bras_robot.angles[1]
        PickJ3 = bras_robot.angles[2]


        bras_robot.Calculate(0, J1, J2, J3, 0)


        V1 = bras_robot.vitesse[0]
        V2 = bras_robot.vitesse[1]
        V3 = bras_robot.vitesse[2]


        envoyer_angles(ser, PickJ1, V1, -PickJ2, V2, -PickJ3, V3, 0)


   
        while Parts[0] != "Doneline":
            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)


                if MemoryMessage != None:
                    Parts = MemoryMessage.split()
                    if len(Parts) != 4:
                        MemoryMessage = None
                        continue


            bras_robot.Calculate(0, float(Parts[1]), float(Parts[2]), float(Parts[3]), 0)


            V1 = bras_robot.vitesse[0]
            V2 = bras_robot.vitesse[1]
            V3 = bras_robot.vitesse[2]


            envoyer_angles(ser, PickJ1, V1, -PickJ2, V2, -PickJ3, V3, 0)


            MemoryMessage = None

def Move_ReverseLineaire(LastMessage):
 
    global directive, turn
    turn = 1
    Parts = [None]
    MemoryMessage = None
    if LastMessage == "DoneLine":
           
        directive = "LineaireReverse"
 
        bras_robot.Calculate(2, 0.0, 0.0, 0.0, 0)
 
        J1 = bras_robot.angles[0]
        J2 = bras_robot.angles[1]
        J3 = bras_robot.angles[2]
 
        bras_robot.Calculate(1, 0.0, 0.0, 0.0, 0)
 
        ReachJ1 = bras_robot.angles[0]
        ReachJ2 = bras_robot.angles[1]
        ReachJ3 = bras_robot.angles[2]
 
        bras_robot.Calculate(0, J1, J2, J3, turn)
 
        V1 = bras_robot.vitesse[0]
        V2 = bras_robot.vitesse[1]
        V3 = bras_robot.vitesse[2]
 
        envoyer_angles(ser, ReachJ1, V1, -ReachJ2, V2, -ReachJ3, V3, 0)
 

        while Parts[0] != "DoneLineReverse":
            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)
 
                if MemoryMessage != None:
                    Parts = MemoryMessage.split()
                    if len(Parts) != 1:
                        MemoryMessage = None
                        continue
 
            bras_robot.Calculate(0, float(Parts[1]), float(Parts[2]), float(Parts[3]), turn)
 
            V1 = bras_robot.vitesse[0]
            V2 = bras_robot.vitesse[1]
            V3 = bras_robot.vitesse[2]
 
            envoyer_angles(ser, ReachJ1, V1, -ReachJ2, V2, -ReachJ3, V3, 0)
 
            MemoryMessage = None

# MAIN


COULEUR_MAP = {"rouge": rouge, "bleu": bleu, "jaune": jaune} #à définir

ser = connect_serial()
if ser is None:
    raise SystemExit("Port série indisponible.")

fA1 = fA2 = fA3 = 0.0

try:
    while True:

        # 1. Aller à la position scan
        print("\n--- Position scan ---")
        fA1, fA2, fA3, _ = aller_a(ser, X_SCAN, Y_SCAN, Z_SCAN, fA1, fA2, fA3)

        # 2. Détecter les pilules avec la caméra
        print("--- Détection caméra ---")
        points_bruts = detecter_pilules()

        # TODO : adapter selon dict camera
        
        points = []
        for p in points_bruts["pilules"]:
            points.append((
                p["x"],
                p["y"],
                p["angle"],
                p["couleur"]
            ))

##      Why?
#        if not points:
#            print("Aucune pilule détectée")
#            time.sleep(2)
#            continue
##
        print(f"{len(points)} pilule(s) détectée(s) :")
        for p in points:
            print(f"  x={p[0]:.3f} y={p[1]:.3f} angle={p[2]:.1f}° couleur={p[3]}")

        # 3. Générer la trajectoire

        traj = generate_trajectory(points)
        print(f"  {len(traj)} points de trajectoire")

        # 4. Exécuter chaque point
        for i, pt in enumerate(traj):
            print(f"  Point {i+1}/{len(traj)} | mvt={int(pt[4])} pince={int(pt[5])} couleur={int(pt[6])}")
            #fA1, fA2, fA3 = executer_point(ser, pt, fA1, fA2, fA3)
            executer_point(ser, pt, fA1, fA2, fA3)

        

except KeyboardInterrupt:
    print("\nArrêt demandé.")
finally:
    ser.close()
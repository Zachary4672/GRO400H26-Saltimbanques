import cv2
import serial
import time
import threading
from pathlib import Path

from torch import mv
import donnees
import bras_robot
import affichage
import sys
import json
sys.path.append(str(Path(__file__).parents[2]))
import integration
import ComHMI

PORT  = "/dev/ttyACM0"  # à adapter selon votre système
BAUD  = 115200
#Lock
serial_lock = threading.Lock()

# Position scan
X_SCAN = donnees.Donnees.x_scan
Y_SCAN = donnees.Donnees.y_scan
Z_SCAN = donnees.Donnees.z_scan
X_INTER1 = donnees.Donnees.x_inter1
Y_INTER1 = donnees.Donnees.y_inter1
Z_INTER1 = donnees.Donnees.z_inter1
Z_PICK = donnees.Donnees.z_pick
LastMessage = None
StartRobot = False  #Variable utilisé par le HMI
StopMoteur = False  #Variable utilisé par le HMI
turn = 0
directive = "Joint"

# Initialisation de la caméra et des threads
integration.init_integration()
time.sleep(2)  # Attente pour stabiliser la caméra
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

    filename = Path(__file__).resolve().parents[2] / "donnees.json"
    
    if filename.exists():
        filename.unlink()  # Supprimer le fichier existant

    data = {
        "x" : x_pos,
        "y" : y_pos,
        "z" : z_pos
    }
    with open(filename, "w") as f:
        json.dump(data, f, indent=4)
    


#Fonction: envoyer_angles(ser, a1, b1, a2, b2, a3, b3): Envoie des angles à l'arduino pour un prochain mouvement
def envoyer_angles(ser, a1, b1, a2, b2, a3, b3, a4):
    global turn, directive
    if directive == "Joint":
        message = "#"+ directive + f",{a1:.4f}~{a2:.4f} {a3:.4f}/{a4:.4f}*\n"
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


# Aller à une position cartésienne (Joint)


def aller_a(ser, x, y, z, fA1=0.0, fA2=0.0, fA3=0.0):
    global directive

    writingJSON(x, y, z)

    bras_robot.Calculate(2, fA1, fA2, fA3, 0)

    
    j1, j2, j3, angle = bras_robot.angles

    ComHMI.PublishMessage("J1_angle", f"{j1:.4f}")
    ComHMI.PublishMessage("J2_angle", f"{j2:.4f}")
    ComHMI.PublishMessage("J3_angle", f"{j3:.4f}")
    ComHMI.PublishMessage("J1_speed", f"{0:.4f}")
    ComHMI.PublishMessage("J2_speed", f"{0:.4f}")
    ComHMI.PublishMessage("J3_speed", f"{0:.4f}")
    with serial_lock:
        directive = "Joint"
        envoyer_angles(ser, j1,0, -j2,0, -j3,0, 90)

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

    return result[0], result[1], result[2], angle

def detecter_pilules(pilules):
    # integration.display_cam()
    # Simuler la détection de pilules
    pilules = integration.scan_cam() 
    if pilules is None:
        return []
    taille = len(pilules)
    _ = 0
    for _ in range(taille):  # nombre de frames   
        pilules[_]["x"], pilules[_]["y"] = bras_robot.CalculateCamera(X_SCAN,Y_SCAN,pilules[_]["x"], pilules[_]["y"])
        
    return pilules


# Exécuter un point de trajectoire

def executer_point(ser, pt, fA1, fA2, fA3):
    x, y, z, angle, type_mvt, pince, couleur = pt
    global directive
    writingJSON(x, y, z)
    print("X:"+ str(x)+"Y: "+str(y)+" Z: "+str(z))
    bras_robot.Calculate(1, fA1, fA2, fA3, 1)
    if bras_robot.skip:
        print("Point hors de portée, passage au point suivant")
        return

    j1, j2, j3, j4 = bras_robot.angles
    j4 = angle
    ComHMI.PublishMessage("J1_angle", f"{j1:.4f}")
    ComHMI.PublishMessage("J2_angle", f"{j2:.4f}")
    ComHMI.PublishMessage("J3_angle", f"{j3:.4f}")
    if type_mvt == 0:       
        # Mouvement Joint
        with serial_lock:
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

        #Mouvement linéaire
        Move_lineaire("DoneJoint")


    if pince == 1:
        #Fermer la pince
        with serial_lock:
            directive = "Pince1"
            envoyer_angles(ser, 0,0,0,0,0,0,0)
        time.sleep(0.8)
        Move_ReverseLineaire("DoneLine")
    else:
        #Ouvrir la pince
        with serial_lock:
            directive = "Pince0"
            envoyer_angles(ser, 0,0,0,0,0,0,0)
        time.sleep(0.8)
        Move_ReverseLineaire("DoneLine")

    return

def Verifier_MoteurStop():
    while True:

        Moteur = ComHMI.is_MotorStop()
        if Moteur:
            with serial_lock:
                global directive
                directive = "StopMoteur"
                envoyer_angles(ser, 0,0,0,0,0,0,0)



def Move_lineaire(LastMessage):


    global directive, turn
    turn = 0
    Parts = [None]
    MemoryMessage = None
    if LastMessage == "DoneJoint":
        
        with serial_lock: 
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

        ComHMI.PublishMessage("J1_angle", f"{PickJ1:.4f}")
        ComHMI.PublishMessage("J2_angle", f"{PickJ2:.4f}")
        ComHMI.PublishMessage("J3_angle", f"{PickJ3:.4f}")
        ComHMI.PublishMessage("J1_speed", abs(V1))
        ComHMI.PublishMessage("J2_speed", abs(V2))
        ComHMI.PublishMessage("J3_speed", abs(V3))
        with serial_lock:
            envoyer_angles(ser, PickJ1, V1, -  PickJ2, V2, -PickJ3, V3, 0)


   
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

            ComHMI.PublishMessage("J1_speed", abs(V1))
            ComHMI.PublishMessage("J2_speed", abs(V2))
            ComHMI.PublishMessage("J3_speed", abs(V3))

            with serial_lock:
                envoyer_angles(ser, PickJ1, V1, -PickJ2, V2, -PickJ3, V3, 0)


            MemoryMessage = None

def Move_ReverseLineaire(LastMessage):
 
    global directive, turn
    turn = 1
    Parts = [None]
    MemoryMessage = None
    if LastMessage == "DoneLine":
           
        with serial_lock:
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
 
        ComHMI.PublishMessage("J1_angle", f"{ReachJ1:.4f}")
        ComHMI.PublishMessage("J2_angle", f"{ReachJ2:.4f}")
        ComHMI.PublishMessage("J3_angle", f"{ReachJ3:.4f}")
        ComHMI.PublishMessage("J1_speed", abs(V1))
        ComHMI.PublishMessage("J2_speed", abs(V2))
        ComHMI.PublishMessage("J3_speed", abs(V3))

        with serial_lock:
            envoyer_angles(ser, ReachJ1, V1, -ReachJ2, V2, -ReachJ3, V3, 0)
 

        while Parts[0] != "DoneLineReverse":
            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)
 
                if MemoryMessage != None:
                    Parts = MemoryMessage.split()
                    if len(Parts) != 1:
                        MemoryMessage = None
                        continue
            if len(Parts) == 4:
                bras_robot.Calculate(0, float(Parts[1]), float(Parts[2]), float(Parts[3]), turn)
 
                V1 = bras_robot.vitesse[0]
                V2 = bras_robot.vitesse[1]
                V3 = bras_robot.vitesse[2]

                ComHMI.PublishMessage("J1_speed", abs(V1))
                ComHMI.PublishMessage("J2_speed", abs(V2))
                ComHMI.PublishMessage("J3_speed", abs(V3))
                with serial_lock:
                    envoyer_angles(ser, ReachJ1, V1, -ReachJ2, V2, -ReachJ3, V3, 0)
 
            MemoryMessage = None

# MAIN



ser = connect_serial()
# t_moteur = threading.Thread(target=Verifier_MoteurStop, daemon=True)
# t_moteur.start()
if ser is None:
    PORT = "/dev/ttyACM1"
    # Tentative de connexion sur un autre port
    ser = connect_serial()
    if ser is None:
        raise SystemExit("Port série indisponible.")

while True:
    fA1 = fA2 = fA3 = 0.0
    try:
        #Call ici pour setup StartRobot à true
        ComHMI.PublishMessage("start", 0)
        time.sleep(0.5)
        StartRobot = ComHMI.is_started()
        time.sleep(0.5)

        if StartRobot == -1:
            raise Exception("Connection perdue avec le HMI, arrêt du robot")
        ComHMI.PublishMessage("Robot_state", "WAITING")
        while StartRobot:

            
            # 1. Aller à la position scan
            print("\n--- Position scan ---")
            ComHMI.PublishMessage("Robot_state", "SCANNING")
            fA1, fA2, fA3, _ = aller_a(ser, X_SCAN, Y_SCAN, Z_SCAN, fA1, fA2, fA3)
            time.sleep(2) # Attente pour stabiliser la caméra
            # 2. Détecter les pilules avec la caméra
            print("--- Détection caméra ---")

            points = [] 
            points = detecter_pilules(points)
            
            if len(points) == 0:
                print("Aucune pilule détectée, en attente de détection")
                while not points:
                    points = []
                    points = detecter_pilules(points)


            if len(points) != 0:
                ComHMI.PublishMessage("Robot_state", "PICK")
                len_points = len(points)
                pil = 0
                for pil in range(len_points):
                    pt = (points[pil]["x"], points[pil]["y"], Z_PICK, points[pil]["angle"], 0, 1, points[pil]["color"])
                    executer_point(ser, pt, fA1, fA2, fA3)
                    if bras_robot.skip:
                        bras_robot.skip = False
                        x_next = X_SCAN
                        y_next = Y_SCAN
                        z_next = Z_SCAN
                    
                    elif points[pil]["color"] == "rouge":
                        col = "Red"
                        x_next = donnees.Donnees.x_r
                        y_next = donnees.Donnees.y_r
                        z_next = donnees.Donnees.z_drop
                    elif points[pil]["color"] == "jaune":
                        col = "Yellow"
                        x_next = donnees.Donnees.x_b
                        y_next = donnees.Donnees.y_b
                        z_next = donnees.Donnees.z_drop
                    else:
                        col = "Other"
                        x_next = donnees.Donnees.x_j
                        y_next = donnees.Donnees.y_j
                        z_next = donnees.Donnees.z_drop
                    pt = (x_next, y_next, z_next, 90, 0, 0, points[pil]["color"])
                    ComHMI.PublishMessage("Robot_state", "DROP")
                    executer_point(ser, pt, fA1, fA2, fA3)
                    ComHMI.PublishMessage("Item_dropped", col+" 1")
                    if pil < len_points-1:
                        aller_a(ser, X_INTER1, Y_INTER1, Z_INTER1, fA1, fA2, fA3)
                    time.sleep(2)
                    #Ici if call fonction = true, then StartRobot = False
            if ComHMI.is_stopped():
                StartRobot = False

            
    except (serial.SerialException,OSError) as e:
        print(f"Connection série perdue : {e}")
        try:
            ser.close()
        except Exception:
            pass
        ser = None
        while ser is None:
            ser = connect_serial()
            time.sleep(1)
    except Exception as e:
        while StartRobot == -1:
            print(f"Connection perdue avec le HMI : {e}")
            StartRobot = ComHMI.is_started()
            time.sleep(3)

ser.close()
integration.close_camera()
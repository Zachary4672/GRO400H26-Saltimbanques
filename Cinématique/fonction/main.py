#Import des fichiers nécessaires
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
from Camera import integration
import ComHMI

PORT  = "/dev/ttyACM0"  # à adapter selon votre système
BAUD  = 115200


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

# ----------------------------------------
# Fonction majeure: connect_serial(): permet de connecter le RaspberryPi à l'OpenRB
# ----------------------------------------
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
    
# ----------------------------------------
# Fonction mineure: writingJSON(x,y,z): permet d'écrire un JSON contenant la prochaine position xyz du robot
# ----------------------------------------
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
    


# ----------------------------------------
# Fonction mineure: envoyer_angles(ser, a1, b1, a2, b2, a3, b3, a4): permet d'envoyer un message à l'OpenRB selon la valeur de la variable directive
# ----------------------------------------
def envoyer_angles(ser, a1, b1, a2, b2, a3, b3, a4):
    global turn, directive
    if directive == "Joint": #Envoie des valeurs pour un déplacement en joint sous la forme: directive, J1, J2, J3
        message = "#"+ directive + f",{a1:.4f}~{a2:.4f} {a3:.4f}/{a4:.4f}*\n" #exemple: "#Joint,0.25~0.25 0.25/0.25*"
    elif directive == "Lineaire" and turn == 0: #Envoie des valeurs pour un déplacement linéaire sous la forme: directive, J1, V1, J2, V2, J3, V3
        message = "#" + directive + f",{a1:.4f}~{b1:.4f} {a2:.4f}_{b2:.4f}&{a3:.4f}!{b3:.4f}*\n" #exemple: "#Lineaire,0.25~1 0.25_1&0.25!1*"
    elif directive == "LineaireReverse": #Envoie des valeurs pour un déplacement linéaire-inverse sous la forme: directive, J1, V1, J2, V2, J3, V3
        message = "#" + "LineaireReverse" + f",{a1:.4f}~{b1:.4f} {a2:.4f}_{b2:.4f}&{a3:.4f}!{b3:.4f}*\n" #exemple: "#LineaireReverse,0.25~1 0.25_1&0.25!1*"
    elif directive == "Pince1": #Envoie la demande de fermer la pince
        message = f"#Pince,1*\n" 
    elif directive == "Pince0": #Envoie la demande d'ouvrir la pince
        message = f"#Pince,0*\n"
    ser.write(message.encode("utf-8"))
                

# ----------------------------------------
# Fonction mineure: lire_latest_ligne_complete(ser): permet de lire le dernier message envoyé depuis l'OpenRB
# ----------------------------------------
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


# ----------------------------------------
# Fonction majeure: aller_a(ser, x, y, z): permet un déplacement en joint seulement.
# Utilisé pour le déplacement à la position de scan ou au point intermédiaire entre 2 pilules
# ----------------------------------------
def aller_a(ser, x, y, z, fA1=0.0, fA2=0.0, fA3=0.0):
    global directive #Directive pour le type de déplacement

    writingJSON(x, y, z) #Écrire le point d'arrivé voulu

    bras_robot.Calculate(2, fA1, fA2, fA3, 0) #calculer avec le JSON les angles

    
    j1, j2, j3, angle = bras_robot.angles

    ComHMI.PublishMessage("J1_angle", f"{j1:.4f}") #Envoyer l'angle J1 au HMI
    ComHMI.PublishMessage("J2_angle", f"{j2:.4f}") #Envoyer l'angle J2 au HMI
    ComHMI.PublishMessage("J3_angle", f"{j3:.4f}") #Envoyer l'angle J3 au HMI
    ComHMI.PublishMessage("J1_speed", f"{0:.4f}") #Envoyer la vitesse de J1 au HMI
    ComHMI.PublishMessage("J2_speed", f"{0:.4f}") #Envoyer la vitesse de J2 au HMI
    ComHMI.PublishMessage("J3_speed", f"{0:.4f}") #Envoyer la vitesse de J3 au HMI

    directive = "Joint"
    envoyer_angles(ser, j1,0, -j2,0, -j3,0, 90)

    #Cette partie attend la confirmation que le mouvement est terminé de la part du OpenRB
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

# ----------------------------------------
# Fonction majeure: detecter_pilules(pilules): Permet de détecter les pilules et de produire un dictionnaire de tout les pilules détectées
# ----------------------------------------
def detecter_pilules(pilules):
    pilules = integration.scan_cam() # Obtentions du dictionnaire
    if pilules is None:
        return [] #On retourne un dictionnaire vide si aucune pilule est détectée
    taille = len(pilules)
    _ = 0
    for _ in range(taille):  # nombre de frames   
        pilules[_]["x"], pilules[_]["y"] = bras_robot.CalculateCamera(X_SCAN,Y_SCAN,pilules[_]["x"], pilules[_]["y"]) #On retourne la valeur x,y corrigée pour que que le x et y soit selon le robot et non selon la caméra
        
    return pilules

# ----------------------------------------
# Fonction majeure: executer_points(ser, pt): Permet d'excécuter la séquence suivante: 1. Mouvement en joint jusqu'à l'approche, 2. Mouvement linéaire pour descendre jusqu'au point, 3. ouvrir ou fermer la pince, 4. Mouvement linéaire inverse pour revenir à l'approche
# ----------------------------------------
def executer_point(ser, pt, fA1, fA2, fA3):
    x, y, z, angle, type_mvt, pince, couleur = pt
    global directive
    writingJSON(x, y, z) #Écriture du JSON pour la fonction de calcul
    print("X:"+ str(x)+"Y: "+str(y)+" Z: "+str(z)) #Impression du point envoyé dans le JSON pour debug
    bras_robot.Calculate(1, fA1, fA2, fA3, 1) #Calcul avec une approche.
    if bras_robot.skip: #Si le point est impossible à atteindre
        print("Point hors de portée, passage au point suivant")
        return

    j1, j2, j3, j4 = bras_robot.angles
    j4 = angle
    ComHMI.PublishMessage("J1_angle", f"{j1:.4f}") #Envoie au HMI de l'angle J1
    ComHMI.PublishMessage("J2_angle", f"{j2:.4f}") #Envoie au HMI de l'angle J2
    ComHMI.PublishMessage("J3_angle", f"{j3:.4f}") #Envoie au HMI de l'angle J3
    if type_mvt == 0:       
        # Mouvement Joint
        directive = "Joint"
        envoyer_angles(ser, j1, 0, -j2, 0, -j3, 0, j4)

        #Attente de la confirmation du mouvement en Joint terminé
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
        directive = "Pince1"
        envoyer_angles(ser, 0,0,0,0,0,0,0) #Envoie de la demande de fermer la pince à l'OpenRB
        time.sleep(0.8)
        Move_ReverseLineaire("DoneLine") #Début de la remonté à l'approche
    else:
        #Ouvrir la pince
        directive = "Pince0"
        envoyer_angles(ser, 0,0,0,0,0,0,0) #Envoie de la demande d'ouvrir la pince à l'OpenRB
        time.sleep(0.8)
        Move_ReverseLineaire("DoneLine") #Début de la remonté à l'approche
    return


# ----------------------------------------
# Fonction majeure: Move_lineaire(LastMessage): Cette fonction applique une communication constante avec l'OpenRB pour modifier graduellement la vitesse des moteurs pour maintenir un déplacement linéaire.
# ----------------------------------------
def Move_lineaire(LastMessage):


    global directive, turn
    turn = 0
    Parts = [None]
    MemoryMessage = None
    if LastMessage == "DoneJoint":
        
        #Nouvelle directive
        directive = "Lineaire"

        #Calcul du point de départ en joint
        bras_robot.Calculate(1, 0.0, 0.0, 0.0, 0)


        J1 = bras_robot.angles[0]
        J2 = bras_robot.angles[1]
        J3 = bras_robot.angles[2]

        #Calcul du point d'arrivé en joint
        bras_robot.Calculate(2, 0.0, 0.0, 0.0, 0)


        PickJ1 = bras_robot.angles[0]
        PickJ2 = bras_robot.angles[1]
        PickJ3 = bras_robot.angles[2]


        bras_robot.Calculate(0, J1, J2, J3, 0)

        #Calcul de la première vitesse envoyé à l'OpenRB
        V1 = bras_robot.vitesse[0]
        V2 = bras_robot.vitesse[1]
        V3 = bras_robot.vitesse[2]

        ComHMI.PublishMessage("J1_angle", f"{PickJ1:.4f}") #Envoie de l'angle J1 au HMI
        ComHMI.PublishMessage("J2_angle", f"{PickJ2:.4f}") #Envoie de l'angle J2 au HMI
        ComHMI.PublishMessage("J3_angle", f"{PickJ3:.4f}") #Envoie de l'angle J3 au HMI
        ComHMI.PublishMessage("J1_speed", abs(V1)) #Envoie de la vitesse du J1 au HMI
        ComHMI.PublishMessage("J2_speed", abs(V2)) #Envoie de la vitesse du J2 au HMI
        ComHMI.PublishMessage("J3_speed", abs(V3)) #Envoie de la vitesse du J3 au HMI
        
        envoyer_angles(ser, PickJ1, V1, -  PickJ2, V2, -PickJ3, V3, 0) #Envoie des valeurs calculées précédemment à l'OpenRB


       #Tant et aussi longtemps que le linéaire est pas terminé fait ceci:
        while Parts[0] != "Doneline":
            #Boucle de lecture qui attend de recevoir un message de l'OpenRB pour confirmer le calcul d'une nouvelle vitesse ou que le linéaire est terminé
            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)


                if MemoryMessage != None:
                    Parts = MemoryMessage.split()
                    if len(Parts) != 4:
                        MemoryMessage = None
                        continue

            #Calcul des nouvelles vitesses
            bras_robot.Calculate(0, float(Parts[1]), float(Parts[2]), float(Parts[3]), 0)


            V1 = bras_robot.vitesse[0]
            V2 = bras_robot.vitesse[1]
            V3 = bras_robot.vitesse[2]

            ComHMI.PublishMessage("J1_speed", abs(V1)) #Envoie des nouvelles vitesses au HMI
            ComHMI.PublishMessage("J2_speed", abs(V2)) #Envoie des nouvelles vitesses au HMI
            ComHMI.PublishMessage("J3_speed", abs(V3)) #Envoie des nouvelles vitesses au HMI

            envoyer_angles(ser, PickJ1, V1, -PickJ2, V2, -PickJ3, V3, 0) #Envoyer les valeurs à l'OpenRB


            MemoryMessage = None
            
# ----------------------------------------
# Fonction majeure: Move_ReverseLineaire(LastMessage): Cette fonction applique une communication constante avec l'OpenRB pour modifier graduellement la vitesse des moteurs pour maintenir un déplacement linéaire.
# ----------------------------------------
def Move_ReverseLineaire(LastMessage):
 
    global directive, turn
    turn = 1
    Parts = [None]
    MemoryMessage = None
    if LastMessage == "DoneLine":
           
        #Nouvelle directive
        directive = "LineaireReverse"

        #Calcul du point de départ du robot
        bras_robot.Calculate(2, 0.0, 0.0, 0.0, 0)
 
        J1 = bras_robot.angles[0]
        J2 = bras_robot.angles[1]
        J3 = bras_robot.angles[2]

        #Calcul du point d'arrivé du robot
        bras_robot.Calculate(1, 0.0, 0.0, 0.0, 0)
 
        ReachJ1 = bras_robot.angles[0]
        ReachJ2 = bras_robot.angles[1]
        ReachJ3 = bras_robot.angles[2]

        #Calcul de la vitesse de départ du robot
        bras_robot.Calculate(0, J1, J2, J3, turn)
 
        V1 = bras_robot.vitesse[0]
        V2 = bras_robot.vitesse[1]
        V3 = bras_robot.vitesse[2]
 
        ComHMI.PublishMessage("J1_angle", f"{ReachJ1:.4f}") #Envoie de l'angle J1 au HMI
        ComHMI.PublishMessage("J2_angle", f"{ReachJ2:.4f}") #Envoie de l'angle J2 au HMI
        ComHMI.PublishMessage("J3_angle", f"{ReachJ3:.4f}") #Envoie de l'angle J3 au HMI
        ComHMI.PublishMessage("J1_speed", abs(V1)) #Envoie de la vitesse J1 au HMI
        ComHMI.PublishMessage("J2_speed", abs(V2)) #Envoie de la vitesse J2 au HMI
        ComHMI.PublishMessage("J3_speed", abs(V3)) #Envoie de la vitesse J3 au HMI

        envoyer_angles(ser, ReachJ1, V1, -ReachJ2, V2, -ReachJ3, V3, 0) #Envoie des angles et de la vitesse à l'OpenRB
 

        #Tant et aussi longtemps que le linéaire est pas terminé faire ceci:
        while Parts[0] != "DoneLineReverse":
            #Attend la réception d'un message de l'OpenRB pour calculer une nouvelle vitesse ou sortir de la boucle
            while MemoryMessage == None:
                MemoryMessage = lire_latest_ligne_complete(ser)
 
                if MemoryMessage != None:
                    Parts = MemoryMessage.split()
                    if len(Parts) != 1:
                        MemoryMessage = None
                        continue
            if len(Parts) == 4:

                #Calcul des nouvelles vitesses
                bras_robot.Calculate(0, float(Parts[1]), float(Parts[2]), float(Parts[3]), turn)
 
                V1 = bras_robot.vitesse[0]
                V2 = bras_robot.vitesse[1]
                V3 = bras_robot.vitesse[2]

                ComHMI.PublishMessage("J1_speed", abs(V1)) #Envoie de la vitesse du J1 au HMI
                ComHMI.PublishMessage("J2_speed", abs(V2)) #Envoie de la vitesse du J2 au HMI
                ComHMI.PublishMessage("J3_speed", abs(V3)) #Envoie de la vitesse du J3 au HMI

                envoyer_angles(ser, ReachJ1, V1, -ReachJ2, V2, -ReachJ3, V3, 0) #Envoie des valeurs à l'OpenRB
 
            MemoryMessage = None



# MAIN
ser = connect_serial()
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
        ComHMI.PublishMessage("start", 0) #Set de la valeur à 0
        time.sleep(0.5)
        StartRobot = ComHMI.is_started() #Réception de la valeur provenant du HMI
        time.sleep(0.5)

        if StartRobot == -1:
            raise Exception("Connection perdue avec le HMI, arrêt du robot")
        ComHMI.PublishMessage("Robot_state", "WAITING") #Envoie de l'état du robot au HMI
        while StartRobot:

            
            # 1. Aller à la position scan
            print("\n--- Position scan ---")
            ComHMI.PublishMessage("Robot_state", "SCANNING") #Envoie de l'état du robot au HMI
            
            fA1, fA2, fA3, _ = aller_a(ser, X_SCAN, Y_SCAN, Z_SCAN, fA1, fA2, fA3) 
            time.sleep(2) # Attente pour stabiliser la caméra
            # 2. Détecter les pilules avec la caméra
            print("--- Détection caméra ---")

            points = [] 
            points = detecter_pilules(points)

            #Si aucune pilule est détectée
            if len(points) == 0:
                print("Aucune pilule détectée, en attente de détection")
                while not points:
                    points = []
                    points = detecter_pilules(points)

            #Si au moins une pilule est détectée
            if len(points) != 0:
                ComHMI.PublishMessage("Robot_state", "PICK") #Envoie d'un nouvel état du robot au HMI
                len_points = len(points)
                pil = 0
                for pil in range(len_points): #Pour chaque pilule fait:
                    pt = (points[pil]["x"], points[pil]["y"], Z_PICK, points[pil]["angle"], 0, 1, points[pil]["color"])
                    executer_point(ser, pt, fA1, fA2, fA3)
                    if bras_robot.skip: #Si la position n'est pas atteignable
                        bras_robot.skip = False
                        x_next = X_SCAN
                        y_next = Y_SCAN
                        z_next = Z_SCAN
                    
                    elif points[pil]["color"] == "rouge": #Si la pilule est rouge
                        col = "Red"
                        x_next = donnees.Donnees.x_r
                        y_next = donnees.Donnees.y_r
                        z_next = donnees.Donnees.z_drop
                    elif points[pil]["color"] == "jaune": #Si la pilule est jaune
                        col = "Yellow"
                        x_next = donnees.Donnees.x_b
                        y_next = donnees.Donnees.y_b
                        z_next = donnees.Donnees.z_drop
                    else: #Pour toute autres couleurs
                        col = "Other"
                        x_next = donnees.Donnees.x_j
                        y_next = donnees.Donnees.y_j
                        z_next = donnees.Donnees.z_drop
                    pt = (x_next, y_next, z_next, 90, 0, 0, points[pil]["color"])
                    ComHMI.PublishMessage("Robot_state", "DROP") #Envoie du nouvel état du robot au HMI
                    executer_point(ser, pt, fA1, fA2, fA3)
                    ComHMI.PublishMessage("Item_dropped", col+" 1") #Envoie de la confirmation que la pilule est tombée dans le conteneur
                    if pil < len_points-1:
                        aller_a(ser, X_INTER1, Y_INTER1, Z_INTER1, fA1, fA2, fA3) #Déplacement au point intermédiaire
                    time.sleep(2)
            #Ici if call fonction = true, then StartRobot = False
            if ComHMI.is_stopped():
                StartRobot = False

            
    except (serial.SerialException,OSError) as e: #Si la connection à l'OpenRB est perdue
        print(f"Connection série perdue : {e}")
        try:
            ser.close()
        except Exception:
            pass
        ser = None
        while ser is None:
            ser = connect_serial()
            time.sleep(1)
    except Exception as e: #Si le bouton start renvoie -1
        while StartRobot == -1:
            print(f"Connection perdue avec le HMI : {e}")
            StartRobot = ComHMI.is_started()
            time.sleep(3)

ser.close()
integration.close_camera()

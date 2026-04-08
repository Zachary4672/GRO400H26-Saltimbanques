import serial
import time
import donnees
import bras_robot
from pick_and_place import generate_trajectory, Rouge, Bleu, Jaune

PORT  = "dev/ttyUSB0"  # à adapter selon votre système
BAUD  = 115200


# Position scan
X_SCAN = donnees.Donnees.x_scan
Y_SCAN = donnees.Donnees.y_scan
Z_SCAN = donnees.Donnees.z_scan


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
    

def envoyer_joint(ser, j1, j2, j3, j4=0):
    msg = f"#Joint,{j1:.4f}~{j2:.4f} {j3:.4f} {j4:.4f}*\n"
    ser.write(msg.encode("utf-8"))


def envoyer_lineaire(ser, j1, v1, j2, v2, j3, v3, j4=0):
    msg = f"#Lineaire,{j1:.4f}~{v1:.4f} {j2:.4f}_{v2:.4f}&{j3:.4f}!{v3:.4f}*\n"
    ser.write(msg.encode("utf-8"))


def envoyer_reverse(ser, j1, v1, j2, v2, j3, v3, j4=0):
    msg = f"#LineaireReverse,{j1:.4f}~{v1:.4f} {j2:.4f}_{v2:.4f}&{j3:.4f}!{v3:.4f}*\n"
    ser.write(msg.encode("utf-8"))


def envoyer_pince(ser, fermer=True):
    msg = b"#Pince,1*\n" if fermer else b"#Pince,0*\n"
    ser.write(msg)



def attendre_doneline(ser, timeout=15):
    debut = time.time()
    done_recu = False
    while time.time() - debut < timeout:
        ligne = ser.readline().decode("utf-8", errors="replace").strip()
        if not ligne:
            continue
        if ligne.startswith("Working"):
            continue
        if ligne.startswith("Doneline"):
            done_recu = True
            continue
        if ligne.startswitth("Donejoint"):
            done_recu = True
            continue
        if done_recu:
            parts = ligne.split()
            if len(parts) >= 4:
                return float(parts[1]), float(parts[2]), float(parts[3])
    print("Doneline non recu")
    return None


# Aller à une position cartésienne (Joint)


def aller_a(ser, x, y, z, fA1=0.0, fA2=0.0, fA3=0.0):
    donnees.Donnees.x_cible = x
    donnees.Donnees.y_cible = y
    donnees.Donnees.z_cible = z
    bras_robot.Calculate(1, fA1, fA2, fA3)
    j1, j2, j3, angle = bras_robot.angles

    envoyer_joint(ser, j1, -j2, -j3, angle)
    result = attendre_doneline(ser) #!!!
    if result:
        print(f"  Position atteinte : j1={result[0]:.3f} j2={result[1]:.3f} j3={result[2]:.3f}")
        return result[0], result[1], result[2], angle
    return j1, j2, j3, angle

def detecter_pilules():
    # Simuler la détection de pilules
    # TODO : remplacer par la vraie détection caméra
    return {
        "pilules": [
            {"x": 0.1, "y": 0.1, "angle": 0, "couleur": "rouge"},
            {"x": 0.15, "y": 0.1, "angle": 45, "couleur": "bleu"},
            {"x": 0.2, "y": 0.1, "angle": -30, "couleur": "jaune"},
        ]   
    }


# Exécuter un point de trajectoire

def executer_point(ser, pt, fA1, fA2, fA3):
    x, y, z, angle, type_mvt, pince, couleur = pt

    donnees.Donnees.x_cible     = x
    donnees.Donnees.y_cible     = y
    donnees.Donnees.z_cible     = z
    donnees.Donnees.angle_cible = angle
    bras_robot.Calculate(1, fA1, fA2, fA3)
    j1, j2, j3, j4 = bras_robot.angles

    if type_mvt == 0:       # Joint
        envoyer_joint(ser, j1, j2, j3, j4)
        result = attendre_doneline(ser)

    elif type_mvt == 1:     # Lineaire
        v1 = v2 = v3 = 0.5
        envoyer_lineaire(ser, j1, v1, j2, v2, j3, v3, j4)
        result = attendre_doneline(ser)

    else:                   # Reverse
        v1 = v2 = v3 = 0.5
        envoyer_reverse(ser, j1, v1, j2, v2, j3, v3, j4)
        result = attendre_doneline(ser)

    if pince == 1:
        print(f"  → FERMER pince")
        envoyer_pince(ser, fermer=True)
        time.sleep(0.8)
    else:
        print(f"  → OUVRIR pince")
        envoyer_pince(ser, fermer=False)
        time.sleep(0.8)

    if result:
        return result[0], result[1], result[2]
    return j1, j2, j3


# MAIN


COULEUR_MAP = {"rouge": Rouge, "bleu": Bleu, "jaune": Jaune} #à définir

ser = connect_serial()
if ser is None:
    raise SystemExit("Port série indisponible.")

fA1 = fA2 = fA3 = 0.0

try:
    while True:
        # 1. Aller à la position scan
        print("\n--- Position scan ---")
        fA1, fA2, fA3, _ = aller_a(ser, X_SCAN, Y_SCAN, Z_SCAN, fA1, fA2, fA3)
        time.sleep(0.5)

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
                COULEUR_MAP[p["couleur"]]
            ))

        if not points:
            print("Aucune pilule détectée")
            time.sleep(2)
            continue

        print(f"{len(points)} pilule(s) détectée(s) :")
        for p in points:
            print(f"  x={p[0]:.3f} y={p[1]:.3f} angle={p[2]:.1f}° couleur={p[3]}")

        # 3. Générer la trajectoire

        traj = generate_trajectory(points)
        print(f"  {len(traj)} points de trajectoire")

        # 4. Exécuter chaque point
        for i, pt in enumerate(traj):
            print(f"  Point {i+1}/{len(traj)} | mvt={int(pt[4])} pince={int(pt[5])} couleur={int(pt[6])}")
            fA1, fA2, fA3 = executer_point(ser, pt, fA1, fA2, fA3)

        time.sleep(1)

except KeyboardInterrupt:
    print("\nArrêt demandé.")
finally:
    ser.close()
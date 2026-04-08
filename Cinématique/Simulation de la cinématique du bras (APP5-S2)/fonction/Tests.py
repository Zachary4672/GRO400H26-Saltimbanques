from bras_robot import fk_xyz
import bras_robot
import donnees
import json

def writingJSON(x_pos, y_pos, z_pos):
    filename = "donnees.json"
    data = {
        "x" : x_pos,
        "y" : y_pos,
        "z" : z_pos
    }
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)

def envoyer_pince(ser, fermer=True):
    msg = b"#Pince,1*\n" if fermer else b"#Pince,0*\n"
    ser.write(msg.encode("utf-8"))


while True:
    # Définir la cible du bras
    #writingJSON(0, 0.2, 0)


    # Calcul cinématique inverse
    #bras_robot.Calculate(True, 0, 0, 0, 0)

    x, y, z = fk_xyz(bras_robot.p_w0, donnees.Donnees.L1, donnees.Donnees.L2, donnees.Donnees.L3, donnees.Donnees.sens_outil, (1.125), (-1.594), (1.807))
    print(f"Position de l'effecteur : x={x:.3f}, y={y:.3f}, z={z:.3f}")
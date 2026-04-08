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


while True:
    # Définir la cible du bras
    writingJSON(0, 0.2, 0)


    # Calcul cinématique inverse
    bras_robot.Calculate(True, 0, 0, 0, 0)

    print(f"Position cible : ({donnees.Donnees.x_cible}, {donnees.Donnees.y_cible}, {donnees.Donnees.z_cible})")
    print(f"Angles calculés : j1={bras_robot.angles[0]:.3f} j2={bras_robot.angles[1]:.3f} j3={bras_robot.angles[2]:.3f} j4={bras_robot.angles[3]:.3f}")
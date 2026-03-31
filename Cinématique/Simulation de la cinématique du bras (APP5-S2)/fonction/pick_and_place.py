# pick_and_place.py
import numpy as np
import bras_robot
import donnees

# Déplacement
Joint    = 0
Lineaire = 1
Reverse  = 2

# Pince
Fermee = 1
Ouvert = 0

# Couleurs
Rouge = 1
Bleu  = 2
Jaune = 3

z_pick = 0.0
z_drop = 0.15

#Positions des pots (x, y)
POTS = {
    Rouge: (donnees.Donnees.x_r, donnees.Donnees.y_r),
    Bleu:  (donnees.Donnees.x_b, donnees.Donnees.y_b),
    Jaune: (donnees.Donnees.x_j, donnees.Donnees.y_j),
}


def generate_pick(pil):
    x, y, angle, couleur = pil
    return [                                                        # list, pas np.array
        [x, y, z_pick + 0.1, angle, Joint,    Ouvert, couleur],
        [x, y, z_pick,       angle, Lineaire,  Fermee, couleur],
        [x, y, z_pick + 0.1, angle, Reverse,   Fermee, couleur],
    ]

def generate_drop(pil):
    x, y, angle, couleur = pil
    xd, yd = POTS[couleur]  # position du pot selon la couleur
    return [                                                        # list, pas np.array
        [xd, yd, z_drop + 0.1, angle, Joint,    Fermee, couleur],
        [xd, yd, z_drop,       angle, Lineaire,  Ouvert, couleur],
        [xd, yd, z_drop + 0.1, angle, Reverse,   Ouvert, couleur],
    ]

def generate_trajectory(points):
    traj = []
    for pil in points:
        traj += generate_pick(pil)
        traj += generate_drop(pil)
    return np.array(traj)

def compute_angles(traj):
    trajectoire = []
    fA1, fA2, fA3 = 0.0, 0.0, 0.0

    for pt in traj:
        x, y, z, angle, type_mvt, pince, couleur = pt
        donnees.Donnees.x_cible     = x
        donnees.Donnees.y_cible     = y
        donnees.Donnees.z_cible     = z
        donnees.Donnees.angle_cible = angle
        bras_robot.Calculate(1, fA1, fA2, fA3)  # bState=1 pour IK

        j1, j2, j3, j4 = bras_robot.angles
        trajectoire.append({
            "angles":   (j1, j2, j3, j4),
            "type_mvt": type_mvt,
            "pince":    pince,
            "couleur":  couleur,
        })

        # angles du point actuel = point de départ du prochain
        fA1, fA2, fA3 = j1, j2, j3

    return trajectoire
# TEST

#points_test = [
#    (0.10, 0.20, 45.0, Bleu),
#    (0.30, 0.15, 30.0, Rouge),
#    (0.25, 0.40, 60.0, Jaune),
#]

#traj1 = generate_trajectory(points_test)
#trajectoire = compute_angles(traj1)

# Affichage lisible
#print(f"{'X':>6} {'Y':>6} {'Z':>6} {'Angle':>6} {'Mvt':>4} {'Pince':>5} {'Couleur':>7}")
#print("-" * 48)
#for pt in traj1:
#    print(f"{pt[0]:>6.2f} {pt[1]:>6.2f} {pt[2]:>6.2f} {pt[3]:>6.1f} {pt[4]:>4.0f} {pt[5]:>5.0f} {pt[6]:>7.0f}")

#print(f"\n{'Pt':>3} {'J1(rad)':>9} {'J2(rad)':>9} {'J3(rad)':>9} {'J4(rad)':>9} {'Mvt':>4} {'Pince':>5}")
#print("-" * 56)
#for i, pt in enumerate(trajectoire):
#    j1, j2, j3, j4 = pt["angles"]
#    print(f"{i+1:>3}  {j1:>9.4f} {j2:>9.4f} {j3:>9.4f} {j4:>9.4f} {pt['type_mvt']:>4.0f} {pt['pince']:>5.0f}")

# pick_and_place.py
import numpy as np
import bras_robot
import donnees

turn = 0

# -----------------------------
# Déplacement
# -----------------------------
Joint    = 0
Lineaire = 1
Reverse  = 2

# -----------------------------
# Pince
# -----------------------------
Fermee = 1
Ouvert = 0

# -----------------------------
# Couleurs (format interne = INT)
# -----------------------------
rouge = 1
bleu  = 2
jaune = 3

# Mapping string → int
COLOR_MAP = {
    "rouge": rouge,
    "bleu": bleu,
    "jaune": jaune
}

# -----------------------------
# Paramètres
# -----------------------------
z_pick = donnees.Donnees.z_pick
z_drop = donnees.Donnees.z_drop
hauteur_boite = donnees.Donnees.h_boite

# -----------------------------
# Positions des pots
# -----------------------------
POTS = {
    rouge: (donnees.Donnees.x_r, donnees.Donnees.y_r),
    bleu:  (donnees.Donnees.x_b, donnees.Donnees.y_b),
    jaune: (donnees.Donnees.x_j, donnees.Donnees.y_j),
}

# -----------------------------
# Normalisation couleur
# -----------------------------
def normalize_couleur(couleur):
    if isinstance(couleur, str):
        couleur = COLOR_MAP.get(couleur.lower())

    if couleur not in POTS:
        raise ValueError(f"Couleur invalide: {couleur} | Options: {list(POTS.keys())}")

    return couleur

# -----------------------------
# PICK
# -----------------------------
def generate_pick(point):
    couleur = normalize_couleur(point[_]["couleur"])

    return [
        [point[_]["x"], point[_]["y"], z_pick, point[_]["angle"], Joint, Fermee, point[_]["couleur"]],
    ]

# -----------------------------
# DROP
# -----------------------------
def generate_drop(pil):
    x, y, angle, couleur = pil
    couleur = normalize_couleur(couleur)

    xd, yd = POTS[couleur]

    return [
        [xd, yd, z_drop, angle, Joint, Ouvert, couleur],
    ]

# -----------------------------
# TRAJECTOIRE
# -----------------------------
def generate_trajectory(points):
    traj = []
    len_points = len(points)
    pil = 0
    for pil in len_points:
        traj += generate_pick(points[pil])
        traj += generate_drop(points[pil])

    return np.array(traj)

# -----------------------------
# CALCUL DES ANGLES (IK)
# -----------------------------
def compute_angles(traj):
    trajectoire = []
    fA1, fA2, fA3 = 0.0, 0.0, 0.0

    for pt in traj:
        x, y, z, angle, type_mvt, pince, couleur = pt

        # Mise à jour cible
        donnees.Donnees.x_cible     = x #A CHANGER POUR LE CALL DU WRITE JSON
        donnees.Donnees.y_cible     = y
        donnees.Donnees.z_cible     = z
        donnees.Donnees.angle_cible = angle

        # IK
        bras_robot.Calculate(1, fA1, fA2, fA3, turn)

        j1, j2, j3, j4 = bras_robot.angles

        trajectoire.append({
            "angles":   (j1, j2, j3, j4),
            "type_mvt": type_mvt,
            "pince":    pince,
            "couleur":  couleur,
        })

        # Mise à jour pour continuité
        fA1, fA2, fA3 = j1, -j2, -j3

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

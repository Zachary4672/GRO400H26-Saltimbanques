# donnees.py

class Donnees:
    # longueurs (m)
    L1 = 0.195
    L2 = 0.185
    L3 = 0.05  # poignet -> bout effecteur

    # position world de la base
    wx, wy, wz = 0.0, 0.0, 0.0

    # CIBLE du bout effecteur (world)
    x_cible = 0.25
    y_cible = 0
    z_cible = 0.10

    # Orientation de l'outil: perpendiculaire au sol (plan XY)
    # -1 = pointe vers le sol (-z), +1 = pointe vers le haut (+z)
    sens_outil = -1

    # Configuration du coude
    # "bas" = elbow-down, "haut" = elbow-up
    config_coude = "bas"

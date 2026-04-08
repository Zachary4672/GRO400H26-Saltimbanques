# donnees.py

class Donnees:
    # longueurs (m)
    L0 = 0.046
    L1 = 0.155
    L2 = 0.175
    L3 = 0.145725  # poignet -> bout effecteur
    Lcam_y = 0.05
    Lcam_x = 0.05
    Lcam_z = 0

    h_boite = 0.15
    z_pick = -h_boite
    z_drop = h_boite + 0.05

    # position world de la base
    wx, wy, wz = 0.0, 0.0, 0.0

    # valeur y lors du pick
    y_pick = 0.0

    # CIBLE du bout effecteur (world)
    x_cible = 0.10
    y_cible = 0.20
    z_cible = 0.10
    Angle_cible = 1

    #Position caméra par rapport à L3
    #x_cam = 

    # Orientation de l'outil: perpendiculaire au sol (plan XY)
    # -1 = pointe vers le sol (-z), +1 = pointe vers le haut (+z)
    sens_outil = -1

    # Configuration du coude
    # "bas" = elbow-down, "haut" = elbow-up
    config_coude = "bas"

    x_scan = 1
    y_scan = 2
    z_scan = 3

   # Position des pots (x,y)

    x_r = 0.1
    y_r = 0.1
    x_b = 0.2
    y_b = 0.1
    x_j = 0.3
    y_j = 0.1
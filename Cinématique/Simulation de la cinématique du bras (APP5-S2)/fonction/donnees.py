# donnees.py

class Donnees:
    # longueurs (m)
    L0 = 0.05
    L1 = 0.195
    L2 = 0.185
    L3 = 0.10  # poignet -> bout effecteur
    Lcam_y = 0.05
    Lcam_x = 0.05
    Lcam_z = 0

    # position world de la base
    wx, wy, wz = 0.0, 0.0, 0.0

    # valeur y lors du pick
    y_pick = 0.0

    # CIBLE du bout effecteur (world)
    x_cible = 0.10
    y_cible = 0.10
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

    x_cam1 = 1
    y_cam1 = 2
    z_cam1 = 3

    x_cam2 = 4
    y_cam2 = 5
    z_cam2 = 6

    x_cam3 = 7
    y_cam3 = 8
    z_cam3 = 9

    x_home = 1
    y_home = 2
    z_home = 3

    x_r = 0.1
    y_r = 0.1
    x_b = 0.2
    y_b = 0.1
    x_j = 0.3
    y_j = 0.1
# donnees.py

class Donnees:
    # longueurs (m)
    L0 = 0.046
    L1 = 0.155
    L2 = 0.175
    L3 = 0.13442  # poignet -> bout effecteur
    Lcam_y = 0.05
    Lcam_x = 0.05
    Lcam_z = 0

    h_boite = 0.086
    z_pick = -0.075
    z_drop = 0.02
   

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
    x_cam = 0.031
    z_cam = 0.01

    # Orientation de l'outil: perpendiculaire au sol (plan XY)
    # -1 = pointe vers le sol (-z), +1 = pointe vers le haut (+z)
    sens_outil = -1

    # Configuration du coude
    # "bas" = elbow-down, "haut" = elbow-up
    config_coude = "bas"

    x_scan = 0.2
    y_scan = 0
    z_scan = 0.1
    # Position des intermédiaires
    x_inter1 = 0.2
    y_inter1 = 0.
    z_inter1 = 0.02

   # Position des pots (x,y)

    x_r = 0.050
    y_r = -0.145
    x_b = 0.00
    y_b = -0.145
    x_j = -0.057
    y_j = -0.145
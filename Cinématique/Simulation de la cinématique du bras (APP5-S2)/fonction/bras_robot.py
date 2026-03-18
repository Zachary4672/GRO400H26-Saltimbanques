# bras_robot.py
import math
import numpy as np
from donnees import Donnees

# -----------------------------
# Rotations
# -----------------------------
def Rz(theta):
    return np.array([
        [math.cos(theta), -math.sin(theta), 0.0],
        [math.sin(theta),  math.cos(theta), 0.0],
        [0.0, 0.0, 1.0]
    ])

def Ry(theta):
    return np.array([
        [ math.cos(theta), 0.0,  math.sin(theta)],
        [0.0,              1.0,  0.0],
        [-math.sin(theta), 0.0,  math.cos(theta)]
    ])

# -----------------------------
# Base et cible
# -----------------------------
p_w0 = np.array([[Donnees.wx],
                 [Donnees.wy],
                 [Donnees.wz]], dtype=float)

p_ee_cible = np.array([[Donnees.x_cible],
                       [Donnees.y_cible],
                       [Donnees.z_cible]], dtype=float)

L0, L1, L2, L3 = Donnees.L0, Donnees.L1, Donnees.L2, Donnees.L3
sens = Donnees.sens_outil

# ---------------------------------------------------------
# Point j1 -> j2 : segment fixe vertical L0
# ---------------------------------------------------------
# On suppose que L0 est un décalage vertical selon +z monde.
p_j2_base = p_w0 + np.array([[0.0],
                             [0.0],
                             [L0]])

# ---------------------------------------------------------
# IK : on vise le poignet
# ---------------------------------------------------------
# outil vertical => poignet = bout - [0,0,sens*L3]
p_e2_cible = p_ee_cible - np.array([[0.0],
                                    [0.0],
                                    [sens * L3]])

# vecteur épaule -> poignet
d = p_e2_cible - p_j2_base
dx, dy, dz = d[0, 0], d[1, 0], d[2, 0]

# 1) yaw
j1 = math.atan2(dy, dx)

# projection dans le plan après yaw
r = math.sqrt(dx * dx + dy * dy)
z_plan = dz

D = math.sqrt(r * r + z_plan * z_plan)

# atteignabilité
if D > (L1 + L2) + 1e-9 or D < abs(L1 - L2) - 1e-9:
    raise ValueError(
        f"Cible hors de l'espace atteignable : D={D:.4f}, "
        f"borne=[{abs(L1 - L2):.4f}, {L1 + L2:.4f}]"
    )

# 2) coude (loi des cosinus)
c3 = (D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
c3 = max(-1.0, min(1.0, c3))

if Donnees.config_coude.lower() == "haut":
    j3 = -math.acos(c3)
else:
    j3 = math.acos(c3)

# 3) épaule
phi  = math.atan2(-z_plan, r)
beta = math.atan2(L2 * math.sin(j3), L1 + L2 * math.cos(j3))
j2   = phi - beta

# 4) poignet : outil perpendiculaire au sol
# j2 + j3 + j4 = sens*pi/2
j4 = sens * (math.pi / 2.0) - (j2 + j3)

# ---------------------------------------------------------
# FK : positions des points
# ---------------------------------------------------------
r_aw = Rz(j1)
r_ba = Ry(j2)
r_cb = Ry(j3)
r_ec = Ry(j4)

r_bw = r_aw @ r_ba
r_cw = r_bw @ r_cb
r_ew = r_cw @ r_ec

# segment fixe L0 vertical
v_0j2_w  = np.array([[0.0],
                     [0.0],
                     [L0]])

# segments articulés
v_j2e1_b  = np.array([[L1],
                      [0.0],
                      [0.0]])

v_e1e2_c  = np.array([[L2],
                      [0.0],
                      [0.0]])

v_e2ee_e  = np.array([[-L3],
                      [0.0],
                      [0.0]])

# positions
p_j2_w = p_w0 + v_0j2_w
p_e1_w = p_j2_w + r_bw @ v_j2e1_b
p_e2_w = p_e1_w + r_cw @ v_e1e2_c
p_ee_w = p_e2_w + r_ew @ v_e2ee_e

# axe outil (x local)
x_tool_w = r_ew @ np.array([[1.0],
                            [0.0],
                            [0.0]])

angles = (j1, j2, j3, j4)

# Debug
if __name__ == "__main__":
    print("Base      =", p_w0.ravel())
    print("Joint j2  =", p_j2_w.ravel())
    print("Cible     =", p_ee_cible.ravel())
    print("Calculée  =", p_ee_w.ravel())
    print("Erreur    =", (p_ee_w - p_ee_cible).ravel())
    print("Axe outil =", x_tool_w.ravel())
    print("Angles    =", angles)



#cone de la camera
#centre
    cam0_x, cam0_y, cam0_z = Donnees.x_cible, Donnees.y_cible, 0
    cam = np.array([[cam0_x],[cam0_y],[cam0_z]])
    #contour
    fov_h = np.deg2rad(110)
    height = 480
    width = 640
    fov_v = 2 * np.arctan(np.tan(fov_h/2) * (height/width))
    R = Donnees.z_cible
    half_h = R * np.tan(fov_h/2)
    half_w = R * np.tan(fov_v/2)

    corners_local = np.array([
        [ half_w,  half_h, 0],
        [ half_w, -half_h, 0],
        [-half_w, -half_h, 0],
        [-half_w,  half_h, 0]
    ]).T
    r_cam = r_ew @ np.array([[0,0,1],[0,1,0],[1,0,0]])
    corners_world = cam + r_cam @ corners_local
    

    print("Base      =", P_w0.ravel())
    print("Cible     =", p_ee_cible.ravel())
    print("Calculée  =", P_ee_w.ravel())
    print("Erreur    =", (P_ee_w - p_ee_cible).ravel())
    print("Axe outil =", X_tool_w.ravel())
    print("Angles    =", Angles)

    #cone de la camera À AJOUTER
    #centre
    cam0_x, cam0_y, cam0_z = donnees.x_cible, donnees.y_cible, 0
    cam = np.array([[cam0_x],[cam0_y],[cam0_z]])
    #contour
    fov_h = np.deg2rad(110)
    height = 480
    width = 640
    fov_v = 2 * np.arctan(np.tan(fov_h/2) * (height/width))
    R = donnees.z_cible
    half_h = R * np.tan(fov_h/2)
    half_w = R * np.tan(fov_v/2)

    corners_local = np.array([
        [ half_w,  half_h, 0],
        [ half_w, -half_h, 0],
        [-half_w, -half_h, 0],
        [-half_w,  half_h, 0]
    ]).T
    r_cam = r_ew @ np.array([[0,0,1],[0,1,0],[1,0,0]])
    corners_world = cam + r_cam @ corners_local



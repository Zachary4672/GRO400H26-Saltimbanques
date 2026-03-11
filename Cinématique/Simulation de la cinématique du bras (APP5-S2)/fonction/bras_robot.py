# bras_robot.py
import math
import numpy as np
import importlib
import donnees


# -----------------------------
# Global variables
#------------------------------
p_w0 = 0
x_tool_w = 0
angles = 0
p_e1_w = 0
p_e2_w = 0
p_ee_w = 0
# -----------------------------
# Rotations
# -----------------------------
def Rz(theta):
    return np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta),  math.cos(theta), 0],
        [0, 0, 1]
    ])

def Ry(theta):
    return np.array([
        [ math.cos(theta), 0,  math.sin(theta)],
        [0, 1, 0],
        [-math.sin(theta), 0,  math.cos(theta)]
    ])




def Calculate():
# -----------------------------
# Base et cible
# -----------------------------
    importlib.reload(donnees)
    arrP_w0 = np.array([[donnees.Donnees.wx],
                     [donnees.Donnees.wy],
                     [donnees.Donnees.wz]])

    p_ee_cible = np.array([[donnees.Donnees.x_cible],
                           [donnees.Donnees.y_cible],
                           [donnees.Donnees.z_cible]])

    L1, L2, L3 = donnees.Donnees.L1, donnees.Donnees.L2, donnees.Donnees.L3
    sens = donnees.Donnees.sens_outil

# ---------------------------------------------------------
# IK : on vise le poignet
# ---------------------------------------------------------
# outil vertical => poignet = bout - [0,0,sens*L3]
    p_e2_cible = p_ee_cible - np.array([[0], [0], [sens * L3]])

# vecteur base -> poignet
    d = p_e2_cible - arrP_w0
    dx, dy, dz = d[0,0], d[1,0], d[2,0]

# 1) yaw
    j1 = math.atan2(dy, dx)

# projection dans le plan après yaw
    r = math.sqrt(dx*dx + dy*dy)
    z_plan = dz

    D = math.sqrt(r*r + z_plan*z_plan)

# atteignabilité
    if D > (L1 + L2) + 1e-9 or D < abs(L1 - L2) - 1e-9:
        raise ValueError("Cible hors de l'espace atteignable")

# 2) coude (loi des cosinus)
    c3 = (D*D - L1*L1 - L2*L2) / (2*L1*L2)
    c3 = max(-1.0, min(1.0, c3))

    if donnees.Donnees.config_coude.lower() == "haut":
        j3 = -math.acos(c3)
    else:
        j3 =  math.acos(c3)
        
# 3) épaule (ATTENTION AU SIGNE — correction clé)
    phi  = math.atan2(-z_plan, r)
    beta = math.atan2(L2 * math.sin(j3), L1 + L2 * math.cos(j3))
    j2   = phi - beta

# 4) poignet : outil perpendiculaire au sol
# j2 + j3 + j4 = sens*pi/2
    j4 = sens * (math.pi / 2) - (j2 + j3)

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

    v_0e1_b  = np.array([[L1],[0],[0]])
    v_e1e2_c = np.array([[L2],[0],[0]])
    v_e2ee_e = np.array([[-L3],[0],[0]])

    arrP_e1_w = arrP_w0 + r_bw @ v_0e1_b
    arrP_e2_w = arrP_e1_w + r_cw @ v_e1e2_c
    arrP_ee_w = arrP_e2_w + r_ew @ v_e2ee_e

    global p_ee_w
    p_ee_w = arrP_ee_w
    global p_e1_w
    p_e1_w = arrP_e1_w
    global p_e2_w
    p_e2_w = arrP_e2_w
    global p_w0
    p_w0 = arrP_w0


# axe outil (x local)
    arrX_tool_w = r_ew @ np.array([[1],[0],[0]])
    global x_tool_w
    x_tool_w = arrX_tool_w


    arrAngles = (j1, j2, j3, j4)
    global angles
    angles = arrAngles
    

    #print("Base      =", arrP_w0.ravel())
    #print("Cible     =", p_ee_cible.ravel())
    #print("Calculée  =", arrP_ee_w.ravel())
    #print("Erreur    =", (arrP_ee_w - p_ee_cible).ravel())
    #print("Axe outil =", arrX_tool_w.ravel())
    #print("Angles    =", arrAngles)



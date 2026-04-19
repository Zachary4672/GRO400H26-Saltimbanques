# bras_robot.py
import math
import numpy as np
import donnees
import json
 
 
# -----------------------------
# Global variables
#------------------------------
p_w0 = 0
x_tool_w = 0
vitesse = 0
angles = 0
gangles = 0
p_e1_w = 0
p_e2_w = 0
p_ee_w = 0
skip = False
# -----------------------------
# Rotations
# -----------------------------
def fk_from_j123(arrP_w0, L1, L2, L3, sens, j1, j2, j3):
 
    global p_ee_w, p_e1_w, p_e2_w, x_tool_w
    j4 = sens * (math.pi / 2) - (j2 + j3)
 
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
 
 
    p_e1_w1 = arrP_w0 + r_bw @ v_0e1_b
    p_e2_w1 = p_e1_w1 + r_cw @ v_e1e2_c
    p_ee_w1 = p_e2_w1 + r_ew @ v_e2ee_e
 
    x_tool_w1 = r_ew @ np.array([[1],[0],[0]])  # axe outil (x local)
 
    return p_ee_w1, p_e1_w1, p_e2_w1, x_tool_w1, j4
 
#Calcul de la jacobienne
def jacobian_pos_numeric(arrP_w0, L1, L2, L3, sens, j1, j2, j3, eps=1e-6):
 
    p0, *_ = fk_from_j123(arrP_w0, L1, L2, L3, sens, j1, j2, j3)
    J = np.zeros((3, 3))
 
    for i in range(3):
        dj1, dj2, dj3 = 0.0, 0.0, 0.0
        if i == 0: dj1 = eps
        if i == 1: dj2 = eps
        if i == 2: dj3 = eps
 
        p1, *_ = fk_from_j123(arrP_w0, L1, L2, L3, sens, j1 + dj1, j2 + dj2, j3 + dj3)
        J[:, i] = ((p1 - p0) / eps).reshape(3)
 
    return J

#Matrice de rotation
def Rz(theta):
    return np.array([
        [math.cos(theta), -math.sin(theta), 0],
        [math.sin(theta),  math.cos(theta), 0],
        [0, 0, 1]
    ])
 
#Matrice de rotation
def Ry(theta):
    return np.array([
        [ math.cos(theta), 0,  math.sin(theta)],
        [0, 1, 0],
        [-math.sin(theta), 0,  math.cos(theta)]
    ])
 
#Lecture du JSON
def load_xyz(filename = "donnees.json"):
    with open(filename, "r", encoding= "utf-8") as f:
        data = json.load(f)
 
    return data["x"], data["y"], data["z"]

def fk_xyz(arrP_w0, L1, L2, L3, sens, j1, j2, j3):
    """
    Retourne la position (x,y,z) de l'effecteur à partir des angles
    """

    # Calcul j4 (contrainte outil vertical)
    j4 = sens * (math.pi / 2) - (j2 + j3)

    # Matrices de rotation
    r_aw = Rz(j1)
    r_ba = Ry(j2)
    r_cb = Ry(j3)
    r_ec = Ry(j4)

    # Rotations cumulées
    r_bw = r_aw @ r_ba
    r_cw = r_bw @ r_cb
    r_ew = r_cw @ r_ec

    # Vecteurs des segments
    v_0e1_b  = np.array([[L1],[0],[0]])
    v_e1e2_c = np.array([[L2],[0],[0]])
    v_e2ee_e = np.array([[-L3],[0],[0]])

    # Positions successives
    p_e1 = arrP_w0 + r_bw @ v_0e1_b
    p_e2 = p_e1 + r_cw @ v_e1e2_c
    p_ee = p_e2 + r_ew @ v_e2ee_e

    # Extraction x,y,z
    x = p_ee[0,0]
    y = p_ee[1,0]
    z = p_ee[2,0]

    return x, y, z
 
def Calculate(iState, fA1, fA2, fA3, turn):
    global vitesse, p_ee_w, p_e1_w, p_e2_w, x_tool_w, angles, gangles, p_w0
 
    x_cible, y_cible, z_cible = load_xyz()
    if iState == 1 or iState == 2 or iState == 0:
        if iState == 1 or turn == 1:
            q = 0.05 #Calcul avec une approche de 5 cm
        else:
            q = 0 #Calcul sans l'approche de 5 cm
    if iState == 1 or iState == 2: #Calcul en déplacement en joint
    # -----------------------------
    # Base et cible
    # -----------------------------
        z = z_cible + q
        arrP_w0 = np.array([[donnees.Donnees.wx],
                        [donnees.Donnees.wy],
                        [donnees.Donnees.wz]])
   
        p_ee_cible = np.array([[x_cible],
                            [y_cible],
                            [z]])
   
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
   


   
    # 2) coude (loi des cosinus)
        c3 = (D*D - L1*L1 - L2*L2) / (2*L1*L2)
        c3 = max(-1.0, min(1.0, c3))
   
        if donnees.Donnees.config_coude.lower() == "haut":
            j3 = -math.acos(c3)
        else:
            j3 =  math.acos(c3)

    # atteignabilité
        if abs(math.sin(j3)) < 0.01:
            global skip
            skip = True
            return
       
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
   
 
        p_ee_w = arrP_ee_w
 
        p_e1_w = arrP_e1_w
 
        p_e2_w = arrP_e2_w
 
        p_w0 = arrP_w0
   
   
    # axe outil (x local)
        arrX_tool_w = r_ew @ np.array([[1],[0],[0]])
 
        x_tool_w = arrX_tool_w
   
   
        arrAngles = (j1, j2, j3, j4)
 
        angles = arrAngles
   
    else: 
 
        # ---------------------------------------------------------
        # Linéaire (servoing cartésien) : à partir des angles actuels
        # Inputs: fA1,fA2,fA3 = j1,j2,j3 actuels (RAD)
        # Cible cartésienne: information du JSON
        # Sortie: global vitesse = (qdot1,qdot2,qdot3,qdot4) en RAD/S
        # ---------------------------------------------------------
 
        # Base et cible
        arrP_w0 = np.array([[donnees.Donnees.wx],
                            [donnees.Donnees.wy],
                            [donnees.Donnees.wz]])
 
        p_ee_cible = np.array([[x_cible],
                                [y_cible],
                               [z_cible + q]])
 
        L1, L2, L3 = donnees.Donnees.L1, donnees.Donnees.L2, donnees.Donnees.L3
        sens = donnees.Donnees.sens_outil
 
        # Angles actuels (RAD) reçus du robot
        j1 = fA1
        j2 = fA2
        j3 = fA3
 
        # FK actuelle (EEF + points pour affichage)
        arrP_ee_w, arrP_e1_w, arrP_e2_w, arrX_tool_w, j4 = fk_from_j123(arrP_w0, L1, L2, L3, sens, j1, j2, j3)
 
        # Mise à jour globals pour affichage
 
        p_ee_w = arrP_ee_w
 
        p_e1_w = arrP_e1_w
 
        p_e2_w = arrP_e2_w
 
        p_w0   = arrP_w0
 
        x_tool_w = arrX_tool_w
 
        gangles = (j1, j2, j3, j4)
 
        # Erreur cartésienne
        e = (p_ee_cible - arrP_ee_w)   # 3x1
        dist = float(np.linalg.norm(e))
 
        # -----------------------------
        # Paramètres
        # -----------------------------
        v_cart = 50.0*1e-3     # vitesse cartésienne max (ex: 50 mm/s)
        Kp     = 2.0      # gain de correction (plus haut = plus agressif)
        tol    = 1.0*1e-3      # tolérance position
 
        # Limites vitesses joints (rad/s)
        qdot_max = np.array([2.0, 2.0, 2.0])
 
        # DLS (anti-singularités)
        lam = 0.05  # amortissement fixe (plus grand = plus stable près singularités, mais moins précis)
 
        # Si on est rendu
        if dist < tol:
            vitesse = (0.0, 0.0, 0.0, 0.0)
            return
 
        # Direction vers cible
        u = e / dist  # 3x1
 
        # Vitesse cartésienne désirée (direction + correction)
        xdot = v_cart * u + Kp * e
 
        # limiter la norme de xdot (évite des vitesses énormes loin de la cible)
        xdot_norm = float(np.linalg.norm(xdot))
        if xdot_norm > v_cart:
            xdot = xdot * (v_cart / xdot_norm)
 
        # Jacobienne position 3x3 (numérique)
        J = jacobian_pos_numeric(arrP_w0, L1, L2, L3, sens, j1, j2, j3, eps=1e-6)
 
        # Damped Least Squares: qdot = J^T (J J^T + lam^2 I)^-1 xdot
        JJt = J @ J.T
        A = JJt + (lam * lam) * np.eye(3)
        qdot_123 = (J.T @ np.linalg.solve(A, xdot.reshape(3, 1))).reshape(3)
 
        # Saturation vitesses
        qdot_123 = np.clip(qdot_123, -qdot_max, qdot_max)
 
        # j4 est imposé par contrainte: j4 = sens*pi/2 - (j2+j3)
        # Donc une vitesse cohérente serait: qdot4 = -(qdot2 + qdot3)
        qdot4 = -(float(qdot_123[1]) + float(qdot_123[2]))
 
        vitesse = (float(qdot_123[0]), float(qdot_123[1]), float(qdot_123[2]), qdot4)


def CalculateCamera(pos_x, pos_y, jb_x, jb_y):
# carré caméra
        global p_ee_w
        # pour compenser le jeu des moteurs et du mvt linéaire
        offset_descente_x = 0.02
        offset_descente_y = 0.005
         # à ajuster pour que le point calculé soit au centre de la boîte (compense la hauteur de la caméra)
        # Dimmension image
        height_px = 480
        width_px = 640
        # Champ de vision de la caméra calculé
        # Mettre à jour les angles selon la caméra
        # angle 1
        fov_w = np.deg2rad(60.43)
        t1 = fov_w/2

        # angle 2
        fov_h = np.deg2rad(44.78)
        t2 = fov_h/2
        # Hauteur de la caméra (distance verticale à la cible)
        R = 0.176 #donnees.Donnees.z_cam + donnees.Donnees.h_boite + donnees.Donnees.z_scan #à programmer

        #Vecteur en x et y pour se rendre à l'origine de la caméra
        half_w = R * math.tan(t1)
        half_h = R * math.tan(t2)

        posjb = np.array([((jb_y/height_px) * 2*half_h), ((jb_x/width_px) * 2*half_w)-half_w, 0]) 
 
        corners_local = np.array([
            [ half_w,  half_h, 0], 
            [ half_w, -half_h, 0],
            [-half_w, -half_h, 0],
            [-half_w,  half_h, 0]
        ]).T


        posjb[0] = float(p_ee_w[0, 0]) -donnees.Donnees.x_cam + float(posjb[0]) +offset_descente_x
        posjb[1] = float(p_ee_w[1, 0])+ float(posjb[1]) -offset_descente_y
        return posjb[0], posjb[1]

 
 

# bras_robot.py
import math
import numpy as np
import importlib
import donnees


# Global variables

p_w0 = None
x_tool_w = None
vitesse = None
angles = None
gangles = None
p_e1_w = None
p_e2_w = None
p_ee_w = None
p_cam_w = None  # caméra
corners_cam_w = None  # coins du cone de la caméra


# Rotations

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


# Forward Kinematics

def fk_from_j123(arrP_w0, L1, L2, L3, sens, j1, j2, j3):
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

    p_e1 = arrP_w0 + r_bw @ v_0e1_b
    p_e2 = p_e1 + r_cw @ v_e1e2_c
    p_ee = p_e2 + r_ew @ v_e2ee_e

    x_tool = r_ew @ np.array([[1],[0],[0]])

    return p_ee, p_e1, p_e2, x_tool, j4, r_ew


# Jacobienne numérique

def jacobian_pos_numeric(arrP_w0, L1, L2, L3, sens, j1, j2, j3, eps=1e-6):
    p0, *_ = fk_from_j123(arrP_w0, L1, L2, L3, sens, j1, j2, j3)
    J = np.zeros((3,3))
    for i in range(3):
        dq = [0.0, 0.0, 0.0]
        dq[i] = eps
        p1, *_ = fk_from_j123(arrP_w0, L1, L2, L3, sens,
                              j1 + dq[0], j2 + dq[1], j3 + dq[2])
        J[:, i] = ((p1 - p0)/eps).reshape(3)
    return J


# MAIN FUNCTION

def Calculate(bState, fA1, fA2, fA3):

    global vitesse, p_ee_w, p_e1_w, p_e2_w, x_tool_w
    global angles, gangles, p_w0, p_cam_w, corners_cam_w

    importlib.reload(donnees)


    # Paramètres de base

    arrP_w0 = np.array([[donnees.Donnees.wx],
                        [donnees.Donnees.wy],
                        [donnees.Donnees.wz]])
    L1 = donnees.Donnees.L1
    L2 = donnees.Donnees.L2
    L3 = donnees.Donnees.L3
    sens = donnees.Donnees.sens_outil
    Lcam_x = getattr(donnees.Donnees, "Lcam_x", 0.0)
    Lcam_y = getattr(donnees.Donnees, "Lcam_y", 0.0)
    Lcam_z = getattr(donnees.Donnees, "Lcam_z", 0.0)


    # MODE IK

    if bState==1:

        # Cible

        p_ee_cible = np.array([[donnees.Donnees.x_cible],
                               [donnees.Donnees.y_cible],
                               [donnees.Donnees.z_cible + 0.1]])
        # poignet (pour outil vertical)
        p_e2_cible = p_ee_cible - np.array([[0],[0],[sens*L3]])
        d = p_e2_cible - arrP_w0
        dx, dy, dz = d[0,0], d[1,0], d[2,0]

        # 1) yaw
        j1 = math.atan2(dy, dx)

        # projection plan après yaw
        r = math.sqrt(dx*dx + dy*dy)
        z_plan = dz
        D = math.sqrt(r*r + z_plan*z_plan)
        if D > (L1 + L2) or D < abs(L1-L2):
            raise ValueError("Cible hors de portée")

        # 2) coude (loi des cosinus)
        c3 = np.clip((D*D - L1*L1 - L2*L2)/(2*L1*L2), -1.0, 1.0)
        j3 = -math.acos(c3) if donnees.Donnees.config_coude.lower()=="haut" else math.acos(c3)

        # 3) épaule
        phi  = math.atan2(-z_plan, r)
        beta = math.atan2(L2*math.sin(j3), L1+L2*math.cos(j3))
        j2 = phi - beta

        # FK pour positions
        p_ee, p_e1, p_e2, x_tool, j4, r_ew = fk_from_j123(arrP_w0, L1,L2,L3,sens,j1,j2,j3)

        # Caméra

        offset_cam = np.array([[Lcam_x],[Lcam_y],[Lcam_z]])
        p_cam_w = p_ee + r_ew @ offset_cam

        # construire repère caméra
        z_cam = np.array([[0],[0],[-1]])            # pointe vers le sol
        x_cam = (r_ew @ np.array([[1],[0],[0]]))   # aligné avec axe pince
        y_cam = np.cross(z_cam.ravel(), x_cam.ravel()).reshape(3,1)
        y_cam /= np.linalg.norm(y_cam)
        R_cam = np.hstack([x_cam, y_cam, z_cam])

        # carré caméra
        fov_h = np.deg2rad(110)
        height = 480
        width = 640
        R = donnees.Donnees.z_cible
        half_h = R * np.tan(fov_h/2)
        fov_v = 2 * np.arctan(np.tan(fov_h/2)*(height/width))
        half_w = R * np.tan(fov_v/2)

        corners_local = np.array([
            [ half_w,  half_h, 0],
            [ half_w, -half_h, 0],
            [-half_w, -half_h, 0],
            [-half_w,  half_h, 0]
        ]).T
        corners_cam_w = p_cam_w + R_cam @ corners_local


        # globals pour affichage

        p_w0 = arrP_w0
        p_ee_w = p_ee
        p_e1_w = p_e1
        p_e2_w = p_e2
        x_tool_w = x_tool
        angles = (j1,j2,j3,j4)

    
    # MODE LINÉAIRE

    elif bState==2:
        # À FAIRE : linéaire
        pass

    else:
        pass
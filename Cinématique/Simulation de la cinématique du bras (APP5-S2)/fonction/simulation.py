# simulation_statistique.py
import numpy as np
import matplotlib.pyplot as plt
import bras_robot
import donnees

# -----------------------------
# Calcul de la position du bras et de la caméra
# -----------------------------
# Définir la cible du bras
donnees.Donnees.x_cible = 0.1
donnees.Donnees.y_cible = 0.1
donnees.Donnees.z_cible = 0.1

# Calcul cinématique inverse
bras_robot.Calculate(True, 0, 0, 0)

# Position caméra
cam_pos = bras_robot.p_cam_w.flatten()  # position caméra
cam_dir = np.array([0,0,-1])            # caméra pointe vers le sol

# Intersection caméra → sol
dist = cam_pos[2] / (-cam_dir[2])
ground_proj = cam_pos + cam_dir * dist  # point projeté au sol

# Taille du carré au sol selon FOV
fov = np.deg2rad(110)
half_size = dist * np.tan(fov/2)
corners = np.array([
    [ half_size,  half_size, 0],
    [ half_size, -half_size, 0],
    [-half_size, -half_size, 0],
    [-half_size,  half_size, 0]
]).T
corners += ground_proj.reshape(3,1)

# -----------------------------
# Boîte au sol (30x45 cm, h=5 cm)
# -----------------------------
box_x = np.array([0.0, 0.308, 0.308, 0.0, 0.0])
box_y = np.array([0.0, 0.0, 0.250, 0.250, 0.0])
box_z = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
box_top_z = box_z + 0.07 

# -----------------------------
# Tracé 3D
# -----------------------------
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.1,0.4)
ax.set_ylim(-0.1,0.5)
ax.set_zlim(0,0.3)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Bras robotique avec caméra et boîte au sol")

# Bras robotique
xs = [bras_robot.p_w0[0,0], bras_robot.p_e1_w[0,0], bras_robot.p_e2_w[0,0], bras_robot.p_ee_w[0,0]]
ys = [bras_robot.p_w0[1,0], bras_robot.p_e1_w[1,0], bras_robot.p_e2_w[1,0], bras_robot.p_ee_w[1,0]]
zs = [bras_robot.p_w0[2,0], bras_robot.p_e1_w[2,0], bras_robot.p_e2_w[2,0], bras_robot.p_ee_w[2,0]]
ax.plot(xs, ys, zs, 'bo-', label="Bras")

# Caméra
ax.scatter(cam_pos[0], cam_pos[1], cam_pos[2], c='r', s=50, label="Caméra")

# Cône caméra → sol
for j in range(4):
    ax.plot([cam_pos[0], corners[0,j]],
            [cam_pos[1], corners[1,j]],
            [cam_pos[2], corners[2,j]], 'r--')
# Carré au sol
ax.plot(np.append(corners[0,:], corners[0,0]),
        np.append(corners[1,:], corners[1,0]),
        np.append(corners[2,:], corners[2,0]), 'r-')

# Boîte au sol
ax.plot(box_x, box_y, box_z, 'k-')  # base
ax.plot(box_x, box_y, box_top_z, 'k-')  # haut
for i in range(4):
    ax.plot([box_x[i], box_x[i]], [box_y[i], box_y[i]], [box_z[i], box_top_z[i]], 'k-')  # arêtes verticales

ax.legend()
plt.show()
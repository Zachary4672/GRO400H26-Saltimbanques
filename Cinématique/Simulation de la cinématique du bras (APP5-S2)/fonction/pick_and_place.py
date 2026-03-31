# pick_and_place.py
import numpy as np
import bras_robot

Joint = 0
Lineaire = 1
Attente= 0
Fermeture = 1
Ouverture = 2
moment = 0
Pick = 1
Drop = 0
Mouvement = 2
Pince = 0
Rouge = 1
Bleu = 2
Jaune = 3

# Trajectoire de pick and place


def generate_approche(points):
    approche = []
    
    
    for pil in points:
        x,y,z = pil
        # pré-pick
        approche.append([x, y, z+0.1, Joint, Attente])
        # pick
        approche.append([x, y, z, Lineaire, Pince])
        # post-pick
        approche.append([x, y, z+0.1, license, Attente])
    return np.array(approche)

def generate_trajectory(approche, traj):
    traj = []

    if moment == Pick:
        Pince = Ouverture
    elif moment == Drop:
        Pince = Fermeture
    else : 
        Pince = Attente

    
        

    

    

    


def compute_angles(traj):
    angles_traj = []
    for pt in traj:
        bras_robot.Donnees.x_cible, bras_robot.Donnees.y_cible, bras_robot.Donnees.z_cible = pt
        bras_robot.Calculate(True)
        angles_traj.append(bras_robot.angles)
    return angles_traj
import bras_robot
import affichage
import serial
import time
 
# 1. Connexion au port (vérifie si c'est COM3, COM4, etc. dans ton gestionnaire de périphériques)
try:
    ser = serial.Serial('COM3', 115200, timeout=1) #Modifier Port
    time.sleep(2) # Très important : laisse l'OpenRB redémarrer après la connexion
    print("Connexion établie avec l'OpenRB-150")
except:
    print("Erreur : Impossible de trouver le port. Vérifie le câble USB.")
    exit()
 
def envoyer_angles(a1, a2, a3):
    # On crée la chaîne : "90.5,45.0,120.2\n"
    # On utilise .2f pour envoyer des nombres propres avec 2 décimales
    message = f"{a1:.2f},{a2:.2f},{a3:.2f}\n"
    # On encode en utf-8 (format texte) et on envoie
    ser.write(message.encode('utf-8'))
    # Optionnel : lire la confirmation de l'OpenRB
    if ser.in_waiting > 0:
        reponse = ser.readline().decode('utf-8').strip()
        print(f"OpenRB dit : {reponse}")
 
# --- Main ---
while True:
    bras_robot.Calculate()
    affichage.Draw()
    # Exemple : envoyer des positions
    envoyer_angles(180, 90, 45) #Changer en rad
    time.sleep(2) # On envoie une mise à jour toutes les 100ms
    if ():
        ser.close()
        print("Connexion fermée.")
        break

import serial
import time
import bras_robot

# 1. Connexion au port (vérifie si c'est COM3, COM4, etc. dans ton gestionnaire de périphériques)
try:
    ser = serial.Serial('COM9', 115200, timeout=1)
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

# --- TEST ---
try:
    while True:
        # Exemple : envoyer des positions
        envoyer_angles(j1, j2, j3)
        time.sleep(1) # On envoie une mise à jour toutes les 100ms
except KeyboardInterrupt:
    ser.close()
    print("Connexion fermée.")
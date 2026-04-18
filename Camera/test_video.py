#---------------test_video.py------------#
'''Description: Test les fonctionnalité de
main_video.py. Permet d'afficher les images
capté par la caméra avant et après le 
traitement d'image YOLO. Programme à utiliser
pour calibré les valeur de couleurs. 
'''
import numpy as np
from ultralytics import YOLO
import cv2
import math as m
import threading as th
import queue as q
import time
import main_video as mv


#Initialisation de la caméra et des threads
mv.init_camera()
th.Thread(target=mv.capture_frame, args=(mv.cap,mv.frame_queue), daemon=True).start()
th.Thread(target=mv.image_process, args=(mv.frame_queue,), daemon=True).start()
i = 0
key = cv2.waitKey(1)

while True:
    #Lancer la détection en appuyant sur 'd'
    if cv2.waitKey(1) == ord('d'):  # exemple touche clavier
        for _ in range(5):  # nombre de frames
            mv.run_detection.set()
            time.sleep(0.05)
        if not mv.pos_queue.empty():
            pos_JB = mv.pos_queue.get()
            print(f"Position JB {pos_JB}")
    if not mv.disp_queue.empty():
        frame = mv.disp_queue.get()
        cv2.imshow("YOLO", frame)
    #Affichage sans traitement d'image
    if mv.disp_queue.empty() and not mv.frame_queue.empty():
        frame = mv.frame_queue.get()
        cv2.imshow("No detection", frame)
    if cv2.waitKey(1) == 27:
        break

    #Sauvegarder des images au besoin
    if key == ord(' '):  # appuyer sur espace
        cv2.imwrite(f"calib_{i}.jpg", frame)
        print(f"Image {i} sauvée")
        i += 1
    key = cv2.waitKey(1) & 0xFF



mv.cap.release()
mv.cv2.destroyAllWindows()


import numpy as np
from ultralytics import YOLO
import cv2
import math as m
import threading as th
import queue as q
import time

#Création des files d'attente ("queue") pour les données à utilisé entre les fils ("thread")
frame_queue = q.Queue(maxsize=1)
disp_queue = q.Queue(maxsize=1)
pos_queue = q.Queue(maxsize=1)

# notification pour détecter JB
run_detection = th.Event()

#valeurs hue couleurs
#À calibré manuellement selon l'éclairage
rouge_low = 8
rouge_high = 175
orange = 15
jaune = 35
vert = 85
mauve = 129
noir = 125
rose = 173




def init_camera():
    global cap, model, K, dist

    #Initiation de la caméra
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    #Chargement du model yolo
    model = YOLO("Camera/jelly_bean1.pt")

    #Chargement des fichier de calibration
    K = np.load("Camera/K.npy")
    dist = np.load("Camera/dist.npy")

#Obtention d'une image calibrée sans autre traitement
def capture_frame(cap, frame_queue):
    ret, frame = cap.read()
    h, w = frame.shape[:2]

    #Nouveau facteur de calibration et dimension
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0, (w, h))

    while True:
        ret, frame = cap.read()

        #sécurité anti-crash
        if not ret or frame is None:
            continue
        #Application de la calibration et du redimensionnement
        frame = frame.copy()
        undistord = cv2.undistort(frame, K, dist, None, new_K)
        frame = undistord
        x, y, w, h = roi
        frame = frame[y:y + h, x:x + w]
        
        #Vérification de la disponibilité de la file d'attente
        if frame_queue.full():
            try:
                frame_queue.get_nowait()
            except q.Empty:
                pass
        #Ajout de la nouvelle frame
        frame_queue.put(frame.copy())
        time.sleep(0.001)

#Traitement de l'image
def image_process(frame_queue):
    prev_angle = None

    while True:
        #Attente de la notification
        run_detection.wait()
        try:
            #obtiention d'une image de frame_queue
            frame = frame_queue.get(timeout=0.01)
        except q.Empty:
            continue #Recommence la boucle si aucune image

        pos_JB = []
        #Détection des objets du modèle yolo
        results = model(frame, conf=0.7)

    #Traitement pour chaque pilules individuellement
        for box in results[0].boxes:
            #Redimenisionnement pour une pilules seulement
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            #ROI = région d'intérêt
            roi = frame[y1:y2, x1:x2]

            #Convesion en HSV pour analyse de couleur
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            
        # Seuil pour couleurs vives (à ajuster)
            lower = np.array([0, 80, 80])
            upper = np.array([180, 255, 255])
            tresh = cv2.inRange(hsv, lower, upper)

        #Nettoyage du bruit
            kernel = np.ones((5, 5), np.uint8)
            tresh = cv2.morphologyEx(tresh, cv2.MORPH_CLOSE, kernel)
            tresh = cv2.morphologyEx(tresh, cv2.MORPH_OPEN, kernel)

        #Masque pour couleur
            mask = tresh > 0
            pixels = hsv[mask]
            mean_s = np.mean(pixels[:,1])

            if len(pixels) == 0 :
                color = "unknown"
                continue

            mean_h = np.mean(pixels[:,0])
        #classification couleur
            if mean_h < rouge_low:
                color = "rouge"
            elif mean_h < orange:
                color = "orange"
            elif mean_h < jaune:
                color = "jaune"
            elif mean_h < vert:
                color = "vert"
            elif mean_h < mauve:
                color = "mauve"
            elif mean_h < noir:
                color = "noir"
            elif mean_h < rose and mean_s < 188:
                color = "rose"
            elif mean_h < rouge_high and mean_s >= 188:
                color = "rouge"
            
            else:
                color = "unknown"

            #filtre saturation
            mean_s = np.mean(pixels[:,1])
            if mean_s < 50:
                color = "unknown"



        #Détection des contours d'une pilules
            contours, _ = cv2.findContours(tresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            #Garde le contour le plus important
            cnt = max(contours, key=cv2.contourArea)

            #Filtre sur la taile minimale
            if cv2.contourArea(cnt) < 200:
                continue


        # PCA pour orientation
            #Nécecite un minimum de 5 points
            if len(cnt) < 5:
                continue

            #Pour trouver l'orientation principale
            data_pts = cnt.reshape(-1, 2).astype(np.float32)
            mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
            #Direction principale
            vx, vy = eigenvectors[0]
            
            #Calcul de l'angle en degrés
            angle = m.degrees(m.atan2(-vy, vx))

            # Calcul du centre avec les moments
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"]) + x1
                cy = int(M["m01"] / M["m00"]) + y1

            # vecteur normal (perpendiculaire)
            nx, ny = -vy, vx

            # dimensions (YOLO box)
            w = x2 - x1
            h = y2 - y1
            L = w / 2
            W = h / 2

            #4 coins du rectangle orienté
            pt1 = (cx + vx * L + nx * W, cy + vy * L + ny * W)
            pt2 = (cx - vx * L + nx * W, cy - vy * L + ny * W)
            pt3 = (cx - vx * L - nx * W, cy - vy * L - ny * W)
            pt4 = (cx + vx * L - nx * W, cy + vy * L - ny * W)

            box_pts = np.array([pt1, pt2, pt3, pt4], dtype=np.int32)

            #Vecteur direction (pour affichage)
            length = 50
            dx = int(length * vx)
            dy = int(length * vy)


            #Normalisation de l'angle entre 0 et 360 degrés
            angle = ((angle + 360) % 360)
            prev_angle = angle

            #Sauvegarde des infos
            pos_JB.append({
                'x': cx,
                'y': cy,
                'angle': angle,
                'color': color # remplacer par couleur éventuellement
            })

            #---Affichage--- (Utilise seulement en test)
            #centre
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            #Vecteur direction
            cv2.line(frame, (int(cx), int(cy)), (int(cx+dx), int(cy+dy)), (255, 0, 0), 2)
            # ligne de référence (angle 0°)
            cv2.line(frame, (cx, cy), (cx + 50, cy), (255, 0, 0), 2)
            #--Texte--
            cv2.putText(frame, 
                f"x:{cx:.0f},y:{cy:.0f}, angle : {angle:.1f}, couleur : {color},
                mean_h :{mean_h:.0f}, mean_s :{mean_s:.0f}",
                (int(cx + dx), int(cy + dy)), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (0, 0, 255), 
                2
            )
            #Contour de la pilule
            cv2.drawContours(frame, [box_pts], 0, (0, 0, 255), 2)

        #Mise à jours des infos
        if pos_queue.full():
            try:
                pos_queue.get_nowait()
            except q.Empty:
                pass
        pos_queue.put(pos_JB)
        #Image yolo annotée
        annotated = results[0].plot()

        #Mise à jour de la file d'attente d'affichage avec traitement
        if disp_queue.full():
            try:
                disp_queue.get_nowait()
            except q.Empty:
                pass

        disp_queue.put(frame)
        #Reset la notificaiton
        run_detection.clear()







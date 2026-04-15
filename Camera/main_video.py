import numpy as np
from ultralytics import YOLO
import cv2
import math as m
import threading as th
import queue as q
import time

# import warnings
# warnings.filterwarnings("ignore")

frame_queue = q.Queue(maxsize=1)
disp_queue = q.Queue(maxsize=1)
pos_queue = q.Queue(maxsize=1)

# notification pour détecter JB
run_detection = th.Event()

#valeur hue couleurs
rouge_low = 8
rouge_high = 170
orange = 15
jaune = 35
vert = 85
mauve = 120
noir = 125
rose = 180


def init_camera():
    global cap, model, K, dist

    model = YOLO("Camera/jelly_bean1.pt")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    K = np.load("Camera/K.npy")
    dist = np.load("Camera/dist.npy")

def capture_frame(cap, frame_queue):
    ret, frame = cap.read()
    h, w = frame.shape[:2]

    new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0, (w, h))
    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            continue
        frame = frame.copy()
        undistord = cv2.undistort(frame, K, dist, None, new_K)
        frame = undistord
        x, y, w, h = roi
        frame = frame[y:y + h, x:x + w]
        
        if frame_queue.full():
            try:
                frame_queue.get_nowait()
            except q.Empty:
                pass

        frame_queue.put(frame.copy())
        time.sleep(0.001)

def image_process(frame_queue):
    prev_angle = None
    while True:
        run_detection.wait()
        try:
            frame = frame_queue.get(timeout=0.01)
        except q.Empty:
            continue

        pos_JB = []

        results = model(frame, conf=0.7)

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            roi = frame[y1:y2, x1:x2]

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # exemple pour couleurs vives (à ajuster)
            lower = np.array([0, 80, 80])
            upper = np.array([180, 255, 255])
            tresh = cv2.inRange(hsv, lower, upper)


            kernel = np.ones((5, 5), np.uint8)
            tresh = cv2.morphologyEx(tresh, cv2.MORPH_CLOSE, kernel)
            tresh = cv2.morphologyEx(tresh, cv2.MORPH_OPEN, kernel)

            #Masque pour couleur
            mask = tresh > 0
            pixels = hsv[mask]

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
            elif mean_h < rouge_high:
                color = "rouge"
            elif mean_h < rose:
                color = "rose"
            else:
                color = "unknown"

            #filtre saturation
            mean_s = np.mean(pixels[:,1])
            if mean_s < 50:
                color = "unknown"
            contours, _ = cv2.findContours(tresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(cnt) < 200:
                continue


            if len(cnt) < 5:
                continue



            # PCA pour orientation
            data_pts = cnt.reshape(-1, 2).astype(np.float32)
            mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
            vx, vy = eigenvectors[0]

            angle = m.degrees(m.atan2(-vy, vx))




            length = 50
            dx = int(length * vx)
            dy = int(length * (-vy))


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

            # coins du rectangle orienté
            pt1 = (cx + vx * L + nx * W, cy + vy * L + ny * W)
            pt2 = (cx - vx * L + nx * W, cy - vy * L + ny * W)
            pt3 = (cx - vx * L - nx * W, cy - vy * L - ny * W)
            pt4 = (cx + vx * L - nx * W, cy + vy * L - ny * W)

            box_pts = np.array([pt1, pt2, pt3, pt4], dtype=np.int32)
            dx = int(length * vx)
            dy = int(length * vy)



            angle = ((angle + 360) % 360)
            prev_angle = angle
            


            #centre JB


            pos_JB.append({
                'x': cx,
                'y': cy,
                'angle': angle,
                'color': color # remplacer par couleur éventuellement
            })
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.line(frame, (int(cx), int(cy)), (int(cx+dx), int(cy+dy)), (255, 0, 0), 2)
            # ligne angle 0°
            cv2.line(frame, (cx, cy), (cx + 50, cy), (255, 0, 0), 2)

            cv2.putText(frame, f"{cx:.0f},{cy:.0f}, angle : {angle:.1f}, couleur : {color}, mean_h :{mean_h:.0f}, mean_s :{mean_s:.0f}", (int(cx + dx), int(cy + dy)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.drawContours(frame, [box_pts], 0, (0, 0, 255), 2)

        if pos_queue.full():
            try:
                pos_queue.get_nowait()
            except q.Empty:
                pass
        pos_queue.put(pos_JB)
        # print(f"Position JB {pos_JB}")
        annotated = results[0].plot()

        if disp_queue.full():
            try:
                disp_queue.get_nowait()
            except q.Empty:
                pass

        disp_queue.put(frame)
        
        run_detection.clear()




# th.Thread(target=capture_frame, args=(cap,frame_queue), daemon=True).start()
# th.Thread(target=image_process, args=(frame_queue,), daemon=True).start()

# #Main
# while True:

#     if cv2.waitKey(1) == ord('d'):  # exemple touche clavier
#         for _ in range(5):  # nombre de frames
#             run_detection.set()
#             time.sleep(0.05)
#         if not pos_queue.empty():
#             pos_JB = pos_queue.get()
#             print(f"Position JB {pos_JB}")
#     if not disp_queue.empty():
#         frame = disp_queue.get()
#         cv2.imshow("YOLO", frame)
#     if disp_queue.empty() and not frame_queue.empty():
#         frame = frame_queue.get()
#         cv2.imshow("No detection", frame)
#     if cv2.waitKey(1) == 27:
#         break
#     key = cv2.waitKey(1) & 0xFF






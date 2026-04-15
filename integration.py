import numpy as np
from torch import mv
from ultralytics import YOLO
import cv2
import math as m
import threading as th
import queue as q
import time
from Camera import main_video as cam

#Initialisation de la caméra et des threads
#declarer que la liste soit une variable globale


def init_integration():
    cam.init_camera()
    th.Thread(target=cam.capture_frame, args=(cam.cap,cam.frame_queue), daemon=True).start()
    th.Thread(target=cam.image_process, args=(cam.frame_queue,), daemon=True).start()


#Main
def scan_cam():

    for _ in range(5):  # nombre de frames
        cam.run_detection.set()
        time.sleep(0.05)
    if not cam.pos_queue.empty():
        pos_JB = cam.pos_queue.get()
        if not cam.disp_queue.empty():
            frame = cam.disp_queue.get()
            cv2.imshow("YOLO", frame)
        print(f"Position JB {pos_JB}")
        return pos_JB
    key = cv2.waitKey(1) & 0xFF 
    if key == 27:
        return

    

def display_cam():
    if not cam.disp_queue.empty():
        frame = cam.disp_queue.get()
    elif not cam.frame_queue.empty():
        frame = cam.frame_queue.get()
    else:
        frame = None
    cv2.imshow("No detection", frame)
    key = cv2.waitKey(1) & 0xFF 
    if key == 27:
        return


def close_camera():
    cam.cap.release()
    cv2.destroyAllWindows()


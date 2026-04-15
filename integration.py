import numpy as np
from ultralytics import YOLO
import cv2
import math as m
import threading as th
import queue as q
import time
from Camera import main_video as mv

#Initialisation de la caméra et des threads
#declarer que la liste soit une variable globale
key = cv2.waitKey(1) & 0xFF

def init_integration():
    mv.init_camera()
    th.Thread(target=mv.capture_frame, args=(mv.cap,mv.frame_queue), daemon=True).start()
    th.Thread(target=mv.image_process, args=(mv.frame_queue,), daemon=True).start()


#Main
def scan_cam():

    for _ in range(5):  # nombre de frames
        mv.run_detection.set()
        time.sleep(0.05)
    if not mv.pos_queue.empty():
        pos_JB = mv.pos_queue.get()
        if not mv.disp_queue.empty():
            frame = mv.disp_queue.get()
            cv2.imshow("YOLO", frame)
        print(f"Position JB {pos_JB}")
        return pos_JB
    
    if key == 27:
        return

    

def display_cam():

    if mv.disp_queue.empty() and not mv.frame_queue.empty():
        frame = mv.frame_queue.get()
        cv2.imshow("No detection", frame)
    if key == 27:
        return


def close_camera():
    mv.cap.release()
    cv2.destroyAllWindows()


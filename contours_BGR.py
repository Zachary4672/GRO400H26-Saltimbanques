from __future__ import print_function
import cv2 as cv
import numpy as np

max_value = 255
max_value_H =360//2

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
cap = cv.VideoCapture(0,cv.CAP_V4L2)

while True:
    ret, frame = cap.read()
    #print (ret, frame)
    if not ret:
        print("Erreur camera")
        break
    #blur channel
    blur = cv.GaussianBlur(frame, (3, 3), 0)

    #separation cannaux BGR
    blue, green, red =cv.split(blur)
    cv.imshow("blue", blue)
    cv.imshow("green", green)
    cv.imshow("red", red)

    #detect the contours using blue channel
    contours1, hierarchy1 = cv.findContours( image=blue, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)

    vid_cont_blue = frame.copy()
    cv.drawContours(image=vid_cont_blue, contours=contours1, contourIdx=-1, color=(0, 255, 0), thickness=2,
                    lineType=cv.LINE_AA)
    #show result

    cv.imshow("Contour detection cannal bleu", vid_cont_blue)

    # detect the contours using green channel
    contours2, hierarchy2 = cv.findContours(image=green, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)

    vid_cont_green = frame.copy()
    cv.drawContours(image=vid_cont_green, contours=contours2, contourIdx=-1, color=(0, 255, 0), thickness=2,
                    lineType=cv.LINE_AA)
    # show result

    cv.imshow("Contour detection cannal vert", vid_cont_green)

    # detect the contours using red channel
    contours3, hierarchy3 = cv.findContours(image=red, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)

    vid_cont_red = frame.copy()
    cv.drawContours(image=vid_cont_red, contours=contours3, contourIdx=-1, color=(0, 255, 0), thickness=2,
                    lineType=cv.LINE_AA)
    # show result

    cv.imshow("Contour detection cannal red", vid_cont_red)
    #draw contours on the original vid


    #creation de masques locaux

    #cv.imshow("filtreHSV", frame_HSV)
    cv.imshow("video", frame)


    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
    #Ferme le programme lorsque la fenetre est fermee
    if cv.getWindowProperty("Camera", 0) >= 1:
        break

cap.release()
cv.destroyAllWindows()
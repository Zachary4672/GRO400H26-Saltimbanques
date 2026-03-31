from __future__ import print_function
import cv2 as cv
import numpy as np

max_value = 255
max_value_H =360//2

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'




cap = cv.VideoCapture(0,cv.CAP_V4L2)
cv.namedWindow(window_capture_name)
cv.namedWindow(window_detection_name)

while True:
    ret, frame = cap.read()
    #print (ret, frame)
    if not ret:
        print("Errreur camera")
        break
    #blur channel
    blur = cv.GaussianBlur(frame, (3, 3), 0)

    vid_gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)

    ret, thresh = cv.threshold(vid_gray, 150, 255, cv.THRESH_BINARY)

    #detect the contours on the binary image using cv.CHAINE_APPROX_SIMPLE and RETR_LIST
    contours, hierarchy = cv.findContours( image=thresh, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_SIMPLE)

    #draw contours on a copy of the original vid
    vid_copy_simple = frame.copy()
    cv.drawContours(vid_copy_simple, contours, -1,(0, 255, 0),2, cv.LINE_AA)

    vid_copy_point_only = frame.copy()
    for i, contour in enumerate(contours): # loop over one contour area
        for j, contour_point in enumerate(contour): #loop over the points
            #draw a circle on the current contour coordinate
            cv.circle(vid_copy_point_only, ((contour_point[0][0], contour_point[0][1])), 2, (0, 0, 255), 2, cv.LINE_AA)

    cv.imshow(window_capture_name, frame)
    cv.imshow(window_detection_name, thresh)
    cv.imshow("detection point seulement", vid_copy_point_only)
    cv.imshow("detection contour simple", vid_copy_simple )
    print(f"LIST:{hierarchy}")

    key = cv.waitKey(30)
    if key == ord('q') or key == 27:
        break
    #Ferme le programme lorsque la fenetre est fermee
    if cv.getWindowProperty("Camera", 0) >= 1:
        break

cap.release()
cv.destroyAllWindows()


import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2) #sur linux/piOS
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) #sur windows 

i = 0
while True:
    ret, frame = cap.read()
    cv2.imshow("capture", frame)

    key = cv2.waitKey(1)

    if key == ord(' '):  # Appuyer sur "espace" pour sauvegarder l'image 
        cv2.imwrite(f"calib_{i}.jpg", frame)
        print(f"Image {i} sauvée")
        i += 1

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
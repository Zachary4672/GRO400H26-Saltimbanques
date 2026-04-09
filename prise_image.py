import cv2

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

i = 0
while True:
    ret, frame = cap.read()
    cv2.imshow("capture", frame)

    key = cv2.waitKey(1)

    if key == ord('s'):  # sauvegarder
        cv2.imwrite(f"calib_{i}.jpg", frame)
        print(f"Image {i} sauvée")
        i += 1

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
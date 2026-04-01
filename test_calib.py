import cv2
import numpy as np

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

K = np.load("K.npy")
dist = np.load("dist.npy")
K2 = np.load("K2.npy")
dist2 = np.load("dist2.npy")

i = 0
ret, frame = cap.read()
h, w = frame.shape[:2]

new_K, roi = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0, (w, h))
new_K2, roi2 = cv2.getOptimalNewCameraMatrix(K2, dist2, (w, h), 0, (w, h))




# crop optionnel

while True:

    ret, frame = cap.read()
    frame0 = cv2.undistort(frame, K, dist, None, new_K)
    frame1 = cv2.undistort(frame, K2, dist2, None, new_K2)
    x, y, w, h = roi
    x2, y2, w2, h2 = roi2
    frame0 = frame[y:y + h, x:x + w]
    frame1 = frame1[y:y2 + h, x:x2 + w]
    cv2.imshow("capture1", frame0)
    cv2.imshow("capture2", frame1)

    key = cv2.waitKey(1)

    if key == ord('s'):  # sauvegarder
        cv2.imwrite(f"calib_{i}.jpg", frame)
        print(f"Image {i} sauvée")
        i += 1

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
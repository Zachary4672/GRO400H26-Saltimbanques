import cv2
import numpy as np
import glob

lar = 17
haut= 11
# damier
chessboard_size = (lar,haut)

# points 3D
objp = np.zeros((lar*haut, 3), np.float32)
objp[:, :2] = np.mgrid[0:lar, 0:haut].T.reshape(-1, 2)

objpoints = []
imgpoints = []

images = glob.glob("calib_*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    print(fname, "->", ret)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow("corners", img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("K =\n", K)
print("dist =\n", dist)
np.save("K2.npy", K)
np.save("dist2.npy", dist)
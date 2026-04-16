import cv2
import numpy as np
import glob

lar =7
haut= 7
# damier
chessboard_size = (lar,haut)

#critère de raffinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# points 3D
objp = np.zeros((lar*haut, 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

objpoints = []
imgpoints = []

images = glob.glob("calib_*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    print(fname, "->", ret)

    if ret:
        objpoints.append(objp)
        #raffiner les coins
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        #affichage
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow("corners", img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

#calibration
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("K =\n", K)
print("dist =\n", dist)
np.save("K2.npy", K)
np.save("dist2.npy", dist)
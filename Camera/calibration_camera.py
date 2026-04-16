import cv2
import numpy as np
import glob
import os

# Nombre de coins internes du damier
# Exemple : si ton damier a 10 carrés par 7 carrés,
# il a probablement 9 x 6 coins internes.
CHECKERBOARD = (5, 4)

# Taille réelle d'un carré du damier.
# Mets ta vraie valeur si tu la connais.
# L'unité peut être en mm.
square_size = 6.0

criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[
    0:CHECKERBOARD[0],
    0:CHECKERBOARD[1]
].T.reshape(-1, 2)

objp *= square_size

objpoints = []
imgpoints = []

images = glob.glob("calibration_images/*.jpg")


if len(images) == 0:
    raise RuntimeError("Aucune image trouvée dans calibration_images/")

img_size = None
valid_images = 0

for fname in images:
    img = cv2.imread(fname)
     

    if img is None:
        print(f"Impossible de lire : {fname}")
        continue

    img = img[70:415, 45:590] 

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    if img_size is None:
        img_size = gray.shape[::-1]

    ret, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret:

        objpoints.append(objp)
        imgpoints.append(corners)
        valid_images += 1

        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow("Coins detectes", img)
        cv2.waitKey(100)

    else:
        print(f"Damier non detecte : {fname}")

cv2.destroyAllWindows()

print(f"Images valides utilisees : {valid_images}")

if valid_images < 10:
    raise RuntimeError("Pas assez d'images valides. Vise au moins 15 a 20 bonnes images.")

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    img_size,
    None,
    None
)

print("Erreur RMS :", ret)
print("K =")
print(K)
print("dist =")
print(dist)

os.makedirs("Camera", exist_ok=True)

np.save("Camera/K.npy", K)
np.save("Camera/dist.npy", dist)

print("Calibration sauvegardee dans Camera/K.npy et Camera/dist.npy")
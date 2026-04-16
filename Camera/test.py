import cv2
import glob

images = glob.glob("calibration_images/*.jpg")

if len(images) == 0:
    raise RuntimeError("Aucune image trouvee dans calibration_images/")

# Prend la première image pour tester
image_path = images[0]
print("Image testee :", image_path)

img = cv2.imread(image_path)

if img is None:
    raise RuntimeError("Impossible de lire l'image")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Amélioration du contraste
gray = cv2.equalizeHist(gray)

# Liste de dimensions à tester
# Format = nombre de coins internes, pas nombre de carres
tests = []

for largeur in range(5, 25):
    for hauteur in range(4, 15):
        tests.append((largeur, hauteur))

found = False

for checkerboard in tests:
    ret, corners = cv2.findChessboardCornersSB(
        gray,
        checkerboard,
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ret:
        print("DAMIER DETECTE :", checkerboard)

        img_draw = img.copy()
        cv2.drawChessboardCorners(img_draw, checkerboard, corners, ret)

        cv2.imshow(f"Detecte {checkerboard}", img_draw)
        cv2.waitKey(0)

        found = True
        break

if not found:
    print("Aucun damier detecte avec les dimensions testees.")

cv2.destroyAllWindows()
import cv2
import numpy as np
import glob

# https://markhedleyjones.com/projects/calibration-checkerboard-collection

# Chessboard size (inner corners)
chessboard_size = (10, 7)

# Real square size (meters)
square_size = 0.025  # 25 mm

# Prepare object points
objp = np.zeros((10*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:10, 0:7].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

images = glob.glob("cameracalibration/calib_images/calib_*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow("Corners", img)
        cv2.waitKey(200)

cv2.destroyAllWindows()

# Calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

def print_matrix(mat):
    print("[")
    for row in mat:
        print("  [" + ", ".join(str(x) for x in row) + "],")
    print("]")

print("Camera matrix:")
print(mtx)
print_matrix(mtx)
# for row in mtx:
#     print("[")
#     for col in row:
#         print(f"{col}")
#     print("]")
print("\nDistortion:")
print(dist)
print_matrix(dist)
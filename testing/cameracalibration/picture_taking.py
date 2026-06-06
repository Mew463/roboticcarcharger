import cv2
import os

# cap = cv2.VideoCapture(0)

# Use the V4L2 backend for more reliable control on Linux/Pi
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# # Set the resolution
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# # Check the actual result
w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)

print(f"Final Config: {int(w)}x{int(h)} at {fps} FPS")

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Create folder for images
os.makedirs("cameracalibration/calib_images", exist_ok=True)

count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    winname = "Camera - Press 's' to save, 'q' to quit"
    cv2.namedWindow(winname, cv2.WINDOW_NORMAL)
    cv2.imshow(winname, frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        filename = f"cameracalibration/calib_images/calib_{count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
        count += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
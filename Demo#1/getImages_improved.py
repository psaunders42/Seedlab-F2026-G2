import cv2
import os
import glob

# --------------------------------------------------
# CAMERA SETTINGS - USE THESE SAME SETTINGS IN THE DEMO
# --------------------------------------------------
IMAGE_FOLDER = "images_new"
os.makedirs(IMAGE_FOLDER, exist_ok=True)

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    raise RuntimeError("Could not open /dev/video0")

print("Actual capture size:", cap.get(cv2.CAP_PROP_FRAME_WIDTH),
      "x", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Press s to save a NEW chessboard viewpoint, ESC to quit.")
print("Aim for 25-40 varied, sharp, well-lit pictures: corners and tilts.")

num = len(glob.glob(os.path.join(IMAGE_FOLDER, "*.png")))
while True:
    success, frame = cap.read()
    if not success:
        print("Could not read image")
        break

    cv2.imshow("Calibration capture", frame)
    key = cv2.waitKey(5) & 0xFF
    if key == 27:
        break
    if key == ord("s"):
        name = os.path.join(IMAGE_FOLDER, f"img_{num:03d}.png")
        if cv2.imwrite(name, frame):
            print("Saved:", name)
            num += 1
        else:
            print("Save failed:", name)

cap.release()
cv2.destroyAllWindows()

import cv2
import cv2.aruco as aruco
import numpy as np
import pickle
import time
import threading
import queue
from smbus import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# --------------------------------------------------
# CALIBRATION - RUN calibration_improved.py FIRST
# --------------------------------------------------
with open("calibration_improved.pkl", "rb") as f:
    camera_matrix, dist_coeffs = pickle.load(f)

optical_cx = float(camera_matrix[0, 2])
optical_cy = float(camera_matrix[1, 2])

# --------------------------------------------------
# SETTINGS
# --------------------------------------------------
TARGET_ID = 0
ARUCO_DICT = aruco.DICT_6X6_50
DEADBAND_PX = 15
ANGLE_THRESHOLD = 0.25          # Arduino change threshold (NOT precision)
ANGLE_ALPHA = 0.25              # Smoothed fraction of newest angle
ARD_ADDR = 8
CMD_QUADRANT = 0x01
CMD_ANGLE = 0x02
QUADRANT_BITS = {
    "NE": (0, 0),
    "NW": (0, 1),
    "SW": (1, 1),
    "SE": (1, 0),
}

# --------------------------------------------------
# LCD
# --------------------------------------------------
lcd_queue = queue.Queue(maxsize=1)

def lcd_display():
    try:
        time.sleep(1)
        lcd = character_lcd.Character_LCD_RGB_I2C(board.I2C(), 16, 2)
        lcd.clear()
        lcd.color = [0, 0, 100]
        while True:
            angle = lcd_queue.get()
            lcd.message = f"x: {angle:.2f} deg".ljust(16)
    except Exception as e:
        print("LCD thread error:", e)

threading.Thread(target=lcd_display, daemon=True).start()
Arduino_bus = SMBus(1)

# --------------------------------------------------
# ARUCO DETECTION WITH SUBPIXEL CORNER REFINEMENT
# --------------------------------------------------
aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
if hasattr(aruco, "DetectorParameters"):
    detector_params = aruco.DetectorParameters()
else:
    detector_params = aruco.DetectorParameters_create()
detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

# --------------------------------------------------
# CAMERA SETUP - MATCH CALIBRATION CAPTURE
# --------------------------------------------------
camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
camera.set(cv2.CAP_PROP_FPS, 30)
if not camera.isOpened():
    raise RuntimeError("Could not open /dev/video0")
print("Camera:", camera.get(cv2.CAP_PROP_FRAME_WIDTH),
      camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
if (int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))) != (1280, 720):
    raise RuntimeError("Capture resolution differs from calibration!")
time.sleep(2)

last_quadrant = None
last_sent_angle = None
last_lcd_time = 0.0
last_ard_angle_time = 0.0
filtered_angle = None

# --------------------------------------------------
# MAIN LOOP
# --------------------------------------------------
try:
    while True:
        ok, frame = camera.read()
        if not ok:
            print("Camera frame failed")
            break

        # Detect on RAW frame. Correct corner coordinates using undistortPoints.
        # This avoids unnecessary full-image undistortion and keeps coordinates consistent.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(
            gray, aruco_dict, parameters=detector_params)
        overlay = frame.copy()
        h, w = frame.shape[:2]
        cv2.line(overlay, (int(optical_cx), 0), (int(optical_cx), h), (255, 255, 255), 1)
        cv2.line(overlay, (0, int(optical_cy)), (w, int(optical_cy)), (255, 255, 255), 1)

        target_found = False
        if ids is not None:
            for marker_outline, marker_id in zip(corners, ids.flatten()):
                if int(marker_id) != TARGET_ID:
                    continue
                target_found = True

                # --------------------------------------------------
                # MARKER CENTER (KEEP SUBPIXEL PRECISION)
                # --------------------------------------------------
                raw_corners = marker_outline.reshape((4, 2)).astype(np.float32)
                center = raw_corners.mean(axis=0)
                center_x, center_y = float(center[0]), float(center[1])
                center_px = (int(round(center_x)), int(round(center_y)))
                dx = center_x - optical_cx
                dy = center_y - optical_cy

                # --------------------------------------------------
                # DISTORTION-CORRECTED HORIZONTAL BEARING
                # --------------------------------------------------
                # Correct each corner to a normalized camera ray. No pixel
                # focal-length division is needed after this function.
                rays = cv2.undistortPoints(
                    raw_corners.reshape(-1, 1, 2), camera_matrix, dist_coeffs
                ).reshape(-1, 2)
                marker_ray_x = float(rays[:, 0].mean())
                raw_angle_deg = float(np.degrees(np.arctan2(-marker_ray_x, 1.0)))
                if filtered_angle is None:
                    filtered_angle = raw_angle_deg
                else:
                    filtered_angle += ANGLE_ALPHA * (raw_angle_deg - filtered_angle)
                angle_deg = filtered_angle

                # --------------------------------------------------
                # QUADRANT - INDEPENDENT OF ANGLE CALCULATION
                # --------------------------------------------------
                if abs(dx) <= DEADBAND_PX or abs(dy) <= DEADBAND_PX:
                    quadrant = last_quadrant
                elif dx > 0 and dy < 0:
                    quadrant = "NE"
                elif dx < 0 and dy < 0:
                    quadrant = "NW"
                elif dx < 0 and dy > 0:
                    quadrant = "SW"
                else:
                    quadrant = "SE"

                if quadrant is not None and quadrant != last_quadrant:
                    left_bit, right_bit = QUADRANT_BITS[quadrant]
                    goal_byte = (left_bit << 1) | right_bit
                    try:
                        Arduino_bus.write_byte_data(ARD_ADDR, CMD_QUADRANT, goal_byte)
                        last_quadrant = quadrant
                        print("Quadrant:", quadrant, "sent:", goal_byte)
                    except OSError as e:
                        print("Failed to send quadrant:", e)

                # --------------------------------------------------
                # ANGLE TO ARDUINO - WORKS EVEN IF QUADRANT IS UNKNOWN
                # --------------------------------------------------
                now = time.time()
                angle_changed = (last_sent_angle is None or
                                 abs(angle_deg - last_sent_angle) >= ANGLE_THRESHOLD)
                if now - last_ard_angle_time >= 0.1 and angle_changed:
                    angle_int = int(round(angle_deg * 100))
                    try:
                        Arduino_bus.write_word_data(ARD_ADDR, CMD_ANGLE,
                                                    angle_int & 0xFFFF)
                        last_ard_angle_time = now
                        last_sent_angle = angle_deg
                    except OSError as e:
                        print("Failed to send angle:", e)

                if now - last_lcd_time >= 0.25:
                    try:
                        if lcd_queue.full():
                            lcd_queue.get_nowait()
                        lcd_queue.put_nowait(angle_deg)
                    except queue.Full:
                        pass
                    last_lcd_time = now

                # --------------------------------------------------
                # DISPLAY
                # --------------------------------------------------
                aruco.drawDetectedMarkers(overlay, [marker_outline],
                                          np.array([[marker_id]]))
                cv2.circle(overlay, center_px, 5, (0, 0, 255), -1)
                cv2.line(overlay, (int(optical_cx), int(optical_cy)),
                         center_px, (0, 255, 0), 2)
                cv2.putText(overlay, f"ID: {marker_id}   Quadrant: {quadrant}",
                            (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(overlay,
                            f"Raw: {raw_angle_deg:+.2f}  Filtered: {angle_deg:+.2f} deg",
                            (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                break

        if not target_found:
            cv2.putText(overlay, "Target marker not found", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            # On reacquisition, don't let the low-pass filter drag in old data.
            filtered_angle = None

        cv2.imshow("ArUco Bearing", overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    camera.release()
    cv2.destroyAllWindows()
    Arduino_bus.close()

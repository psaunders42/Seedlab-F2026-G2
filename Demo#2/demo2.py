import cv2
import cv2.aruco as aruco
import numpy as np
import pickle
import os
import time
import threading
import queue
from smbus import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# --------------------------------------------------
# CALIBRATION - RUN calibration_improved.py FIRST
# --------------------------------------------------
#with open("calibration_improved.pkl", "rb") as f:
    #camera_matrix, dist_coeffs = pickle.load(f)

#optical_cx = float(camera_matrix[0, 2])
#optical_cy = float(camera_matrix[1, 2])

# --------------------------------------------------
# SETTINGS
# --------------------------------------------------
TARGET_ID = 0
ARUCO_DICT = aruco.DICT_6X6_50
MARKER_SIZE_IN = 2.0
DEADBAND_PX = 15
ANGLE_THRESHOLD = 0.25          # Arduino change threshold (NOT precision)
ANGLE_ALPHA = 0.25              # Smoothed fraction of newest angle
DIST_ALPHA= 0.25

SEND_PERIOD_S = 0.1
ARD_ADDR = 8
CMD_TARGET = 0x03

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
            angle,dist = lcd_queue.get()
            lcd.message = f"x: {angle:.2f} deg d: {dist:.2f}".ljust(16)
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

#Default setting
FRAME_W = 1280
FRAME_H = 720

#CAlibrated#
CALIBRATION_FILE = "calibration_improved.pkl"
Assumed_hfov_deg = 60.0
SIMULATE = False

if os.path.exists(CALIBRATION_FILE):
    with open("calibration_improved.pkl", "rb") as f:
        camera_matrix, dist_coeffs = pickle.load(f)

    optical_cx = float(camera_matrix[0, 2])
    optical_cy = float(camera_matrix[1, 2])
    calibrated = True
    print("Using calibration")
else:
    fx = (FRAME_W/ 2.0)/ np.tan(np.radians(Assumed_hfov_deg /2.0))
    camera_matrix = np.array([[fx,0.0,FRAME_W /2.0],
                              [0.0,fx,FRAME_H / 2.0],
                              [0.0,0.0,1.0]], dtype =np.float64)
    dist_coeffs = np.zeros((5,1), dtype = np.float64)
    calibrated = False
    print("no calibration")
optical_cx = float(camera_matrix[0,2])
optical_cy = float(camera_matrix[1,2])
focal_px = float(camera_matrix[0,0])

#set up
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
# I2C
# --------------------------------------------------
Arduino_bus = SMBus(1)

def send_target(angle_deg, dist_in):
    angle_int = int(round(angle_deg *100))
    angle_int = max(-32768, min(32767, angle_int))
    dist_int = int(round(dist_in *10))
    dist_int = max(0, min(65535, dist_int))
    payload = [
        angle_int & 0xFF, (angle_int>>8) & 0xFF,
        dist_int &0xFF, (dist_int>>8) & 0xFF]
    Arduino_bus.write_i2c_block_data(ARD_ADDR, CMD_TARGET, payload)

# --------------------------------------------------
# Aruco detect
# --------------------------------------------------

aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
if hasattr(aruco, "DetectorParameters"):
    detector_params = aruco.DetectorParameters()
else:
    detector_params = aruco.DetectorParameters_create()
detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
 
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

                #angle
                rays = cv2.undistortPoints(
                    raw_corners.reshape(-1, 1, 2), camera_matrix, dist_coeffs
                ).reshape(-1, 2)
                marker_ray_x = float(rays[:, 0].mean())
                raw_angle_deg = float(np.degrees(np.arctan2(-marker_ray_x, 1.0)))

                #distnace
                undist_px = cv2.undistortPoints(
                    raw_corners.reshape(-1,1,2), camera_matrix, dist_coeffs,
                    P = camera_matrix
                    ).reshape(-1,2)
                sides = [np.linalg.norm(undist_px[i] - undist_px[(i + 1) % 4])
                         for i in range(4)]
                side_px = float(np.mean(sides))
                raw_dist_in = focal_px * MARKER_SIZE_IN / side_px
                
                if filtered_angle is None:
                    filtered_angle = raw_angle_deg
                    filtered_dist = raw_dist_in
                else:
                    filtered_angle += ANGLE_ALPHA * (raw_angle_deg - filtered_angle)
                    filtered_dist += DIST_ALPHA * (raw_dist_in - filtered_dist)

                angle_deg = filtered_angle
                dist_in = filtered_dist

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
                        Arduino_bus.write_byte_data(ARD_ADDR, CMD_TARGET, goal_byte)
                        last_quadrant = quadrant
                        print("Quadrant:", quadrant, "sent:", goal_byte)
                    except OSError as e:
                        print("Failed to send quadrant:", e)

                # --------------------------------------------------
                # ANGLE TO ARDUINO - WORKS EVEN IF QUADRANT IS UNKNOWN
                # --------------------------------------------------
                now = time.time()
                #angle_changed = (last_sent_angle is None or
                #                 abs(angle_deg - last_sent_angle) >= ANGLE_THRESHOLD)
                if now - last_ard_angle_time >= SEND_PERIOD_S:
                    #angle_int = int(round(angle_deg * 100))
                    try:
                        send_target(angle_deg, dist_in)
                        last_send_time = now
                        print(f"sent angle = {angle_deg:.2f} deg dist={dist_in:.2f}")
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

"""Python code -- performance-fixed version of demo2.py

Changes vs. the original (search for "PERF"):
  1. LCD no longer blocks the main loop: the shared bus_lock is gone, and the
     LCD is only rewritten when its text actually changes (max 2x per second).
  2. Camera frames come from a background reader that keeps only the newest
     frame, so processing never falls behind on stale buffered frames (lag).
  3. Negotiated FOURCC / FPS are printed so you can confirm MJPG is active.
  4. Optional: detect markers on a downscaled image (DETECT_SCALE), optional
     smaller preview window (DISPLAY_SCALE), FPS readout on the overlay.
"""
import cv2
import cv2.aruco as aruco
import numpy as np
import pickle
import time
import threading
import queue
from collections import deque
from smbus import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

TARGET_ID = 0
ARUCO_DICT = aruco.DICT_6X6_50
MARKER_SIZE_IN = 2.0  # Measure outer black-square width, excluding white margin.
DEADBAND_PX = 15
ANGLE_ALPHA = 0.25
DIST_ALPHA = 0.25
SEND_PERIOD_S = 0.1
ARD_ADDR = 8
CMD_TARGET = 0x03
ARROW_NONE, ARROW_LEFT, ARROW_RIGHT = 0, 1, 2
ARROW_MIN_PIXELS = 15
ARROW_MIN_FRACTION = 0.03
ARROW_HISTORY = 5
ARROW_VOTES_NEEDED = 3

# PERF: tuning knobs
LCD_PERIOD_S = 0.5      # LCD writes over the I2C expander are very slow; keep this >= 0.5
DETECT_SCALE = 1.0      # try 0.5 for a big speedup (shorter max detection range)
DISPLAY_SCALE = 1.0     # try 0.5 if viewing over VNC / SSH X-forwarding
SHOW_WINDOW = True      # False = run headless (fastest)

lcd_queue = queue.Queue(maxsize=1)
# PERF: no shared bus_lock. The Arduino (addr 8) and the LCD expander are
# different devices, and the Linux I2C driver already makes each individual
# transaction atomic. The old lock let a ~0.3-0.5 s LCD rewrite block
# send_target(), stalling the camera loop several times per second.


class LatestFrameReader:
    """PERF: grabs frames on a background thread and keeps only the newest.

    Without this, V4L2 queues several frames; whenever processing is slower
    than the camera, every frame you process is old -> visible lag.
    """

    def __init__(self, camera):
        self.camera = camera
        self.cond = threading.Condition()
        self.frame = None
        self.seq = 0
        self.failed = False
        self.stopped = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stopped:
            ok, frame = self.camera.read()
            with self.cond:
                if not ok:
                    self.failed = True
                    self.cond.notify_all()
                    return
                self.frame = frame
                self.seq += 1
                self.cond.notify_all()

    def read(self, last_seq, timeout=1.0):
        """Return (frame, seq) newer than last_seq, or (None, last_seq) on timeout."""
        with self.cond:
            self.cond.wait_for(lambda: self.failed or self.seq != last_seq, timeout)
            if self.seq != last_seq:
                return self.frame, self.seq
            return None, last_seq

    def stop(self):
        self.stopped = True
        self.thread.join(timeout=1.0)


def lcd_display():
    try:
        time.sleep(1)
        lcd = character_lcd.Character_LCD_RGB_I2C(board.I2C(), 16, 2)
        lcd.clear()
        lcd.color = [0, 0, 100]
        last_text = None
        while True:
            found, angle, distance = lcd_queue.get()
            if found:
                # PERF: 1 decimal place so the text doesn't change every frame
                line1 = f"A:{angle:+.1f} deg"
                line2 = f"D:{distance:.1f} in"
            else:
                line1, line2 = "No marker", "Searching..."
            text = line1.ljust(16)[:16] + "\n" + line2.ljust(16)[:16]
            if text == last_text:
                continue  # PERF: skip the slow rewrite if nothing changed
            lcd.message = text
            last_text = text
    except Exception as exc:
        print("LCD thread error:", exc)


def lcd_push(item):
    try:
        lcd_queue.put_nowait(item)
    except queue.Full:
        try:
            lcd_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            lcd_queue.put_nowait(item)
        except queue.Full:
            pass


def send_target(bus, angle_deg, dist_in, arrow=ARROW_NONE):
    angle_int = max(-32768, min(32767, int(round(angle_deg * 100))))
    dist_int = max(0, min(65535, int(round(dist_in * 10))))
    payload = [
        angle_int & 0xFF, (angle_int >> 8) & 0xFF,
        dist_int & 0xFF, (dist_int >> 8) & 0xFF,
        int(arrow),
    ]
    # Command plus five data bytes: angle, distance, arrow.
    bus.write_i2c_block_data(ARD_ADDR, CMD_TARGET, payload)


def detect_arrow(frame, center_x, center_y, side_px):
    """Color convention: green=left, red=right; does not infer arrow shape."""
    h, w = frame.shape[:2]
    green_count = red_count = total_pixels = 0
    for direction in (-1, 1):
        xa = center_x + direction * 0.6 * side_px
        xb = center_x + direction * 3.0 * side_px
        x0 = max(0, min(w, int(min(xa, xb))))
        x1 = max(0, min(w, int(max(xa, xb))))
        y0 = max(0, min(h, int(center_y - 0.6 * side_px)))
        y1 = max(0, min(h, int(center_y + 0.6 * side_px)))
        if x1 - x0 < 4 or y1 - y0 < 4:
            continue
        hsv = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv, (40, 80, 50), (85, 255, 255))
        red = cv2.bitwise_or(
            cv2.inRange(hsv, (0, 100, 70), (10, 255, 255)),
            cv2.inRange(hsv, (170, 100, 70), (180, 255, 255)))
        green_count += cv2.countNonZero(green)
        red_count += cv2.countNonZero(red)
        total_pixels += (x1 - x0) * (y1 - y0)
    minimum = max(ARROW_MIN_PIXELS, ARROW_MIN_FRACTION * total_pixels)
    if green_count >= minimum and green_count > 1.5 * red_count:
        return ARROW_LEFT
    if red_count >= minimum and red_count > 1.5 * green_count:
        return ARROW_RIGHT
    return ARROW_NONE


def voted_arrow(votes):
    left, right = votes.count(ARROW_LEFT), votes.count(ARROW_RIGHT)
    if left >= ARROW_VOTES_NEEDED and left > right:
        return ARROW_LEFT
    if right >= ARROW_VOTES_NEEDED and right > left:
        return ARROW_RIGHT
    return ARROW_NONE


SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


def detect(gray, aruco_dict, detector_params):
    """PERF: optionally detect on a downscaled image, then refine at full res."""
    if DETECT_SCALE == 1.0:
        return aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)
    small = cv2.resize(gray, None, fx=DETECT_SCALE, fy=DETECT_SCALE,
                       interpolation=cv2.INTER_AREA)
    corners, ids, rejected = aruco.detectMarkers(
        small, aruco_dict, parameters=detector_params)
    refined = []
    for c in corners:
        full = (c.reshape(-1, 1, 2) / DETECT_SCALE).astype(np.float32)
        cv2.cornerSubPix(gray, full, (5, 5), (-1, -1), SUBPIX_CRITERIA)
        refined.append(full.reshape(1, 4, 2))
    return refined, ids, rejected


def fourcc_str(value):
    v = int(value)
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def main():
    with open("calibration_improved.pkl", "rb") as calibration:
        camera_matrix, dist_coeffs = pickle.load(calibration)
    optical_cx = float(camera_matrix[0, 2])
    optical_cy = float(camera_matrix[1, 2])
    focal_px = float((camera_matrix[0, 0] + camera_matrix[1, 1]) / 2.0)

    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
    if hasattr(aruco, "DetectorParameters"):
        detector_params = aruco.DetectorParameters()
    else:
        detector_params = aruco.DetectorParameters_create()
    detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

    camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    bus = None
    reader = None
    try:
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # PERF: some V4L2 drivers reset the pixel format when the size changes;
        # set MJPG again so we don't silently fall back to YUYV (~10 fps at 720p).
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FPS, 30)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # PERF: minimal driver queue
        if not camera.isOpened():
            raise RuntimeError("Could not open /dev/video0")
        resolution = (int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                      int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        # PERF: confirm what the camera actually agreed to
        print("Camera:", resolution,
              "format:", fourcc_str(camera.get(cv2.CAP_PROP_FOURCC)),
              "fps:", camera.get(cv2.CAP_PROP_FPS))
        if resolution != (1280, 720):
            raise RuntimeError("Capture resolution differs from calibration!")
        bus = SMBus(1)
        threading.Thread(target=lcd_display, daemon=True).start()
        reader = LatestFrameReader(camera)  # PERF: replaces time.sleep(2) + camera.read()
        frame_seq = 0
        last_send_time = last_lcd_time = 0.0
        filtered_angle = filtered_dist = None
        arrow_votes = deque(maxlen=ARROW_HISTORY)
        last_quadrant = None
        fps = 0.0
        last_loop_time = time.monotonic()

        while True:
            frame, frame_seq = reader.read(frame_seq)
            if reader.failed:
                print("Camera frame failed; exiting")
                break
            if frame is None:
                continue  # no new frame yet
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detect(gray, aruco_dict, detector_params)
            overlay = frame.copy() if SHOW_WINDOW else frame
            h, w = frame.shape[:2]
            if SHOW_WINDOW:
                cv2.line(overlay, (int(optical_cx), 0),
                         (int(optical_cx), h), (255, 255, 255), 1)
                cv2.line(overlay, (0, int(optical_cy)),
                         (w, int(optical_cy)), (255, 255, 255), 1)
            arrow = ARROW_NONE
            target_found = False
            angle_deg = dist_in = 0.0

            if ids is not None:
                for marker_outline, marker_id in zip(corners, ids.flatten()):
                    if int(marker_id) != TARGET_ID:
                        continue
                    raw_corners = marker_outline.reshape(4, 2).astype(np.float32)
                    center_x, center_y = raw_corners.mean(axis=0)
                    center_px = (int(round(center_x)), int(round(center_y)))
                    dx, dy = center_x - optical_cx, center_y - optical_cy
                    rays = cv2.undistortPoints(
                        raw_corners.reshape(-1, 1, 2), camera_matrix,
                        dist_coeffs).reshape(-1, 2)
                    marker_ray_x = float(rays[:, 0].mean())
                    raw_angle_deg = float(np.degrees(np.arctan2(-marker_ray_x, 1.0)))

                    undist_px = cv2.undistortPoints(
                        raw_corners.reshape(-1, 1, 2), camera_matrix,
                        dist_coeffs, P=camera_matrix).reshape(-1, 2)
                    sides = [np.linalg.norm(undist_px[i] - undist_px[(i + 1) % 4])
                             for i in range(4)]
                    side_px = float(np.mean(sides))
                    if not np.isfinite(side_px) or side_px <= 0:
                        continue
                    # Approximate depth from apparent size, then horizontal range.
                    depth_in = focal_px * MARKER_SIZE_IN / side_px
                    raw_dist_in = depth_in * float(np.hypot(1.0, marker_ray_x))
                    if not np.isfinite(raw_angle_deg) or not np.isfinite(raw_dist_in):
                        continue
                    target_found = True
                    raw_side_px = float(np.mean([
                        np.linalg.norm(raw_corners[i] - raw_corners[(i + 1) % 4])
                        for i in range(4)]))
                    arrow_votes.append(detect_arrow(
                        frame, float(center_x), float(center_y), raw_side_px))
                    arrow = voted_arrow(arrow_votes)
                    if filtered_angle is None:
                        filtered_angle = raw_angle_deg
                    else:
                        filtered_angle += ANGLE_ALPHA * (raw_angle_deg - filtered_angle)
                    if filtered_dist is None:
                        filtered_dist = raw_dist_in
                    else:
                        filtered_dist += DIST_ALPHA * (raw_dist_in - filtered_dist)
                    angle_deg, dist_in = filtered_angle, filtered_dist

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
                    last_quadrant = quadrant
                    if SHOW_WINDOW:
                        arrow_text = {0: "UNKNOWN", 1: "LEFT", 2: "RIGHT"}[arrow]
                        cv2.putText(overlay, f"Arrow: {arrow_text}", (15, 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 0), 2)
                        aruco.drawDetectedMarkers(overlay, [marker_outline],
                                                  np.array([[marker_id]]))
                        cv2.circle(overlay, center_px, 5, (0, 0, 255), -1)
                        cv2.line(overlay, (int(optical_cx), int(optical_cy)),
                                 center_px, (0, 255, 0), 2)
                        cv2.putText(overlay, f"ID:{marker_id} Quadrant:{quadrant}",
                                    (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 2)
                        cv2.putText(overlay,
                                    f"Angle:{angle_deg:+.2f} deg Range:{dist_in:.2f} in",
                                    (15, 60), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 2)
                    break

            if not target_found:
                arrow_votes.clear()
                arrow = ARROW_NONE
                filtered_angle = filtered_dist = None
                last_quadrant = None
                if SHOW_WINDOW:
                    cv2.putText(overlay, "Target marker not found", (15, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255), 2)

            # PERF: measure real processing rate
            now = time.monotonic()
            dt = now - last_loop_time
            last_loop_time = now
            if dt > 0:
                fps = 1.0 / dt if fps == 0.0 else fps + 0.1 * (1.0 / dt - fps)

            # Always report, including (0, 0) when the target is absent.
            if now - last_send_time >= SEND_PERIOD_S:
                last_send_time = now
                try:
                    send_target(bus, angle_deg, dist_in, arrow)
                    print(f"fps={fps:5.1f} found={target_found} angle={angle_deg:+.2f} "
                          f"range_in={dist_in:.2f} arrow={arrow}")
                except OSError as exc:
                    print("Failed to send target:", exc)
            if now - last_lcd_time >= LCD_PERIOD_S:
                lcd_push((target_found, angle_deg, dist_in))
                last_lcd_time = now
            if SHOW_WINDOW:
                cv2.putText(overlay, f"FPS: {fps:.1f}", (w - 150, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
                if DISPLAY_SCALE != 1.0:
                    overlay = cv2.resize(overlay, None, fx=DISPLAY_SCALE,
                                         fy=DISPLAY_SCALE,
                                         interpolation=cv2.INTER_NEAREST)
                cv2.imshow("ArUco Bearing", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        if reader is not None:
            reader.stop()
        camera.release()
        cv2.destroyAllWindows()
        if bus is not None:
            bus.close()


if __name__ == "__main__":
    main()

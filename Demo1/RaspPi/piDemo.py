import cv2
import cv2.aruco as aruco
import numpy as np
from time import sleep
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import queue
from smbus import SMBus
import pickle
import struct

## --------------------------------------------------
# CALIBRATION
# --------------------------------------------------
with open("calibration.pkl","rb") as f:
    camera_matrix, dist_coeffs = pickle.load(f)

focal_length = camera_matrix[0,0]
optical_cx = camera_matrix[0,2]
optical_cy= camera_matrix[1,2]


# --------------------------------------------------
# SETTINGS
# --------------------------------------------------

TARGET_ID = 0

ARUCO_DICT = aruco.DICT_6X6_50

DEADBAND_PX = 15

H_FOV = 60.0

ANGLE_THRESHOLD = 0.25

QUADRANT_BITS = {
    "NE": (0, 0),
    "NW": (0, 1),
    "SW": (1, 1),
    "SE": (1, 0),
}

# --------------------------------------------------
# LCD
# --------------------------------------------------
lcdc= 16
lcdr = 2
i2c = board.I2C()

LCDqueue = queue.Queue()
def LCDdisplay():
    sleep(1)
    try:
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c,lcdc,lcdr)
        lcd.clear()
        sleep(0.05)
        lcd.color = [0,0,100]
        while True:
            #location = LCDqueue.get()
            #lcd.clear()
            #lcd.message = "Desied Location:\n" + str(location)
            angle = LCDqueue.get()
            display_text = f"x: {angle:.2f} deg"
            lcd.message = display_text.ljust(16)
        #sleep(0.1)
    except Exception as e:
        print(f"LCD thread error: {e}")

LCDthread = threading.Thread(target = LCDdisplay, args=(),daemon=True)
LCDthread.start()

# --------------------------------------------------
# Ardunio
# --------------------------------------------------

ARD_ADDR = 8
Arduino_bus = SMBus(1)

CMD_QUADRANT = 0x01
CMD_ANGLE = 0x02


# --------------------------------------------------
# ARUCO DICTIONARY
# --------------------------------------------------

aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)

# --------------------------------------------------
# CAMERA SETUP
# --------------------------------------------------

camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

camera.set(
    cv2.CAP_PROP_FOURCC,
    cv2.VideoWriter_fourcc(*"MJPG")
)

camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 30)


print("Camera opened:", camera.isOpened())
print("Width:", camera.get(cv2.CAP_PROP_FRAME_WIDTH))
print("Height:", camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("FPS:", camera.get(cv2.CAP_PROP_FPS))


# Give camera time to stabilize
sleep(2)


# --------------------------------------------------
# VARIABLES
# --------------------------------------------------

last_quadrant = None
last_lcd_time = time.time()

last_ard_angle_time = time.time()
last_sent_angle = None

# --------------------------------------------------
# MAIN LOOP
# --------------------------------------------------

while True:
    # Get image from camera
    ret, raw_frame = camera.read()

    if not ret:
        print("Could not read image from camera")
        break
    frame = cv2.undistort(raw_frame,camera_matrix, dist_coeffs)
    # Get actual frame size
    frame_height, frame_width = frame.shape[:2]

    #caculates focal length in pixels based pn horz FOV
    #focal_length = (frame_width/2)/np.tan(np.radians(H_FOV/2))

    # Find center lines
    mid_x = int(optical_cx)
    mid_y = int(optical_cy)

    # Convert to grayscale
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    # --------------------------------------------------
    # DETECT ARUCO MARKERS
    # --------------------------------------------------

    corners, ids, rejected = aruco.detectMarkers(grey,aruco_dict)

    # Make display image
    overlay = frame.copy()

    # Draw vertical and horizontal center lines
    cv2.line(
        overlay,
        (mid_x, 0),
        (mid_x, frame_height),
        (255, 255, 255),
        1)

    cv2.line(
        overlay,
        (0, mid_y),
        (frame_width, mid_y),
        (255, 255, 255),
        1)


    # --------------------------------------------------
    # CHECK IF ANY MARKERS WERE FOUND
    # --------------------------------------------------

    if ids is not None:

        ids = ids.flatten()

        for outline, marker_id in zip(corners, ids):

            # Ignore markers that aren't our target
            if marker_id != TARGET_ID:
                continue


            # --------------------------------------------------
            # GET MARKER CORNERS
            # --------------------------------------------------

            markerCorners = outline.reshape((4, 2))


            # --------------------------------------------------
            # FIND CENTER OF MARKER
            # --------------------------------------------------

            center_x = int(np.mean(markerCorners[:, 0]))
            center_y = int(np.mean(markerCorners[:, 1]))


            # --------------------------------------------------
            # DETERMINE QUADRANT
            # --------------------------------------------------

            dx = center_x - optical_cx
            dy = center_y - optical_cy

            angle_rad = np.arctan(-dx/focal_length)
            angle_deg = np.degrees(angle_rad)


            # Deadband:
            # If marker is very close to a center line,
            # keep the previous quadrant.

            if abs(dx) <= DEADBAND_PX or abs(dy) <= DEADBAND_PX:

                quadrant = last_quadrant

            else:

                if dx > 0 and dy < 0:
                    quadrant = "NE"

                elif dx < 0 and dy < 0:
                    quadrant = "NW"

                elif dx < 0 and dy > 0:
                    quadrant = "SW"

                else:
                    quadrant = "SE"


            # --------------------------------------------------
            # DRAW MARKER
            # --------------------------------------------------

            overlay = aruco.drawDetectedMarkers(
                overlay,
                [outline],
                np.array([[marker_id]])
            )


            # Draw center of marker
            cv2.circle(
                overlay,
                (center_x, center_y),
                5,
                (0, 0, 255),
                -1
            )


            # Draw line from image center to marker
            cv2.line(
                overlay,
                (mid_x, mid_y),
                (center_x, center_y),
                (0, 255, 0),
                2
            )


            # --------------------------------------------------
            # GET WHEEL BITS
            # --------------------------------------------------

            if quadrant is not None:

                left_bit, right_bit = QUADRANT_BITS[quadrant]

                # Pack the two bits:
                # left = most significant bit
                # right = least significant bit
                goal_byte = (left_bit << 1) | right_bit

            
                # --------------------------------------------------
                # PRINT ONLY WHEN QUADRANT CHANGES
                # --------------------------------------------------

                if quadrant != last_quadrant:

                    print("--------------------------")
                    print("Marker ID:", marker_id)
                    print("Center:", center_x, center_y)
                    print("Quadrant:", quadrant)
                    print("Left:", left_bit)
                    print("Right:", right_bit)
                    print("Goal byte:", goal_byte)
                    print("--------------------------")

                    
                    try:
                        Arduino_bus.write_byte_data(ARD_ADDR, CMD_QUADRANT, goal_byte)
                        print("Byte sent to Arduino: ", goal_byte)
                        
                    except OSError as e:
                        print(f"Failed to send to Ardunio: {e}")
                                        
                    last_quadrant = quadrant
                
                   #LCDqueue.put([left_bit, right_bit])
                # --------------------------------------------------
                # UPDATE ardnio
                # --------------------------------------------------
                current_time = time.time()
                angle_changed = (last_sent_angle is None or
                                abs(angle_deg - last_sent_angle)>=
                                    ANGLE_THRESHOLD)
                
                if (current_time -last_ard_angle_time >0.1) and angle_changed: 
                    try:
                        angle_int = int(round(angle_deg*100))
                        packed_data = struct.pack('<h', angle_int)
                        word_data = angle_int & 0xFFFF
                        
                        Arduino_bus.write_word_data(ARD_ADDR, CMD_ANGLE, word_data)
                        last_ard_angle_time = current_time
                        last_sent_angle = angle_deg
                        print(f"Angle sent to Arduino: {angle_deg:.2f}")
                    except OSError as e:
                        print(f"Failed to send angle to Arduino: {e}")
                # --------------------------------------------------
                # UPDATE LCD
               # --------------------------------------------------

                if current_time -last_lcd_time >0.25: # updates 4 times a sec
                    #flush old data
                    while not LCDqueue.empty():
                        try:
                            LCDqueue.get_nowait()
                        except queue.Empty:
                            break

                    LCDqueue.put(angle_deg)
                    last_lcd_time = current_time

                # --------------------------------------------------
                # DISPLAY INFORMATION
                # --------------------------------------------------

                cv2.putText(
                    overlay,
                    "ID: " + str(marker_id),
                    (center_x - 30, center_y - 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2
                )

                cv2.putText(
                    overlay,
                    "Quadrant: " + quadrant,
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,(0, 255, 0),2)

                cv2.putText(
                    overlay,
                    "L: {}  R: {}".format(left_bit, right_bit),
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),2)

                cv2.putText(
                    overlay,
                    "Goal: {}".format(goal_byte),
                    (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,(0, 255, 0),2)

                cv2.putText(
                    overlay,
                    "Center: ({}, {})".format(center_x, center_y),
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,(255, 255, 255),2)

                cv2.putText(
                    overlay,
                    f"Angle: {angle_deg:.2f} deg",
                    (20,150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,(0,255,255),2)
    # --------------------------------------------------
    # NO MARKER FOUND
    # --------------------------------------------------

    else:

        cv2.putText(
            overlay,
            "Target marker not found",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,(0, 0, 255),2)

    # --------------------------------------------------
    # DISPLAY IMAGE
    # --------------------------------------------------

    cv2.imshow("ArUco Quadrant Detection", overlay)

    # Press q to quit
    k = cv2.waitKey(1) & 0xFF

    if k == ord('q'):
        break

# --------------------------------------------------
# CLEAN UP
# --------------------------------------------------

camera.release()
cv2.destroyAllWindows()


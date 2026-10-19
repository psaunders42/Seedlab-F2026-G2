import cv2
import os

# Create the folder so cv2.imwrite doesn't silently fail
if not os.path.exists('images'):
    os.makedirs('images')

# Fixed camera port
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

# Enforce 640x480 resolution per Demo 1 requirements
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

num = 0

print("Press 's' to save an image. Press 'ESC' to quit.")

while cap.isOpened():
    succes, img = cap.read()
    if not succes:
        break
        
    cv2.imshow('Img', img)
    k = cv2.waitKey(5)

    if k == 27: # ESC key
        break
    elif k == ord('s'): 
        cv2.imwrite('images/img' + str(num) + '.png', img)
        print("Image saved: img" + str(num) + ".png")
        num += 1

cap.release()
cv2.destroyAllWindows()

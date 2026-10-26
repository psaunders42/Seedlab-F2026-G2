# Demo 1

## Project Description
This project is two separate demos, one for CV and one for controls. CV focuses on calibrating a camera to get rid of the distortion error, so it can successfully
detect the ArUco markers with as much accuracy as possible. Controls focuses on the robot being able to travel a certain given distance, as well as rotate to a given angle and travel to that specific point.

## Hardware Initialization
### CV
Connect a camera to the Raspberry Pi as well as plug the Raspberry Pi into a monitor. Use "getImages_improved.py" to capture 20-30 images. These images contain a printed-out chessboard, taped to a flat surface, positioned in different places of the frame, at different angles & orientations; make sure to get all corners and edges. Then upload to "calibration_improved.py"; this will get the pictures from "getImages_improved.py" and use them to calculate the distortion and error. Do note: the lower the "px RMS" the better. You are encouraged to delete the ones greater than 1.0 or even 0.5 for best results. Continue the loop until you are happy with the overall error. This will return a camera matrix in the form of a plk file that "demo1_improved" will read and use these values to fix the distortion to the best of its ability. From there, firmly connect an LCD screen to the Pi and run the code. 

### Controls



## How to Run
### CV
Once following the initialization, you can run "demo1_improved.py." Upon running "demo1_improved.py" on the monitor, a screen should pop up, get an ArUcoMarker, and, to the best of your ability, find where zero is. This is done by moving or angling the camera around until the mointor/LCD are as close to zero as possible; it will be very hard to get it exactly on zero, so +/- 0.1 is okay. Then you are free to move around the arucMarker, and it will detect the angles away from what you zeroed. 


## Files
getImages_improved.py - used for collecting 20-30 images for calibration
calibration_improved.py - used for caculating the calibration matrix
demo1_improved - used for caculating the angles and demo1


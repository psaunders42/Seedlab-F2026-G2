# Demo 2

## Project Description
This project basically combines the code from demo 1 CV and Controls. In addition to the CV
code from demo 1, we added a distance detector. So the camera will detect the distance
and angle then send it to the arduino and the robot will follow the instructions from the CV 
data sent.

## Hardware Initialization
### CV
~from demo 1 continues over go back to demo 1 if need to calibrate camera
Connect a camera to the Raspberry Pi and plug the Raspberry Pi into a monitor. 
Use "getImages_improved.py" to capture 20-30 images. These images contain a printed-out chessboard, 
taped to a flat surface, positioned in different places of the frame, at different angles & orientations; 
make sure to get all corners and edges. Then upload to "calibration_improved.py"; 
This will get the pictures from "getImages_improved.py" and use them to calculate the distortion and error.
Do note: the lower the "px RMS" the better. You are encouraged to delete the ones greater than 1.0 or even 0.5 for best results. 
Continue the loop until you are happy with the overall error. This will return a camera matrix in the form of a plk file that
"demo1_improved" will read and use these values to fix the distortion to the best of its ability. From there, 
firmly connect an LCD screen to the Pi and run the code. 

~New
Rest the PI somewhere on the robot where it is secure. Connect the Arduino to the PI thru using a USB-A to USB-C. And connect the PI 
to a power source that is portable, like a portable battery. 

### Controls
Connect the lithium ion battery to the voltage monitor, motor shield, 
and Arduino circuit combination, but keep the switch off. When the correctly updated Demo 1 
Arduino code is uploaded to the Arduino via the USB connection, disconnect the USB and install a jumper cable between 
the Arduino VIN and Shield VOUT pins. It is critical to have no power provided to the Arduino via the barrel jack while this 
jumper is installed. After the code is uploaded and the jumper is installed, the switch that connects the battery to the circuit 
can be toggled and the robot will execute the movement defined in the code.


## How to Run
### CV
We do not need to worry about zero-ing the camera perfectly since the robot will be moving; as long as it looks good on the marker
and is pretty lined up, that will work. Once the hardware is initialized, connect the Pi to a remote connection and run "Demo2.py"
On your screen, the camera view should appear, and it is ready for the Controls part.

### Controls
In the Demo 2 Arduino code, you do not need to update any floating variables since the camera will be the data for that
So, plug the robot into the computer and upload the Arduino code Demo2.ino, and the robot will start rotating into motion by 
toggling the switch to provide power from the lithium-ion battery. The robot will navigate by rotating in pulses
Once the robot finds an ArUco marker, it will go the distance the CV code told it to and look at the arrows to see which way it should
rotate. 

## Files
Demo2.ino
demo2.py


import numpy as np
import cv2 as cv
import glob
import pickle

# UPDATE THIS if your chessboard does not have 9x6 inner corners
chessboardSize = (9,6)
frameSize = (640,480)

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

# UPDATE THIS to the physical size of one square on your printed board
size_of_chessboard_squares_mm = 20
objp = objp * size_of_chessboard_squares_mm

objpoints = [] 
imgpoints = [] 

# Fixed path to match where the first script saved the pictures
images = glob.glob('images/*.png')

if len(images) == 0:
    print("Error: No images found in the 'images/' folder.")
    exit()

for image in images:
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

# CALIBRATION
print("Calibrating...")
ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

print("Camera Matrix:\n", cameraMatrix)
print("Distortion Coefficients:\n", dist)

# Save the results
pickle.dump((cameraMatrix, dist), open( "calibration.pkl", "wb" ))
print("Calibration saved to calibration.pkl!")

# Reprojection Error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print("Total error: {}".format(mean_error/len(objpoints)))



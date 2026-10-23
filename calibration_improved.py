import glob
import pickle
import cv2 as cv
import numpy as np

# --------------------------------------------------
# SETTINGS
# --------------------------------------------------
CHESSBOARD_SIZE = (8, 5)        # INNER corners: 9 across x 6 down
SQUARE_SIZE_MM = 31.75          # Measure your actual printed square width
IMAGE_FILES = sorted(glob.glob("images_new/*.png"))
OUTPUT_FILE = "calibration_improved.pkl"

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 0.001)

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

objpoints = []
imgpoints = []
used_files = []
frame_size = None

if not IMAGE_FILES:
    raise RuntimeError("No PNG images in images_new/. Run getImages_improved.py first.")

# --------------------------------------------------
# DETECT CHESSBOARD CORNERS
# --------------------------------------------------
for filename in IMAGE_FILES:
    img = cv.imread(filename)
    if img is None:
        print("Unable to read:", filename)
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    size = (gray.shape[1], gray.shape[0])
    if frame_size is None:
        frame_size = size
    if size != frame_size:
        print("SKIPPING mismatched image size:", filename, size)
        continue

    found, corners = cv.findChessboardCorners(
        gray, CHESSBOARD_SIZE,
        cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_NORMALIZE_IMAGE
    )
    if not found:
        print("SKIPPING (no 9x6 board detected):", filename)
        continue

    # Use the REFINED corners in the calibration, not the initial estimates.
    refined = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    objpoints.append(objp.copy())
    imgpoints.append(refined.copy())
    used_files.append(filename)

if len(objpoints) < 10:
    raise RuntimeError(f"Only {len(objpoints)} valid chessboard images; collect more.")

print(f"Using {len(objpoints)} / {len(IMAGE_FILES)} images at {frame_size}.")

# --------------------------------------------------
# CALIBRATE
# --------------------------------------------------
# For a normal camera with square pixels, fx and fy should have a 1:1 ratio.
# Do not use this constraint if your frames are stretched or non-square-pixel.
initial_K = cv.initCameraMatrix2D(objpoints, imgpoints, frame_size, 1.0)
flags = (cv.CALIB_USE_INTRINSIC_GUESS |
         cv.CALIB_FIX_ASPECT_RATIO |
         cv.CALIB_FIX_K3)

rms, camera_matrix, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, frame_size, initial_K, None, flags=flags
)

print("\nOpenCV OVERALL RMS reprojection error (pixels):", rms)
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist)

# --------------------------------------------------
# PER-IMAGE RMS ERRORS (PIXELS)
# --------------------------------------------------
for filename, obj, measured, rvec, tvec in zip(
    used_files, objpoints, imgpoints, rvecs, tvecs
):
    projected, _ = cv.projectPoints(obj, rvec, tvec, camera_matrix, dist)
    # Correct per-image RMS: sqrt(sum of squared 2-D errors / corner count)
    per_image_rms = cv.norm(measured, projected, cv.NORM_L2) / np.sqrt(len(projected))
    print(f"  {filename}: {per_image_rms:.3f} px RMS")

if rms > 1.0:
    print("WARNING: RMS > 1 px. Check focus, board flatness, and viewpoint variety.")

with open(OUTPUT_FILE, "wb") as f:
    pickle.dump((camera_matrix, dist), f)
print("Saved:", OUTPUT_FILE)

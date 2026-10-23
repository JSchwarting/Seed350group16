"""
File: calibrate.py
Author: Isha Singh
Date: October 2026
Description: 
This script processes the saved chessboard images to calculate the camera's 
lens distortion and focal length. It saves the mathematical results into a 
file named 'calibration_data.npz' so the main ArUco detection script can 
load it to calculate highly accurate marker angles.
"""

import cv2
import numpy as np
import glob

# ================= SETTINGS =================
# The number of internal corners on the printed chessboard (width, height)
# Note: This is the points where black squares touch, not the number of squares!
CHESSBOARD_SIZE = (9, 6) 

# Prepare physical 3D points for a perfect real-world chessboard (0,0,0 then 1,0,0 etc.)
# Because we printed it on flat paper, the Z (depth) coordinate is always 0
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

# Lists to store the points from all the pictures we took
objpoints = [] # Perfect 3D real-world points
imgpoints = [] # Distorted 2D pixel points found in the actual camera images

# Grab all the .jpg images we saved using our get_images.py script
images = glob.glob('calibration_images/*.jpg')

print("Finding chessboard corners in images...")

for fname in images:
    # Load the image and make it grayscale (makes it easier to find black/white corners)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Let OpenCV search for the chessboard pattern in this specific image
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    # If it successfully found the whole pattern
    if ret == True:
        # Save the perfect 3D coordinates and the actual 2D pixels it found
        objpoints.append(objp)
        imgpoints.append(corners)
        print(f"Found corners in {fname}")
    else:
        print(f"Could not find corners in {fname}")

print("Calculating camera distortion...")

# Run the heavy math comparing the perfect 3D grid to the warped 2D camera images
# This figures out exactly how the curved lens bends straight lines
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save these exact math numbers into a file so our detect_aruco.py script can load them
# 'mtx' is the camera matrix (focal length and center), 'dist' fixes the fisheye warp
np.savez("calibration_data.npz", camera_matrix=mtx, dist_coeffs=dist)

print("Success! Saved calibration_data.npz")

"""
File: detect_aruco.py
Author: Isha Singh
Date: October 2026
Description: 
This script is the main computer vision program for Demo 1. It runs headlessly 
on a Raspberry Pi and uses the Pi camera to search for a 5x5cm ArUco marker 
(6x6 dictionary, ID 0). It loads calibration data to remove lens distortion, 
calculates the exact angle of the marker relative to the center of the camera 
using trigonometry, and outputs the result to a 16x2 LCD screen. Moving the 
marker to the left of the camera reads as a positive angle, and to the right 
reads as a negative angle.
"""

import cv2
import numpy as np

# ================= CONFIGURATION =================
# Set to False so the Pi can run headlessly on battery without a laptop plugged in
SHOW_WINDOW = False  
# Manually tweak this if the camera is slightly crooked (e.g., -1.2)
ANGLE_OFFSET = 0.0   
# Number of frames to average so the LCD numbers don't jump around too fast
N_AVG = 7            

# ================= CALIBRATION ===================
# Load the math from the flat checkerboard calibration 
calib = np.load("calibration_data.npz")
mtx = calib["camera_matrix"]
dist = calib["dist_coeffs"]

# Get the focal length (fx) and the camera's exact center pixel (cx)
fx = mtx[0, 0]
cx = mtx[0, 2]

# ================= ARUCO SETUP ===================
# Set to look for the 6x6 dictionary to match the rubric and the printed 5cm marker
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# ================= LCD SETUP =====================
try:
    import board
    import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
    
    # Initialize the 16x2 LCD screen
    lcd = character_lcd.Character_LCD_RGB_I2C(board.I2C(), 16, 2)
    lcd.color = [0, 100, 0] # Turn the backlight green
    lcd.clear()
    lcd.message = "Searching...\n"
except Exception as e:
    print("LCD Error:", e)
    lcd = None

def update_lcd(line1, line2=""):
    """Formats text to exactly 16 characters to overwrite old data cleanly without screen flickering."""
    if lcd:
        text = f"{line1[:16]:<16}\n{line2[:16]:<16}"
        lcd.cursor_position(0, 0)
        lcd.message = text
    print(f"{line1} | {line2}")

# ================= CAMERA SETUP ==================
# Open the camera and force the resolution to 640x480 to match the rubric
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

readings = []
misses = 0

while True:
    # Grab a live video frame from the camera
    ret, frame = cap.read()
    if not ret:
        continue

    # Convert the image to grayscale so the black and white marker is easier to find
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    # Check if we saw any markers, specifically looking for ID 0
    if ids is not None and 0 in ids:
        misses = 0 # Reset our miss counter
        
        # Find exactly where marker ID 0 is in the camera's lists
        idx = list(ids.flatten()).index(0)
        c = corners[idx][0]
        
        # Calculate the average X (u) and Y (v) pixel of the marker's corners
        u = np.mean(c[:, 0])
        v = np.mean(c[:, 1])

        # Remove the camera lens distortion from the marker's location to get precise math
        pt = cv2.undistortPoints(np.array([[[u, v]]]), mtx, dist, P=mtx)
        u_clean = pt[0, 0, 0]

        # Calculate the angle using a right triangle (arctangent)
        # Subtracting the marker location from the camera center makes the left side positive
        raw_angle = np.degrees(np.arctan((cx - u_clean) / fx))
        readings.append(raw_angle)

        # Once we collect enough frames (7), calculate the average angle
        if len(readings) >= N_AVG:
            angle = float(np.median(readings)) + ANGLE_OFFSET
            readings.clear()
            # Print the final calculated angle on the LCD
            update_lcd("Marker detected", f"{angle:+.2f} deg")
    else:
        # If we don't see the marker, count it as a miss
        readings.clear()
        misses += 1
        
        # Wait for 15 missed frames in a row before updating the LCD to prevent text flickering
        if misses > 15:
            update_lcd("Searching...", "No marker")

    # Only show the video window if we are testing with a laptop plugged in (SHOW_WINDOW = True)
    if SHOW_WINDOW:
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Clean up the camera and clear the LCD when the script ends
cap.release()
if SHOW_WINDOW:
    cv2.destroyAllWindows()
if lcd:
    lcd.color = [0, 0, 0]
    lcd.clear()

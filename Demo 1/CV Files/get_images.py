"""
File: get_images.py
Author: Isha Singh
Date: October 2026
Description: 
This script captures pictures using the Raspberry Pi camera to be used for 
camera calibration. It displays a live video feed and saves an image into a 
folder every time the 's' key is pressed.
"""

import cv2
import os

# Create a folder to save our calibration pictures if it doesn't exist yet
if not os.path.exists('calibration_images'):
    os.makedirs('calibration_images')

# Open the Raspberry Pi camera
cap = cv2.VideoCapture(0)

# Force the resolution to 640x480 to perfectly match the demo rubric
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

count = 0
print("Press 's' to save a picture. Press 'q' to quit.")

while True:
    # Grab a live video frame from the camera
    ret, frame = cap.read()
    if not ret:
        continue

    # Show the live video on the screen so we can see the chessboard
    cv2.imshow('Camera', frame)
    
    # Wait for a keyboard button press
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('s'):
        # If 's' is pressed, save the current frame as a picture in our folder
        img_name = f"calibration_images/calib_{count}.jpg"
        cv2.imwrite(img_name, frame)
        print(f"Saved {img_name}")
        count += 1
    elif key == ord('q'):
        # If 'q' is pressed, exit the loop
        break

# Turn off the camera and close the popup window safely
cap.release()
cv2.destroyAllWindows()

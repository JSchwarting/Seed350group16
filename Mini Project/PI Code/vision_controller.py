# EENG 350: EEID Lab - Mini Project
# Computer Vision and System Integration
# Author: Isha Singh & Team
# Purpose of the code: 
# Hardware connection:


import cv2
import numpy as np
import threading
import queue
import smbus2
from time import sleep

# Hardware & I2C LCD Library Imports
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# --- Step 1: Set Camera Parameters ---
ARUCO_DICT = cv2.aruco.DICT_6X6_50  
TARGET_ID = 0
CAMERA_INDEX = 0              
FRAME_WIDTH, FRAME_HEIGHT = 640, 480 
DEADBAND_PX = 15              

# Quadrant lookup table mapping regions to left and right wheel goal bits
QUADRANT_BITS = {
    "NE": (0, 0),
    "NW": (0, 1),
    "SW": (1, 1),
    "SE": (1, 0),
}

# I2C Leader-Follower Configuration: Pi is leader, Arduino follower is at address 8
I2C_ADDRESS = 8
try:
    bus = smbus2.SMBus(1)
except FileNotFoundError:
    print("I2C bus not found. Check configuration.")

# --- Step 2: Build the ArUco Detector Function ---
def make_detector():
    """Initializes and returns the ArUco detector function using OpenCV parameters."""
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    p = cv2.aruco.DetectorParameters()
    det = cv2.aruco.ArucoDetector(d, p)
    
    def detect(gray):
        corners, ids, _ = det.detectMarkers(gray)
        return corners, ids
    return detect

detect_marker = make_detector()

# --- Threading Setup for LCD Display ---
q = queue.Queue()

def lcd_thread():
    """Background worker thread to update the I2C physical LCD without blocking the video loop."""
    cols, rows = 16, 2
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        lcd = character_lcd.Character_LCD_RGB_I2C(i2c, cols, rows, address=0x20)
        lcd.color = [100, 100, 100] 
        lcd.clear()
        lcd.message = "System Ready"
    except Exception as e:
        print(f"LCD Init Error: {e}")
        lcd = None

    while True:
        if not q.empty():
            goal_str = q.get()
            print(f"LCD Update: Goal Position: {goal_str}")
            if lcd is not None:
                lcd.clear()
                lcd.message = f"Goal Position:\n{goal_str}"
        sleep(0.1)

# Start the background daemon thread for screen updates
lcd_worker = threading.Thread(target=lcd_thread, daemon=True)
lcd_worker.start()

# --- Initialize Video Capture ---
cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

last_quadrant = None

print("Vision controller active. Press 'q' in camera window to exit.")

# --- Main Control Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    # Convert color frame to grayscale for faster processing 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids = detect_marker(gray)
    
    # --- Step 3: Find One Specific Marker ---
    target_found = False
    center_x, center_y = 0, 0
    
    if ids is not None:
        ids_flat = ids.flatten()
        if TARGET_ID in ids_flat:
            # Find the exact index where marker 0 is located
            idx = np.where(ids_flat == TARGET_ID)[0][0]
            marker_corners = corners[idx][0]
            
            # Calculate center point as the mathematical average of the four marker corners
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
            target_found = True
    # --- Step 4: Map Marker Position to a Quadrant ---
    if target_found:
        h, w, _ = frame.shape
        mid_x, mid_y = w // 2, h // 2
        
        # Apply deadband boundary check around center crosshairs
        if abs(center_x - mid_x) > DEADBAND_PX and abs(center_y - mid_y) > DEADBAND_PX:
            if center_y < mid_y: 
                quad_str = "NE" if center_x > mid_x else "NW"
            else:
                quad_str = "SE" if center_x > mid_x else "SW"
                
            # --- Step 5: Pack Quadrant Info into 1 Byte and Transmit on Change Only ---
            if quad_str != last_quadrant:
                last_quadrant = quad_str
                left_bit, right_bit = QUADRANT_BITS[quad_str]
                
                # Combine left and right wheel states into a single 2-bit value via bitwise left-shift
                goal_byte = (left_bit << 1) | right_bit
                
                # Transmit byte over I2C bus to Arduino follower
                try:
                    bus.write_byte(I2C_ADDRESS, goal_byte)
                except OSError:
                    pass
                
                # Format string message to push to the LCD display thread queue
                display_str = f"{left_bit} {right_bit}"
                q.put(display_str)
                
    # Display live color camera window
    cv2.imshow("Camera Feed", frame)
    
    # Exit loop gracefully when 'q' is pressed on keyboard
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
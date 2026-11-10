import cv2
import numpy as np
import smbus2
import struct
import time

# ==================== CONFIGURATION ====================
SHOW_WINDOW = True      # IMPORTANT: Change to False before putting on the robot to
ANGLE_OFFSET = 0.0      # Adjust if camera is slightly off-center
N_AVG = 7               # Averages 7 frames for stability
RED_RATIO = 3.0         # red flag if red pixels > RED_RATIO x green pixels (camera biases green)
MARKER_SIZE_FT = 2.0 / 12.0  # 2-inch marker
ARDUINO_I2C_ADDR = 0x08 # Arduino's address

# ==================== CALIBRATION ====================
calib = np.load("calibration_data.npz")
mtx = calib["camera_matrix"]
dist = calib["dist_coeffs"]
fx = mtx[0, 0]
cx = mtx[0, 2]

# ==================== COMMUNICATION SETUP ====================
try:
    bus = smbus2.SMBus(1)
    print("I2C Bus connected to Arduino.")
except Exception as e:
    print("I2C Arduino Error (expected if not plugged in):", e)
    bus = None

# ==================== ARUCO SETUP ====================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# ==================== LCD SETUP ====================
try:
    import board
    import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
    lcd = character_lcd.Character_LCD_RGB_I2C(board.I2C(), 16, 2)
    lcd.color = [0, 100, 0]
    lcd.clear()
    lcd.message = "Searching...\n"
except Exception as e:
    print("LCD Error:", e)
    lcd = None

def update_lcd(line1, line2=""):
    """Formats text to exactly 16 characters to overwrite old data."""
    if lcd:
        text = f"{line1[:16]:<16}\n{line2[:16]:<16}"
        lcd.cursor_position(0, 0)
        lcd.message = text

# ==================== CAMERA SETUP ====================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

angle_readings = []
dist_readings = []
misses = 0

# FIXED: Moved outside the loop so they don't reset to zero every frame
distance_ft = 0.0
angle_deg = 0.0

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    flags = 0  # Flags should still reset every frame

    if ids is not None and 0 in ids:
        misses = 0
        flags |= 1  # Beacon Detected (bit 0)

        idx = list(ids.flatten()).index(0)
        c = corners[idx][0]
        u = np.mean(c[:, 0])
        v = np.mean(c[:, 1])

        # Remove lens distortion
        pt = cv2.undistortPoints(np.array([[[u, v]]]), mtx, dist, P=mtx)
        u_clean = pt[0, 0, 0]

        # Calculate angle
        raw_angle = np.degrees(np.arctan((cx - u_clean) / fx))
        angle_readings.append(raw_angle)

        # Calculate distance
        pixel_width = np.linalg.norm(c[0] - c[1])
        raw_dist = (MARKER_SIZE_FT * fx) / pixel_width
        dist_readings.append(raw_dist)

    # Arrow color: only look near the marker (box scales with distance)
        w = pixel_width
        x0 = int(max(u - 3*w, 0)); x1 = int(min(u + 3*w, frame.shape[1]))
        y0 = int(max(v - 1.5*w, 0)); y1 = int(min(v + 1.5*w, frame.shape[0]))
        roi = frame[y0:y1, x0:x1].copy()
        cv2.fillPoly(roi, [np.int32(c - [x0, y0])], (0, 0, 0))   # blank the marker
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, np.array([40, 70, 50]), np.array([90, 255, 255]))
        # Red range widened (saturation 80+, brightness 50+): this camera washes red out
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 80, 50]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([160, 80, 50]), np.array([180, 255, 255])))
        green_pixels = cv2.countNonZero(mask_green)
        red_pixels = cv2.countNonZero(mask_red)
        need = 0.3 * w * w          # green arrow must cover ~30% of a marker's area
        red_need = 0.1 * w * w      # red only needs ~10%: the camera under-reads red
        print(f"green {green_pixels}  red {red_pixels}  need {need:.0f}  red_need {red_need:.0f}")
        # The camera is biased toward green: there is almost always a little green
        # in view (25-50 px). So check RED FIRST and only ask that red clearly beats
        # green (RED_RATIO times as many pixels), not that green is zero.
        if red_pixels > red_need and red_pixels > RED_RATIO * green_pixels:
            flags |= 4              # Right (red)
        elif green_pixels > need and green_pixels > 2 * red_pixels:
            flags |= 2              # Left (green)
        if len(angle_readings) >= N_AVG:
            angle_deg = float(np.median(angle_readings)) + ANGLE_OFFSET
            distance_ft = float(np.median(dist_readings))
            angle_readings.clear()
            dist_readings.clear()

            update_lcd(f"Dist: {distance_ft:.2f} ft", f"Ang: {angle_deg:+.2f} deg")

    else:
        angle_readings.clear()
        dist_readings.clear()
        misses += 1
        if misses > 15:
            update_lcd("Searching...", "No marker")

    # Terminal output for desk testing
    print(f"Flag: {flags} | Dist: {distance_ft:.2f} ft | Ang: {angle_deg:.2f} deg")

    # Send to Arduino
    if bus:
        try:
            packed_data = struct.pack('<Bff', flags, distance_ft, angle_deg)
            bus.write_i2c_block_data(ARDUINO_I2C_ADDR, 0, list(packed_data))
        except Exception:
            pass

    if SHOW_WINDOW:
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
if SHOW_WINDOW:
    cv2.destroyAllWindows()
if lcd:
    lcd.color = [0, 0, 0]
    lcd.clear()

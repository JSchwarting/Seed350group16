Group: Team 16
Purpose: This folder houses all code related to Demo 2 for SEED Lab
File for Controls subsystem grading is Demo2_Optimized_Rolling.ino within the Final Arduino Code Folder.
File for CV subsystem grading is detect_aruco.py within the Final Raspberry Pi Code Folder.

Advanced File Descriptions:
    Final Arduino Code Folder:
        Demo2_Optimized_Rolling.ino: This file is our finalized code. It uses the same controllers as Demo 1 (Inner loop uses a P controller to control the velocity, while the outer loop uses a PI controller to control the position of the robot). It adds a state machine that rotates until the Pi sees the marker, turns to face it, drives toward it, reads the arrow color while slowing down (Trial 2), stops, and then turns 90 degrees in the direction of the arrow.

    Outdated Arduino Folder: Files in this folder are previous iterations of our final code. This includes our first Demo 2 state machine, a version that stopped at 3 ft to read the arrow, speed-tuned versions, and a version with an alignment fix that we did not end up using.

    Starting Arduino Code Folder:
        Demo2_Starting_Code.ino: Our final Demo 1 code with the goal angle and distance taken from the Pi instead of being hard coded, plus the basic I2C setup.

        Demo2_I2C_standalone_Code.ino: Standalone I2C code used to test receiving the Pi's data on the Arduino.

    Final Raspberry Pi Code Folder:
        detect_aruco.py: The main script that runs on the Pi; it finds the ArUco marker, measures its distance and angle (median of 7 frames), checks for a red (right) or green (left) arrow next to it, shows the status on the LCD, and sends the info to the Arduino over I2C.

    LCD Issues Folder:
        README.md: Explains why the LCD stopped working when the Arduino was on the same I2C wires, how to check the wiring, and how to run the code on the Pi.

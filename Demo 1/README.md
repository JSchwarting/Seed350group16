Group: Team 16
Purpose: This repository houses all code related to Demo 1 for SEED Lab
File for Controls subsytem grading is Demo1TurnDrive_UPDATED_PI.ino within the Arduino Code Folder.
Files for CV subsystem grading are all in the CV Files Folder.

Advanced File Descriptions:
    Arduino Code:
        Demo1TurnDrive_UPDATE_PI.ino: This file is our finalized code. It contains code to control the motors that was used in the Mini Project (Encoder ISR, voltage saturation, etc.). It also includes the Simulink Model for our controllers (Inner loop uses a P controller to control the velocity, while the outer loop uses a PI controller to control the position of the robot).

        Demo1OutdatedCode Folder: Files in this code are previous iterations of our final code. This includes edited Mini-Project code from early in the project, to separate files from calibration testing with only slightly different values.

        CV Files Folder:
            calibrate.py: Analyzes those checkerboard pictures to calculate the camera's lens distortion and saves the math (the .npzfile). (Mostly for calibration)

            detect_aruco.py: The main script that runs on the robot; it loads the saved math, finds the ArUco marker, and calculates its exact angle in real-time using simple geometry.

            get_images.py: Snaps pictures of the flat checkerboard pattern using the camera. (Mostly for calibration)

        Simulation Files Folder:
            steering_inner.slxc:

            steering_turn.slxc:

            SteeringFull.slx:

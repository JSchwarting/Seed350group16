Group: Team 16
Purpose: This repository houses all code related to the Mini Project for SEED Lab
Top Level File for grading is Mini_Project.ino within the Arduino Code Folder. 
Organization:
    Arduino Code: Houses all .ino code that go onto the Arduino
    MatLab Code: Houses any MatLab code and Simulink models
    Python Code: Houses all Python scripts that go onto the Raspberry Pi


Advanced File Descriptions: 
    Arduino Code: 
        Mini Project button Test: This file is out test file that simulates the camera output using buttons instead. This was for our Localization 
        and controls group to test our code before the camera portion was up and running. 

        Mini Project: This is our final code. The only difference from button test is we get the information from the camera and not the button. 

        Position Control Draft Copy: Reference File for testing using a step function similar to that of our Velocity Controller. This is FrankenCode
        pieced together from the velocity control step, new values, and motor position control code. 

        Working Position Control: More reference code similar to that of above. While not integral to the project shows iteration. 

        Pi to Arduino Test: Tests the implementation of data transfer from the pi to the Arduino. 

    PiCode:
        Vision Controller: This file uses pixel logic to determine what quadrant the object is relative to the cameras vision, and outputs that information
        as a 2 bit vector, 

    MatCode:
        Position Model: Our Simulink model that was used to simulate the Integration Position Control values. This uses much of our velocity Control as a subsystem
        and then integrates that to get our output position. 
        
        MiniProjectAttempt: This file is a matlab file that proccesses the output of our simulation to plot our important factors, especially voltage and position. 
        
        

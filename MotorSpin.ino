/*
  Danek Colson
  SEED Lab - Group 16
  Spin Motors
*/

// Motor control pins. Configured for direction, speed
int Motor1[2] = {7, 9};
int Motor2[2] = {8, 10};
// Controls whether the motors are on or off
int MotorEnable = 4;

// Pins used for our Motor Encoders
int M1EncA = 2;
int M1EncB = 5;
int M2EncA = 3;
int M2EncB = 6;

// Tracking the position of the motors
int M1Pos = 0;
int M2Pos = 0;
// Tracking whenever the motors have changed position, not currently used in this code.
bool M1move = false;
bool M2move = false;

// How long we should wait between each print statement
unsigned long desired_Ts = 100;
// Tracking the time we started the simulation, and how long each loop is
unsigned long last_time, start_time;
float current_time;

// Tracking the change in the wheel's position in radians
float left_rad_old, left_rad_new;
float right_rad_old, right_rad_new;

// Tracking the change in x, y, and phi in order to track the distance in we have traveled
float x_new, y_new, x_old, y_old;
float phi_new, phi_old;
// The distance between the wheels in cm
float wheel_dist = 34.7;
// The readius of our wheelis in cm
float wheel_radius = 15.4;
// Tracking the change in the distance the left and right wheel have moved each loop
float del_dl, del_dr;

void setup() {
  // Set up our motor to be outputs (only necessary for commented out code that controls actually moving the motors), not necessary if only looking at the motor encoders
  pinMode(MotorEnable, OUTPUT);
  pinMode(Motor1[0], OUTPUT);
  pinMode(Motor1[1], OUTPUT);
  pinMode(Motor2[0], OUTPUT);
  pinMode(Motor2[1], OUTPUT);
  
  // Set up the Motor encoder pins as inputs
  pinMode(M1EncA, INPUT);
  pinMode(M1EncB, INPUT);
  pinMode(M2EncA, INPUT);
  pinMode(M2EncB, INPUT);
  // Have the A pins for each motor be attached to our ISR, triggering every time the A pin changes
  attachInterrupt(digitalPinToInterrupt(M1EncA), M1EncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2EncA), M2EncISR, CHANGE);

  // Set up the Serial Monitor. Note: use PuTTy if copying the data into MatLab, in order to do so, must close the Serial Monitor in Arduino
  Serial.begin(9600);
  // Get our start time
  last_time = millis();
  start_time = last_time;

  // Print a blank line so it is easy to see where our data starts on each iteration
  Serial.println();

  // Set up our variables so they will work for our loops
  x_old = 0;
  y_old = 0;
  phi_old = 0;
  left_rad_old = 0;
  right_rad_old = 0;
}

/*
  Note:
    M1 -> HIGH spins away from arduino, LOW spins towards arduino
    M2 -> HIGH spins towards arduino, LOW spins away from arduino
*/

void loop() {
  //  Calculate the position, in radians, for the wheels based on the encoder position
  right_rad_new = 2*PI*(float)M1Pos/3200;
  left_rad_new = 2*PI*(float)M2Pos/3200;

  // calculate the change in our wheel position by comparing the previous position with the current position
  del_dl = (left_rad_new - left_rad_old)*wheel_radius;
  del_dr = (right_rad_new - right_rad_old)*wheel_radius;

  // Calculate our x, y, and phi positions based on the change in our wheel positions
  x_new = x_old + cos(phi_old)*(del_dl + del_dr)/(float)2;
  y_new = y_old + sin(phi_old)*(del_dl + del_dr)/(float)2;
  phi_new = phi_old + ((float)1/wheel_dist)*(del_dl - del_dr);

/*
  // Code that controls the motor to spin the wheels, unused for this code since the wheels are spun by hand
  digitalWrite(MotorEnable, HIGH);
  analogWrite(Motor1[1], 56);
  digitalWrite(Motor1[0], HIGH);
  analogWrite(Motor2[1], 56);
  digitalWrite(Motor2[0], LOW);
*/

  // Get the current time for our loop by subtracting the time from the last loop from the start time
  current_time = (float)(last_time-start_time)/1000;

  // Prints the data in the format: current_time x_pos y_pos phi_pos;
  // Formatted this way so a direct copy and paste into MatLab works without needing to change any formatting
  Serial.println(String(current_time) + " " + String(x_new) + " " + String(y_new) + " " + String(phi_new) + ";");

  // Waits desired_Ts milliseconds each loop so each calculation and print statement only occurs every desired_Ts ms
  while (millis()<last_time + desired_Ts) {
  //wait until desired time passes to go top of the loop
  }
  // Update our time tracking variable
  last_time = millis();

  // Update all our current variables to be old, so that next loop we can compare them against that loops variables
  right_rad_old = right_rad_new;
  left_rad_old = left_rad_new;
  x_old = x_new;
  y_old = y_new;
  phi_old = phi_new;

/*
  // Old code that would only print statements if it detected any movement
  if (M1move || M2move) {
    Serial.println("M1 Pos: " + String(M1Pos) + " M2 Pos: " + String(M2Pos));
    if (M1move) {
      M1move = false;
    } else if (M2move) {
      M2move = false;
    }
  }
*/
}

// Note: All motor position changes are changes by 2 since we are only tracking changes in A which will change every 2nd position change of the motor

// ISR for Motor 1, triggers anytime A changes
void M1EncISR() {
  // M1move = true;
  // Checks if A and B are equal when A has changed, if so the position increased 
  if (digitalRead(M1EncA) == digitalRead(M1EncB)) {
    M1Pos += 2;
  // Checks if A and B are unequal when A has changed, if so the position decreased 
  } else if (digitalRead(M1EncA) != digitalRead(M1EncB)) {
    M1Pos -=2;
  }
}

// ISR for Motor 2, triggers anytime A changes. 
// Note: The checks are flipped from Motor1 since Motor2 is facing the opposite direction and we want to make positive position one direction the robot moves, not how the individual wheel moves
void M2EncISR() {
  // M2move = true;
  // Checks if A and B are equal when A has changed, if so the position is decreased
  if (digitalRead(M2EncA) == digitalRead(M2EncB)) {
    M2Pos -= 2;
  // Checks if A and B are unequal when A has changed, if so the position is increased
  } else if (digitalRead(M2EncA) != digitalRead(M2EncB)) {
    M2Pos +=2;
  }
}

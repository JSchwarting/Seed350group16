/*
  SEED Lab - Group 16
  Mini Project
    Image will be shown to a camera. Camera returns what quadrant the image is in.
    Then the Arduino will spin it's motors accordingly to display either the "top" or "bottom" of the wheel.
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

// Tracking the position of the motors in encoder counts
int M1Pos = 0;
int M2Pos = 0;

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
}

void loop() {
  // Do some data processing to resolve what two numbers the wheels need to display
  // Then move the motors to that position
    // if -> left motor = 0, move left motor until left motor position = 0
    // if -> left motor = 1, move left motor until left motor position = 1600 (180 degrees)
    // if -> right motor = 0, move right motor until right motor position = 0
    // if -> right motor = 0, move right motor until right motor position = 1600 (180 degrees)
  // Implement PI control so that it stays at the desired position
}

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

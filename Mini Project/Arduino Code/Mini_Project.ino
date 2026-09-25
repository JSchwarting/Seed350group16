/*
  SEED Lab - Group 16
  Mini Project
    Image will be shown to a camera. Camera returns what quadrant the image is in.
    Then the Arduino will spin it's motors accordingly to display either the "top" or "bottom" of the wheel.
*/

// Motor control pins. Configured for direction, speed
// Motor 1 corresponds to the right (passenger side) wheel
// Motor 2 corresponds to the left (driver side) wheel
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

// What the position should be
int M1DesiredPos = 0;
int M2DesiredPos = 0;

// PI controller variables
float M1DesiredRad = 0;
float M2DesiredRad = 0;
float M1Rad = 0;
float M2Rad = 0;

float M1Error = 0;
float M2Error = 0;

float M1IntegralError = 0;
float M2IntegralError = 0;

// Pins that talk to the Pi to track what quadrant the image is in
int NSPin = 1;
int EWPin = 2;

int NS, EW;

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

  // Pin set up to talk to the Raspberry Pi
  pinMode(NSPin, INPUT);
  pinMode(EWPin, INPUT);
}

void loop() {
  // Implement PI control so that it stays at the desired position

  // Getting Data from Raspberry Pi
  NS = digitalRead(NSPin);
  EW = digitalRead(EWPin);

  // In north half if NS = 0, In south half if NS = 1
  if (NS = 0) {
    // move left motor to pos 0 (0 degrees = 0 encoder counts)
    M2DesiredPos = 0;
  } else if (NS = 1) {
    // move left motor to pos 1 (180 degress = 1600 encoder counts)
    M2DesiredPos = 1600;
  }

  // IN east half if EW = 0, In west half if EW = 1
  if (EW = 0) {
    // move right motor to pos 0 (0 degrees = 0 encoder counts)
    M1DesiredPos = 0;
  } else if (EW = 1) {
    // move right motor to pos 1 (180 degrees = 1600 encoder counts)
    M1DesiredPos = 1600:
  }

  M1Rad = 2*PI*(float)M1Pos/3200;
  M2Rad = 2*PI*(float)M2Pos/3200;
  M1DesiredRad = 2*PI*(float)M1DesiredPos/3200;
  M2DesiredPos = 2*PI*(float)M2DesiredPos/3200;

  M1Error = M1DesiredRad - M1Rad;
  M2Error = M2DesiredRad - M2Rad;

  /*
  if ((M1Pos != M1DesiredPos) || (M2Pos != M2DesiredPos)) {
    if (M1Pos < M1DesiredPos) {
      digitalWrite(MotorEnable, HIGH);
      analogWrite(Motor1[1], 56);
      digitalWrite(Motor1[0], HIGH);
    } else if(M1Pos > M1DesiredPos) {
      digitalWrite(MotorEnable, HIGH);
      analogWrite(Motor1[1], 56);
      digitalWrite(Motor1[0], LOW);
    }

    if (M2Pos < M2DesiredPos) {
      digitalWrite(MotorEnable, HIGH);
      analogWrite(Motor2[1], 56);
      digitalWrite(Motor2[0], LOW);
    } else if (M2Pos > M2DesiredPos) {
      digitalWrite(MotorEnable, HIGH);
      analogWrite(Motor1[1], 56);
      digitalWrite(Motor1[0], HIGH);
    }
  } else {
    digitalWrite(MotorEnable, LOW);
  }
    */
}

// ISR for Motor 1, triggers anytime A changes
void M1EncISR() {
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
  // Checks if A and B are equal when A has changed, if so the position is decreased
  if (digitalRead(M2EncA) == digitalRead(M2EncB)) {
    M2Pos -= 2;
  // Checks if A and B are unequal when A has changed, if so the position is increased
  } else if (digitalRead(M2EncA) != digitalRead(M2EncB)) {
    M2Pos +=2;
  }
}

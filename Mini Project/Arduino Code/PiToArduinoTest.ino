/*
  SEED Lab - Group 16
  Mini Project
    Image will be shown to a camera. Camera returns what quadrant the image is in.
    Then the Arduino will spin its motors accordingly to display either the "top" or "bottom" of the wheel.

    Pi -> Arduino link is I2C (Pi = leader, Arduino = follower, address 8):
      Pi SDA (GPIO2) -> Arduino A4 (SDA)
      Pi SCL (GPIO3) -> Arduino A5 (SCL)
      Pi GND         -> Arduino GND
    Use a bidirectional level shifter between the Pi (3.3 V) and Arduino (5 V).

    Pi sends one byte:  bit1 = left bit  = NS (0 = north, 1 = south)  -> Motor 2 (left wheel)
                        bit0 = right bit = EW (0 = east,  1 = west)   -> Motor 1 (right wheel)
*/

#include <Wire.h>

#define I2C_ADDRESS 8

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

// Tracking the position of the motors in encoder counts (changed in ISRs)
volatile int M1Pos = 0;
volatile int M2Pos = 0;

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
float M1Voltage = 0;
float M2Voltage = 0;

// --- closed-loop control gains & limits ---
float BatteryVoltage = 7.8;   // measured/expected battery voltage
float M1Kp = 3.5;             // Proportional gain from Simulink
float M1Ki = 0.5;             // Integral gain from Simulink
float M2Kp = 3.5;
float M2Ki = 0.5;
float SatLimit = 6.0;         // voltage saturation limit, +/- volts
float dt = 0.1;               // control period in seconds

unsigned long lastControlTime = 0;

// Goal byte received from the Pi over I2C (updated in receiveEvent)
volatile byte goalByte = 0;

int NS = 0;
int EW = 0;

void setup() {
  // Set up our motor to be outputs
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

  digitalWrite(MotorEnable, HIGH);

  // Have the A pins for each motor be attached to our ISR, triggering every time the A pin changes
  attachInterrupt(digitalPinToInterrupt(M1EncA), M1EncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2EncA), M2EncISR, CHANGE);

  // I2C follower setup (SDA = A4, SCL = A5 on Uno)
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
}

void loop() {
  // Run the controller at a fixed period of dt seconds
  unsigned long now = millis();
  if (now - lastControlTime < (unsigned long)(dt * 1000)) {
    return;
  }
  lastControlTime = now;

  // Decode the goal byte from the Pi (copy atomically)
  noInterrupts();
  byte goal = goalByte;
  int m1Pos = M1Pos;
  int m2Pos = M2Pos;
  interrupts();

  NS = (goal >> 1) & 1;   // left bit
  EW = goal & 1;          // right bit

  // In north half if NS = 0, In south half if NS = 1
  if (NS == 0) {
    // move left motor to pos 0 (0 degrees = 0 encoder counts)
    M2DesiredPos = 0;
  } else {
    // move left motor to pos 1 (180 degrees = 1600 encoder counts)
    M2DesiredPos = 1600;
  }

  // In east half if EW = 0, In west half if EW = 1
  if (EW == 0) {
    // move right motor to pos 0 (0 degrees = 0 encoder counts)
    M1DesiredPos = 0;
  } else {
    // move right motor to pos 1 (180 degrees = 1600 encoder counts)
    M1DesiredPos = 1600;
  }

  M1Rad = 2 * PI * (float)m1Pos / 3200;
  M2Rad = 2 * PI * (float)m2Pos / 3200;
  M1DesiredRad = 2 * PI * (float)M1DesiredPos / 3200;
  M2DesiredRad = 2 * PI * (float)M2DesiredPos / 3200;

  M1Error = M1DesiredRad - M1Rad;
  M2Error = M2DesiredRad - M2Rad;

  M1IntegralError += M1Error * dt;
  M2IntegralError += M2Error * dt;

  M1Voltage = (M1Kp * M1Error) + (M1Ki * M1IntegralError);
  M2Voltage = (M2Kp * M2Error) + (M2Ki * M2IntegralError);

  // --- Voltage Saturation and Anti-Windup (Motor 1) ---
  if (M1Voltage > SatLimit) {
    M1Voltage = SatLimit;
    M1IntegralError -= M1Error * dt;   // undo this step's integration to prevent windup
  } else if (M1Voltage < -SatLimit) {
    M1Voltage = -SatLimit;
    M1IntegralError -= M1Error * dt;
  }

  // --- Voltage Saturation and Anti-Windup (Motor 2) ---
  if (M2Voltage > SatLimit) {
    M2Voltage = SatLimit;
    M2IntegralError -= M2Error * dt;
  } else if (M2Voltage < -SatLimit) {
    M2Voltage = -SatLimit;
    M2IntegralError -= M2Error * dt;
  }

  // --- Drive Motor 1 ---
  if (M1Voltage > 0) {
    digitalWrite(Motor1[0], HIGH);
  } else {
    digitalWrite(Motor1[0], LOW);
  }
  int M1PWM = (int)(255.0 * abs(M1Voltage) / BatteryVoltage);
  analogWrite(Motor1[1], min(M1PWM, 255));

  // --- Drive Motor 2 ---
  if (M2Voltage > 0) {
    digitalWrite(Motor2[0], LOW);
  } else {
    digitalWrite(Motor2[0], HIGH);
  }
  int M2PWM = (int)(255.0 * abs(M2Voltage) / BatteryVoltage);
  analogWrite(Motor2[1], min(M2PWM, 255));
}

// I2C receive handler: Pi writes a single byte. Keep this short.
void receiveEvent(int numBytes) {
  while (Wire.available()) {
    goalByte = Wire.read();   // last byte received wins
  }
}

// ISR for Motor 1, triggers anytime A changes
void M1EncISR() {
  // Checks if A and B are equal when A has changed, if so the position increased
  if (digitalRead(M1EncA) == digitalRead(M1EncB)) {
    M1Pos += 2;
  // Checks if A and B are unequal when A has changed, if so the position decreased
  } else {
    M1Pos -= 2;
  }
}

// ISR for Motor 2, triggers anytime A changes.
// Note: The checks are flipped from Motor1 since Motor2 is facing the opposite direction and we want
// positive position to be one direction the robot moves, not how the individual wheel moves
void M2EncISR() {
  // Checks if A and B are equal when A has changed, if so the position is decreased
  if (digitalRead(M2EncA) == digitalRead(M2EncB)) {
    M2Pos -= 2;
  // Checks if A and B are unequal when A has changed, if so the position is increased
  } else {
    M2Pos += 2;
  }
}

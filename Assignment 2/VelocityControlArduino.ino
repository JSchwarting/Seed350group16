/*
  Danek Colson / Jake Schwarting
  SEED Lab - Group 16
  Spin Motors and Velocity Control

  Modified for closed-loop P-control step response test on Motor 1.
  Desired velocity is held at 0 rad/s until t=2s, then steps to 2 rad/s.
  Logs time / desired velocity / voltage / velocity so the response can
  be compared against the closed-loop Simulink model.
*/

//Configured for direction, speed
int Motor1[2] = {7, 9};
int Motor2[2] = {8, 10};
int MotorEnable = 4;

int M1EncA = 2;
int M1EncB = 5;
int M2EncA = 3;
int M2EncB = 6;

int M1Pos = 0;
int M2Pos = 0;
bool M1move = false;
bool M2move = false;

unsigned long desired_Ts = 10;
unsigned long last_time, start_time;
float current_time;

// --- closed-loop control ---
long M1PrevPos = 0;          // position at the previous sample, for velocity calc
float BatteryVoltage = 7.8;  // measured/expected battery voltage
float Kp = 3.0;              // proportional gain (matches MATLAB/Simulink model)
float SatLimit = 6.0;        // voltage saturation limit, +/- volts

float DesiredVelocity;       // rad/s, stepped at t=2s
float M1Voltage;             // commanded voltage this sample
float M1Velocity;            // computed velocity this sample
float error;                 // desired - measured

void setup() {
  pinMode(MotorEnable, OUTPUT);
  pinMode(Motor1[0], OUTPUT);
  pinMode(Motor1[1], OUTPUT);
  pinMode(Motor2[0], OUTPUT);
  pinMode(Motor2[1], OUTPUT);
  
  pinMode(M1EncA, INPUT);
  pinMode(M1EncB, INPUT);
  pinMode(M2EncA, INPUT);
  pinMode(M2EncB, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1EncA), M1EncISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2EncA), M2EncISR, CHANGE);

  Serial.begin(115200);  // streaming at 10ms
  last_time = millis();
  start_time = last_time;
  M1PrevPos = M1Pos;
  digitalWrite(MotorEnable, HIGH);
}

/*
  Note:
    M1 -> HIGH spins away from arduino, LOW spins towards arduino
    M2 -> HIGH spins towards arduino, LOW spins away from arduino
*/
void loop() {
  current_time = (float)(last_time - start_time) / 1000;

  if (current_time <= 10.0) {
    // --- step the desired velocity at t = 2s ---
    if (current_time < 2.0) {
      DesiredVelocity = 0.0;
    } else {
      DesiredVelocity = 2.0;
    }

    // --- velocity from consecutive position samples ---
    float dt = (float)desired_Ts / 1000.0;
    M1Velocity = 2 * PI * (float)(M1Pos - M1PrevPos) / 3200.0 / dt;
    M1PrevPos = M1Pos;

    // --- proportional control law ---
    error = DesiredVelocity - M1Velocity;
    M1Voltage = Kp * error;

    // --- saturate command voltage ---
    if (M1Voltage > SatLimit)  M1Voltage = SatLimit;
    if (M1Voltage < -SatLimit) M1Voltage = -SatLimit;

    // --- apply to motor ---
    digitalWrite(Motor1[0], M1Voltage > 0 ? HIGH : LOW);
    int M1PWM = (int)(255.0 * abs(M1Voltage) / BatteryVoltage);
    analogWrite(Motor1[1], min(M1PWM, 255));

    digitalWrite(Motor2[0], LOW);
    analogWrite(Motor2[1], 0);

    Serial.println(String(current_time) + "\t" + String(DesiredVelocity) + "\t" + String(M1Voltage) + "\t" + String(M1Velocity));
  } else {
    // --- test finished: shut motor off and idle ---
    analogWrite(Motor1[1], 0);
    analogWrite(Motor2[1], 0);
  }

  while (millis() < last_time + desired_Ts) {
    //wait until desired time passes to go top of the loop
  }
  last_time = millis();
}

void M1EncISR() {
  M1move = true;
  if (digitalRead(M1EncA) == digitalRead(M1EncB)) {
    M1Pos += 2;
  } else if (digitalRead(M1EncA) != digitalRead(M1EncB)) {
    M1Pos -=2;
  }
}

void M2EncISR() {
  M2move = true;
  if (digitalRead(M2EncA) == digitalRead(M2EncB)) {
    M2Pos -= 2;
  } else if (digitalRead(M2EncA) != digitalRead(M2EncB)) {
    M2Pos +=2;
  }
}
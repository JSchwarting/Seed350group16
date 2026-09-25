/* / Jake Schwarting
  SEED Lab - Group 16
  Closed-loop PI Position Controller test on Motor 1.
  Desired position steps to PI (3.14159 rad) at t = 2s.
*/

// Configured for direction, speed
int Motor1[2] = {7, 9};
int Motor2[2] = {8, 10};
int MotorEnable = 4;

int M1EncA = 2;
int M1EncB = 5;
int M2EncA = 3;
int M2EncB = 6;

// Position counters updated by ISRs (in counts)
volatile long M1Pos = 0;
volatile long M2Pos = 0;
bool M1move = false;
bool M2move = false;

unsigned long desired_Ts = 10; // 10ms loop time
unsigned long last_time, start_time;
float current_time;

// --- closed-loop control gains & limits ---
float BatteryVoltage = 7.8;   // measured/expected battery voltage
float Kp = 3.5;               // Proportional gain from Simulink
float Ki = 0.5;               // Integral gain from Simulink
float SatLimit = 6.0;         // voltage saturation limit, +/- volts

// --- controller state variables ---
float DesiredPosition = 0.0;  // Target angle in radians
float M1Position = 0.0;       // Measured angle in radians
float M1Voltage = 0.0;        // Output voltage command
float error = 0.0;            // Position error (rad)
float integralError = 0.0;    // Accumulated error over time (rad*s)

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

  Serial.begin(115200);
  last_time = millis();
  start_time = last_time;
  digitalWrite(MotorEnable, HIGH);
}

void loop() {
  current_time = (float)(last_time - start_time) / 1000.0;

  if (current_time <= 10.0) {
    // --- Step the desired position to PI (3.14159 rad) at t = 2s ---
    if (current_time < 2.0) {
      DesiredPosition = 0.0;
    } else {
      DesiredPosition = PI; // Step to pi radians (180 degrees)
    }

    float dt = (float)desired_Ts / 1000.0;

    // --- Convert encoder ticks to actual angular position (rad) ---
    // 3200 counts per revolution assumed from original code
    M1Position = 2.0 * PI * (float)M1Pos / 3200.0;

    // --- Calculate Position Error ---
    error = DesiredPosition - M1Position;

    // --- PI Control Calculation ---
    // Accumulate error scaled by sample time dt
    integralError += error * dt;

    // Unclamped control voltage equation
    M1Voltage = (Kp * error) + (Ki * integralError);

    // --- Voltage Saturation and Anti-Windup ---
    if (M1Voltage > SatLimit) {
      M1Voltage = SatLimit;
      // Clamp integral term to prevent windup when output saturates
      integralError -= error * dt; 
    } else if (M1Voltage < -SatLimit) {
      M1Voltage = -SatLimit;
      // Clamp integral term
      integralError -= error * dt; 
    }
   

    // --- Drive Motor 1 ---
    digitalWrite(Motor1[0], M1Voltage > 0 ? HIGH : LOW);
    int M1PWM = (int)(255.0 * abs(M1Voltage) / BatteryVoltage);
    analogWrite(Motor1[1], min(M1PWM, 255));
   
   int MP2PWM = M1PWM;
    // didn't do motor 2 yet that will be read in
   digitalWrite(Motor2[0], M1Voltage > 0 ? HIGH : LOW);
    analogWrite(Motor2[1], min(MP2PWM, 255));


   // just when we are done
    // Output: Time (s) | DesiredPos (rad) | Voltage (V) | ActualPos (rad)
    Serial.println(String(current_time) + "\t" + String(DesiredPosition) + "\t" + String(M1Voltage) + "\t" + String(M1Position));
  } else {
    // --- Test finished: shut motor off and idle ---
    analogWrite(Motor1[1], 0);
    analogWrite(Motor2[1], 0);
  }

  while (millis() < last_time + desired_Ts) {
    // Wait for sample period
  }
  last_time = millis();
}

void M1EncISR() {
  M1move = true;
  if (digitalRead(M1EncA) == digitalRead(M1EncB)) {
    M1Pos += 2;
  } else {
    M1Pos -= 2;
  }
}

void M2EncISR() {
  M2move = true;
  if (digitalRead(M2EncA) == digitalRead(M2EncB)) {
    M2Pos -= 2;
  } else {
    M2Pos += 2;
  }
}
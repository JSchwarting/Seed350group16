/*
  SEED Lab - Group 16
  Demo 1 - Tasks 2 and 3: turn to an angle, then drive a distance.

    Same control structure as the Simulink models (guides 1-3):
      Inner P loops:  Va_bar = Kp_rho_in * (rho_dot_d - rho_dot)
                      dVa    = Kp_phi_in * (phi_dot_d - phi_dot)
      Outer PD loops: phi_dot_d = Kp_phi_out*(phi_des - phi) - Kd_phi_out*phi_dot
                      rho_dot_d = Kp_rho_out*(rho_des - rho) - Kd_rho_out*rho_dot
      (D acts on the measured rate, so a setpoint jump gives no derivative kick)

    Task 2 (straight line): PhiGoalDeg = 0, DistGoalFt = 1..10
    Task 3 (turn then go):  PhiGoalDeg = angle (+ = left / CCW), DistGoalFt = distance

    States:
      WAIT   - stand still for StartDelay s (demo must start within 10 s)
      TURN   - angle loop drives phi to PhiGoalDeg, forward speed held at 0
      DRIVE  - distance loop drives rho to DistGoalFt; angle loop keeps holding heading
      DONE   - motors off
    TURN -> DRIVE once the robot has stopped within TurnTol for SettleTime
    (the Simulink models used a fixed t_drive instead). DRIVE -> DONE the same
    way with DistTol. If friction stalls it outside tolerance for StuckTime, or
    a phase runs past its timeout, it moves on anyway so it can never hang.

    Serial output (115200 baud), tab separated, one line per 10 ms:
      t | state | phi_des | phi | rho_des | rho | phi_dot_d | phi_dot | rho_dot_d | rho_dot | Va1 | Va2
    Angles in degrees and distances in feet, so it reads like the demo.
    The run ends with a line containing only END.
*/

// ---------------- Demo settings (hard-code before download) ----------------
float PhiGoalDeg = 0.0;      // deg, + = left (CCW). 0 = task 2 straight test
float DistGoalFt = 10.0;      // ft, 1 to 10
float StartDelay = 3.0;      // s standing still before moving
// ----------------------------------------------------------------------------


// Inner loop gains (V per m/s, V per rad/s). Same as SteeringInnerLoopTest.ino;
// replace with what design_steering_gains.m prints after system ID.
float Kp_rho_in = 37.8;
float Kp_phi_in = 6.55;

// Outer loop gains, same names as steering_params.m
float Kp_phi_out = 6.0;      // (rad/s) per rad
float Kd_phi_out = 0.0;      // add 0.05 to 0.2 if the turn overshoots
float Kp_rho_out = 4.0;      // (m/s) per m
float Kd_rho_out = 0.0;

// Setpoint limits, same as steering_params.m
float phidot_max = 1.5;      // rad/s
float rhodot_max = 0.5;      // m/s

// When each phase ends. Motor friction stops the robot a little short of the
// exact goal, so a phase ends once the robot has STOPPED within its tolerance.
// The heading loop keeps correcting during the drive.
float TurnTol = 3.0 * PI / 180.0;    // rad, start driving once stopped within this
float DistTol = 0.0127;              // m (0.5 in), finish once stopped within this
float StillRate = 0.01;              // m/s and rad/s counted as "stopped"
float SettleTime = 0.3;              // s it must stay stopped
float StuckTime = 1.0;               // s stopped outside tolerance before moving on anyway
float TurnTimeout = 6.0;             // s, start driving after this no matter what
float DriveTimeout = 40.0;           // s, stop after this no matter what (demo limit is 60 s)

// Optional friction kick (V per wheel) to close the last bit of error. 0 = off.
// Try 0.5 if the robot stops short; lower it if it buzzes at the end.
float FrictionVolts = 0.5;

// Robot geometry. MEASURE THESE (same as steering_params.m).
float r = 0.075;             // wheel radius (m), 15.4 cm wheel diameter (measured)
float d = 0.34;             // wheel separation (m), 34.7 cm (measured)

float BatteryVoltage = 7.8;
const float FtToM = 0.3048;

int Motor1[2] = {7, 9};
int Motor2[2] = {8, 10};
int MotorEnable = 4;

// Motor 1 = right (passenger side) wheel, Motor 2 = left (driver side) wheel,
// same as the mini project. Motor 2 is mounted mirrored, so driveMotors gives
// it the opposite direction pin level. With that, positive voltage drives each
// wheel forward and its encoder counts up. Leave these false unless that changes.
bool FlipM1 = false;
bool FlipM2 = false;

int M1EncA = 2;
int M1EncB = 5;
int M2EncA = 3;
int M2EncB = 6;

volatile long M1Pos = 0;
volatile long M2Pos = 0;
const float CountsPerRev = 3200.0;

unsigned long desired_Ts = 10;          // ms
float dt = desired_Ts / 1000.0;         // s
unsigned long last_time, start_time;

enum State { WAIT, TURN, DRIVE, DONE };
State state = WAIT;

float PrevTh1 = 0, PrevTh2 = 0;
float phi_des = 0, rho_des = 0;
float rho_start = 0;         // rho when DRIVE begins, so the turn doesn't count as distance
float settle_timer = 0;
float phase_timer = 0;       // s since the current TURN or DRIVE phase began

float M1CarpetVScale = 1.1;
float M2CarpetVScale = 1.1;

float M1TileVScale = 1;
float M2TileVScale = 1;


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
  Serial.println("t\tstate\tphi_des\tphi\trho_des\trho\tphi_dot_d\tphi_dot\trho_dot_d\trho_dot\tVa1\tVa2");

  digitalWrite(MotorEnable, HIGH);
  last_time = millis();
  start_time = last_time;
}

void loop() {
  if (state == DONE) return;

  float t = (float)(last_time - start_time) / 1000.0;

  noInterrupts();
  long M1Count = M1Pos;
  long M2Count = M2Pos;
  interrupts();

  float th1 = 2.0 * PI * (float)M1Count / CountsPerRev;
  float th2 = 2.0 * PI * (float)M2Count / CountsPerRev;
  float thdot1 = (th1 - PrevTh1) / dt;
  float thdot2 = (th2 - PrevTh2) / dt;
  PrevTh1 = th1;
  PrevTh2 = th2;

  float rho_dot = r * (thdot1 + thdot2) / 2.0;
  float phi_dot = r * (thdot1 - thdot2) / d;
  float rho = r * (th1 + th2) / 2.0 - rho_start;
  float phi = r * (th1 - th2) / d;

  // State machine: set the outer loop goals
  bool still = abs(rho_dot) < StillRate && abs(phi_dot) < StillRate;
  switch (state) {
    case WAIT:
      phi_des = 0;
      rho_des = 0;
      if (t >= StartDelay) {
        state = TURN;
        phi_des = PhiGoalDeg * PI / 180.0;
        settle_timer = 0;
        phase_timer = 0;
      }
      break;

    case TURN:
      rho_des = rho;   // no forward motion while turning (distance loop error = 0)
      phase_timer += dt;
      if (still) {
        settle_timer += dt;
      } else {
        settle_timer = 0;
      }
      if ((abs(phi_des - phi) < TurnTol && settle_timer >= SettleTime) ||
          settle_timer >= StuckTime || phase_timer >= TurnTimeout) {
        state = DRIVE;
        rho_start += rho;   // start measuring distance from here
        rho = 0;
        rho_des = DistGoalFt * FtToM;
        settle_timer = 0;
        phase_timer = 0;
      }
      break;

    case DRIVE:
      phase_timer += dt;
      if (still) {
        settle_timer += dt;
      } else {
        settle_timer = 0;
      }
      if ((abs(rho_des - rho) < DistTol && settle_timer >= SettleTime) ||
          settle_timer >= StuckTime || phase_timer >= DriveTimeout) {
        driveMotors(0, 0);
        state = DONE;
        Serial.print("Final angle error (deg): ");
        Serial.println((phi_des - phi) * 180.0 / PI, 2);
        Serial.print("Final distance error (in): ");
        Serial.println((rho_des - rho) / 0.0254, 2);
        Serial.println("END");
        return;
      }
      break;

    default:
      break;
  }

  // Outer PD loops -> rate setpoints (zero while waiting)
  float phi_dot_d = 0, rho_dot_d = 0;
  if (state != WAIT) {
    phi_dot_d = Kp_phi_out * (phi_des - phi) - Kd_phi_out * phi_dot;
    rho_dot_d = Kp_rho_out * (rho_des - rho) - Kd_rho_out * rho_dot;
    phi_dot_d = constrain(phi_dot_d, -phidot_max, phidot_max);
    rho_dot_d = constrain(rho_dot_d, -rhodot_max, rhodot_max);
  }

  // Inner P loops
  float Va_bar = Kp_rho_in * (rho_dot_d - rho_dot);
  float dVa = Kp_phi_in * (phi_dot_d - phi_dot);

  float Va1 = M1CarpetVScale * ((Va_bar + dVa) / 2.0);
  float Va2 = M2CarpetVScale * ((Va_bar - dVa) / 2.0);

  // Optional friction kick (skipped for tiny commands so it doesn't buzz at rest)
  if (FrictionVolts > 0) {
    if (abs(Va1) > 0.3) Va1 += (Va1 > 0) ? FrictionVolts : -FrictionVolts;
    if (abs(Va2) > 0.3) Va2 += (Va2 > 0) ? FrictionVolts : -FrictionVolts;
  }

  // Saturate by scaling both wheels together so the turn/forward ratio is kept
  float biggest = max(abs(Va1), abs(Va2));
  if (biggest > BatteryVoltage) {
    Va1 *= BatteryVoltage / biggest;
    Va2 *= BatteryVoltage / biggest;
  }
  if (state == WAIT) {
    Va1 = 0;
    Va2 = 0;
  }
  driveMotors(Va1, Va2);   // Va1 -> right wheel, Va2 -> left wheel

  Serial.print(t, 3);                       Serial.print("\t");
  Serial.print((int)state);                 Serial.print("\t");
  Serial.print(phi_des * 180.0 / PI, 2);    Serial.print("\t");
  Serial.print(phi * 180.0 / PI, 2);        Serial.print("\t");
  Serial.print(rho_des / FtToM, 3);         Serial.print("\t");
  Serial.print(rho / FtToM, 3);             Serial.print("\t");
  Serial.print(phi_dot_d, 3);               Serial.print("\t");
  Serial.print(phi_dot, 3);                 Serial.print("\t");
  Serial.print(rho_dot_d, 3);               Serial.print("\t");
  Serial.print(rho_dot, 3);                 Serial.print("\t");
  Serial.print(Va1, 2);                     Serial.print("\t");
  Serial.println(Va2, 2);

  while (millis() < last_time + desired_Ts) {
    // wait for the sample period
  }
  last_time = millis();
}

void driveMotors(float M1Voltage, float M2Voltage) {
  digitalWrite(Motor1[0], ((M1Voltage > 0) != FlipM1) ? HIGH : LOW);
  int M1PWM = (int)(255.0 * abs(M1Voltage) / BatteryVoltage);
  analogWrite(Motor1[1], min(M1PWM, 255));

  digitalWrite(Motor2[0], ((M2Voltage > 0) != FlipM2) ? LOW : HIGH);   // mirrored, as in the mini project
  int M2PWM = (int)(255.0 * abs(M2Voltage) / BatteryVoltage);
  analogWrite(Motor2[1], min(M2PWM, 255));
}

void M1EncISR() {
  if (digitalRead(M1EncA) == digitalRead(M1EncB)) {
    M1Pos += 2;
  } else {
    M1Pos -= 2;
  }
}

void M2EncISR() {
  if (digitalRead(M2EncA) == digitalRead(M2EncB)) {
    M2Pos -= 2;
  } else {
    M2Pos += 2;
  }
}

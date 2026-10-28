/*
  Got rid of the Demo 1 comments describing everything, going to make a list of changes here:
    1. GoalDeg and GoalFt no longer hard coded values, will get them from the Pi

    2. Basic setup for I2C
  
  List of ideas for how to implement Demo 2:
    1. For goal Deg -> gets value from Pi if it sees the marker (how far off are we from center and how much do we need to turn ->
    verify Pi can do this with Isha), Otherwise set to 30 deg? or we could set the phidot value to some set velocity, don't know how that will 
    affect our controller (check with Jake)

    2. For goal Ft -> can we calculate how far the beacon is away from our camera?
*/

#include <Wire.h>
#define Arduino_Addr 0x08


// ---------------- Demo settings (hard-code before download) ----------------
float PhiGoalDeg;      // deg, + = left (CCW). 0 = task 2 straight test
float DistGoalFt;      // ft, 1 to 10
float StartDelay = 3.0;      // s standing still before moving
// ----------------------------------------------------------------------------


// Inner loop P gains (section 4 above). Kp = (lambda - 1)/K with lambda = 3, i.e.
// Kp*K = 2: inner loops 3x faster than the motors, reaching 2/3 of the setpoint.
//   Kp_rho_in = 2/K_rho = 2/0.053 = 37.8 V per (m/s)
//   Kp_phi_in = 2/K_phi = 2/0.305 = 6.55 V per (rad/s)
// Same as steering_params.m. Recompute if r, d, K or sigma change.
float Kp_rho_in = 37.8;
float Kp_phi_in = 6.55;

// Outer loop PD gains (section 5 above), same names as steering_params.m.
// Kp_phi_out = 6 makes the angle settle with a time constant of about 0.25 s
// once the turn-rate limit is no longer active.
float Kp_phi_out = 6.0;      // (rad/s) per rad
float Kd_phi_out = 0.0;      // add 0.05 to 0.2 if the turn overshoots
float Kp_rho_out = 6.0;      // (m/s) per m  (was 4.0)
float Kd_rho_out = 0.0;

// Integral gains (the "I" in PI). The integral of the position error keeps growing
// while the robot sits short of the goal, so the speed request (and the motor
// voltage) keeps rising until friction is overcome and the error goes to zero.
float Ki_phi_out = 2.5;       // (rad/s) per (rad*s)
float Ki_rho_out = 0.5;       // (m/s) per (m*s)  (was 2.0; bigger = more overshoot)

// Anti-windup: the integral only runs once the robot is close to the goal. Far
// from the goal the P term (and the speed limit) does the work, and letting the
// integral build up there would only cause a big overshoot at the end.
float IntBandPhi = 5.0 * PI / 180.0;   // rad, integrate only within 5 deg of the goal
float IntBandRho = 0.05;               // m, integrate only within 5 cm (2 in) of the goal

// Setpoint limits, same as steering_params.m. These cap what the outer loops can
// ask for, so the wheels don't slip and the voltages stay under the battery.
// Because the inner loop reaches 2/3 of its setpoint, the actual cruise speeds
// are about 1.0 rad/s turning and 0.33 m/s (1.1 ft/s) driving.
float phidot_max = 1.0;      // rad/s
float rhodot_max = 0.5;      // m/s

// When each phase ends. Motor friction stops the robot a little short of the
// exact goal, so a phase ends once the robot has STOPPED within its tolerance.
// The heading loop keeps correcting during the drive.
float TurnTol = 3.0 * PI / 180.0;    // rad, start driving once stopped within this
float DistTol = 0.0127;              // m (0.5 in), finish once stopped within this
float StillRate = 0.01;              // m/s and rad/s counted as "stopped"
float SettleTime = 0.3;              // s it must stay stopped
float StuckTime = 3.0;                // s stopped outside tolerance before moving on anyway
float TurnTimeout = 10.0;             // s, start driving after this no matter what
float DriveTimeout = 40.0;           // s, stop after this no matter what (demo limit is 60 s)

// Optional friction kick (V per wheel) to close the last bit of error. 0 = off.
// Try 0.5 if the robot stops short; lower it if it buzzes at the end.
float FrictionVolts = 0.5;

// Robot geometry (same as steering_params.m). r turns encoder angle into distance,
// d turns the wheel difference into heading. If the robot goes consistently too far
// or too short, r is off; if it over- or under-turns, d is off.
float r = 0.075;             // wheel radius (m), 15.4 cm wheel diameter (measured)
float d = 0.3289;             // wheel separation (m), 34.7 cm (measured)

float BatteryVoltage = 7.8;   // V, used to turn a voltage command into a PWM duty cycle
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
const float CountsPerRev = 3200.0;   // encoder counts per wheel revolution

unsigned long desired_Ts = 10;          // ms, control loop runs at 100 Hz
float dt = desired_Ts / 1000.0;         // s
unsigned long last_time, start_time;

enum State { WAIT, TURN, DRIVE, DONE };
State state = WAIT;

float PrevTh1 = 0, PrevTh2 = 0;
float phi_des = 0, rho_des = 0;
float rho_start = 0;         // rho when DRIVE begins, so the turn doesn't count as distance
float settle_timer = 0;
float phase_timer = 0;       // s since the current TURN or DRIVE phase began
float phi_int = 0;           // integral of the angle error (rad*s)
float rho_int = 0;           // integral of the distance error (m*s)

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
  Serial.println("t\tstate\tphi_des\tphi\trho_des\trho\tphi__d\tphi_dot\trho_dot_d\trho_dot\tVa1\tVa2");

  digitalWrite(MotorEnable, HIGH);
  last_time = millis();
  start_time = last_time;

  Wire.begin(Arduino_Addr);
  Wire.onReceive(CV_Info);
}

// 1st byte of info -> stores when we detect the beacon (for test 1 and 2) and what direction the arrow it sees is (for test 2)
// Idea for how to set up this first byte could be to have bit 0 be whether we detected a marker, while bits 1 and 2 are for the arrow
// TT for bits 1 and 2: 00 -> doesn't detect an arrow, 01 -> left facing arrow, 10 -> right facing arrow (essentially flip bit 1 if we need to go left, flip bit 2 if we need to go right)
// note: in order to get test 1 to work, should add a flag that prevents us from entering the turn in the arduino once we reach the beacon
volatile bool beacon_detected = 0;
// 0 means we need to go left, 1 means we need to go right
volatile bool arrow_direction = 0;
// 0 means we need to turn, 1 means don't turn
volatile bool no_turn = 1;
// Bytes 2-5 of info -> stores how far away the ardunio is from the beacon (note units between the Pi and Arduino)
volatile float distance_away = 0;
// bytes 6-9 of info -> stores the angle the displacement of the marker once the robot detects the marker
volatile float angle_away = 0;
// Flag that checks if we got new data, used in conjuction with turning interrupts off when we store the volatile data into a stable data type so it can't change on us mid-read
volatile bool new_Data = 0;
// note: distance_away and anlge_away are transferred as floats which automatically account for negative values, not sure what the #.# format is, look it up to make sure we code it correctly on the Pi side

void CV_Info(int numBytes) {
  // checks how many bytes we got, if it != 9, then drain then get rid of the data (quickly read it without storing it anywhere)
  if (numBytes != 9) {
    while (Wire.available()) Wire.read();
    return;
  }

  // checking individual bits for the values we need (5 extra bits for more information to carry in future if needed)
  byte beacon_info = Wire.read();
  beacon_detected = bitRead(beacon_info, 0);
  if (bitRead(beacon_info, 1) == 1) {
    arrow_direction = 0;
    no_turn = 0;
  } else if (bitRead(beacon_info, 2) == 1) {
    arrow_direction = 1;
    no_turn = 0;
  } else {
    no_turn = 1;
  }

  // complicated since we are communicating floating point values (4 bytes for each float value, sign is transferred automatically)
  float d, a;
  uint8_t geo_values[8];
  for (int i = 0; i < 8; i++) geo_values[i] = Wire.read();
  memcpy(&d, &geo_values[0], sizeof(float));
  memcpy(&a, &geo_values[4], sizeof(float));
  distance_away = d;
  angle_away = a;
  new_Data = 1;
}


void loop() {
  // I2C code to extract values for our data
  if (new_Data) {
    // Copy with interrupts off so a new packet can't change values mid-read
    noInterrupts();
    bool b_det = beacon_detected;
    bool n_turn = no_turn;
    bool a_dir = arrow_direction;
    float dist = distance_away;
    float  ang  = angle_away;
    new_Data = 0;
    interrupts();
  }

  if (state == DONE) return;

  // Time since power-up in seconds (for the log and the start delay)
  float t = (float)(last_time - start_time) / 1000.0;

  // Copy the encoder counts with interrupts off so an ISR can't change them
  // halfway through reading a 4-byte long
  noInterrupts();
  long M1Count = M1Pos;
  long M2Count = M2Pos;
  interrupts();

  // Wheel angles (rad) from counts, then wheel speeds (rad/s) by finite difference:
  // thetadot = (theta_now - theta_last) / dt
  float th1 = 2.0 * PI * (float)M1Count / CountsPerRev;
  float th2 = 2.0 * PI * (float)M2Count / CountsPerRev;
  float thdot1 = (th1 - PrevTh1) / dt;
  float thdot2 = (th2 - PrevTh2) / dt;
  PrevTh1 = th1;
  PrevTh2 = th2;

  // Kinematics (section 1): wheel motion -> robot forward motion and heading.
  // rho_start is subtracted so distance is measured from where the DRIVE began.
  float rho_dot = r * (thdot1 + thdot2) / 2.0;
  float phi_dot = r * (thdot1 - thdot2) / d;
  float rho = r * (th1 + th2) / 2.0 - rho_start;
  float phi = r * (th1 - th2) / d;

  // State machine: set the outer loop goals
  // "still" = no encoder movement this sample (one encoder count in 10 ms is
  // already 0.0076 m/s or 0.044 rad/s, above StillRate)
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
      // Turning in place: hold the distance setpoint equal to where we are, so the
      // distance loop asks for zero forward speed and only the angle loop acts.
      rho_des = rho;   // no forward motion while turning (distance loop error = 0)
      phase_timer += dt;
      if (still) {
        settle_timer += dt;
      } else {
        settle_timer = 0;
      }
      // Move on once stopped near the angle, or if stuck/too slow (see header)
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
      // Done once stopped within 0.5 in, or if stuck/too slow
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

  // Outer PD loops -> rate setpoints (section 5). Position error in, speed request out.
  //   P term: Kp * (goal - position)  -> ask for more speed the farther away we are
  //   D term: -Kd * measured speed    -> brakes the approach to reduce overshoot
  // Then clamp to the speed limits. Zero while waiting.
  float phi_dot_d = 0, rho_dot_d = 0;
  if (state != WAIT) {
    float e_phi = phi_des - phi;
    float e_rho = rho_des - rho;

    // Integrate the error (rectangle rule, integral += error*dt), but only near
    // the goal; outside the band it is reset to 0. The distance integral only
    // runs while driving, since rho_des just follows rho during the turn.
    if (abs(e_phi) < IntBandPhi) {
      phi_int += e_phi * dt;
    } else {
      phi_int = 0;
    }
    if (state == DRIVE && abs(e_rho) < IntBandRho) {
      rho_int += e_rho * dt;
    } else {
      rho_int = 0;
    }

    phi_dot_d = Kp_phi_out * e_phi + Ki_phi_out * phi_int - Kd_phi_out * phi_dot;
    rho_dot_d = Kp_rho_out * e_rho + Ki_rho_out * rho_int - Kd_rho_out * rho_dot;

    // Speed limits. If a request hits its limit, undo this sample's integration
    // so the integral can't keep growing while it has no effect (anti-windup,
    // same idea as the mini project's PI code).
    if (abs(phi_dot_d) > phidot_max) {
      phi_int -= e_phi * dt;
      phi_dot_d = constrain(phi_dot_d, -phidot_max, phidot_max);
    }
    if (abs(rho_dot_d) > rhodot_max) {
      rho_int -= e_rho * dt;
      rho_dot_d = constrain(rho_dot_d, -rhodot_max, rhodot_max);
    }
  }

  // Inner P loops (section 4). Speed error in, voltage out:
  //   Va_bar (sum) sets forward speed, dVa (difference) sets turning rate.
  float Va_bar = Kp_rho_in * (rho_dot_d - rho_dot);
  float dVa = Kp_phi_in * (phi_dot_d - phi_dot);

  // Back to individual wheel voltages (section 2):
  // Va1 = right wheel, Va2 = left wheel. A positive dVa gives the right wheel
  // more voltage than the left, so the robot turns left (phi increases).
  float Va1 = (Va_bar + dVa) / 2.0;
  float Va2 = (Va_bar - dVa) / 2.0;

  // Friction kick: real motors need some minimum voltage before they move at all,
  // which the linear model above ignores. Adding FrictionVolts in the direction of
  // the command lets small corrections near the goal actually move the wheels.
  // Skipped for tiny commands so it doesn't buzz at rest.
  if (FrictionVolts > 0) {
    if (abs(Va1) > 0.3) Va1 += (Va1 > 0) ? FrictionVolts : -FrictionVolts;
    if (abs(Va2) > 0.3) Va2 += (Va2 > 0) ? FrictionVolts : -FrictionVolts;
  }

  // Saturation: neither wheel can get more than the battery voltage. Scaling BOTH
  // wheels by the same factor keeps the ratio between them, so the robot still
  // follows the same arc (just slower) instead of the turn being cut off.
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

  // Wait out the rest of the 10 ms period so dt is constant (the speeds and the
  // gains all assume a fixed sample time)
  while (millis() < last_time + desired_Ts) {
    // wait for the sample period
  }
  last_time = millis();
}

// Voltage command -> motor driver. The sign of the voltage sets the direction pin,
// the size sets the PWM duty cycle: duty = |V| / BatteryVoltage, scaled to 0-255.
// Motor 2 is mounted mirrored, so its direction pin is the opposite level.
void driveMotors(float M1Voltage, float M2Voltage) {
  digitalWrite(Motor1[0], ((M1Voltage > 0) != FlipM1) ? HIGH : LOW);
  int M1PWM = (int)(255.0 * abs(M1Voltage) / BatteryVoltage);
  analogWrite(Motor1[1], min(M1PWM, 255));

  digitalWrite(Motor2[0], ((M2Voltage > 0) != FlipM2) ? LOW : HIGH);   // mirrored, as in the mini project
  int M2PWM = (int)(255.0 * abs(M2Voltage) / BatteryVoltage);
  analogWrite(Motor2[1], min(M2PWM, 255));
}

// Encoder interrupts: run on every change of channel A. Comparing A and B tells
// the direction (quadrature). Each A change is 2 of the 3200 counts per rev.
// Motor 2's checks are flipped because it faces the other way, so both count up
// when the robot rolls forward.
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

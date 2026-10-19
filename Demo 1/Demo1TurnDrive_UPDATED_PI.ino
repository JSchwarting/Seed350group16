/*
  SEED Lab - Group 16
  Demo 1 - Tasks 2 and 3: turn to an angle, then drive a distance.

  *** UPDATED VERSION (Oct 16): outer loops changed from PD to PI ***
  The original Demo1TurnDrive.ino is unchanged. Differences from it:
    - Ki_phi_out = 2.0 and Ki_rho_out = 0.5 added (integral of the position error),
      so friction can no longer leave the robot stopped a little short of the goal.
    - Anti-windup: the integral only runs within 5 deg / 5 cm of the goal and stops
      growing while the speed request is at its limit (see section 5).
    - StuckTime raised from 1.0 to 3.0 s so the integral has time to push through
      friction before a phase is cut off.
    - Kp_rho_out raised from 4.0 to 6.0 (Oct 19, matches steering_params.m: with
      Ki_rho_out = 0.5 the Simulink model overshoots only ~0.06 in).
    Everything else (other gains, r, d, goals, pins) is the same as the original.

  ============================== HOW IT WORKS ==============================

  1) WHAT WE MEASURE (encoders -> wheel angles -> robot motion)
     Each encoder gives 3200 counts per wheel revolution, so a wheel angle is
         theta = 2*pi * counts / 3200            (rad)
     Motor 1 is the RIGHT wheel and Motor 2 is the LEFT wheel. Both count up
     when their wheel rolls forward. From the two wheel angles we get the robot's
     forward distance rho and heading phi (handout kinematics):
         rho     = r*(theta1 + theta2)/2         (m)    average wheel travel
         phi     = r*(theta1 - theta2)/d         (rad)  right wheel ahead = turned left
         rho_dot = r*(thetadot1 + thetadot2)/2   (m/s)
         phi_dot = r*(thetadot1 - thetadot2)/d   (rad/s)
     r = wheel radius, d = distance between the wheels. phi is positive
     counterclockwise (turning left), matching the demo's sign convention.

  2) WHAT WE CONTROL (two "virtual" voltages instead of two wheel voltages)
     Instead of thinking about Va1 (right) and Va2 (left) separately, we use
         Va_bar = Va1 + Va2     (sum)        -> mostly makes the robot go forward
         dVa    = Va1 - Va2     (difference) -> mostly makes the robot turn
     and convert back at the end:  Va1 = (Va_bar + dVa)/2,  Va2 = (Va_bar - dVa)/2.
     This splits the robot into two independent single-input systems.

  3) THE PLANT (transfer functions from the step-response model)
     Each motor behaves like  thetadot/V = K*sigma/(s + sigma)  with K = 1.375
     rad/s per V and sigma = 14 1/s (mini project fit). Putting two of them
     through the kinematics above gives first-order transfer functions:
         G_rho(s) = rho_dot(s)/Va_bar(s) = K_rho*sigma/(s + sigma),  K_rho = r*K/2
         G_phi(s) = phi_dot(s)/dVa(s)    = K_phi*sigma/(s + sigma),  K_phi = r*K/d
     With r = 0.077 m and d = 0.347 m: K_rho = 0.053 (m/s)/V, K_phi = 0.305 (rad/s)/V.
     Position is the integral of speed: rho = rho_dot/s, phi = phi_dot/s.

  4) INNER LOOPS: proportional (P) speed control
         Va_bar = Kp_rho_in * (rho_dot_d - rho_dot)
         dVa    = Kp_phi_in * (phi_dot_d - phi_dot)
     Closing a P loop around a first-order plant gives another first-order system:
         rho_dot/rho_dot_d = Kp*K_rho*sigma / (s + sigma*(1 + Kp*K_rho))
     We pick Kp so that Kp*K = 2 (Kp_rho_in = 2/0.053 = 37.8, Kp_phi_in = 2/0.305 = 6.55):
       - the closed-loop pole moves from sigma = 14 to 3*sigma = 42 1/s
         (3x faster, time constant about 24 ms)
       - the steady-state gain is Kp*K/(1 + Kp*K) = 2/3, so the robot only reaches
         2/3 of the speed it is asked for. That's fine: the outer loops below see
         the leftover position error and keep pushing until it is gone.
     These match the Simulink inner-loop model (steering_inner.slx).

  5) OUTER LOOPS: PI position control (plus an optional D term, Kd = 0 now)
         phi_dot_d = Kp_phi_out*e_phi + Ki_phi_out*integral(e_phi dt) - Kd_phi_out*phi_dot
         rho_dot_d = Kp_rho_out*e_rho + Ki_rho_out*integral(e_rho dt) - Kd_rho_out*rho_dot
     with e_phi = phi_des - phi and e_rho = rho_des - rho.
     The outer controller looks at the position error and asks the inner loop
     for a speed. Because the plant integrates speed into position (the 1/s),
     the loop drives the position error to zero on its own in the ideal model.
     On the real robot, friction means a small speed request makes no motion, so
     P alone can stall a little short. The integral fixes that: while an error
     remains, integral(e dt) keeps growing, the speed request keeps rising, and the
     robot creeps the rest of the way. In Laplace terms C(s) = Kp + Ki/s, which adds
     a zero at s = -Ki/Kp (-0.33 for phi, -0.083 for rho), well below the loop
     bandwidth, so the fast response is mostly unchanged.
     Anti-windup: integrating during the whole 15 ft drive would build a huge
     integral and overshoot. So the integral is held at 0 until the robot is within
     IntBandPhi / IntBandRho of the goal, and a sample is "undone" whenever the
     speed request is clamped at phidot_max / rhodot_max.
     Treating the fast inner loop as a gain of 2/3, the angle loop is roughly
         phi/phi_des ~= a/(s + a),  a = Kp_phi_out*2/3 = 6*2/3 = 4 1/s  (tau = 0.25 s)
     and the distance loop has a = Kp_rho_out*2/3 = 6*2/3 = 4 1/s (tau = 0.25 s).
     The D term is applied to the measured rate (phi_dot, rho_dot) instead of the
     error, so a sudden setpoint change does not cause a derivative "kick".
     Kd = 0 right now (pure P). Raise Kd_phi_out to 0.05-0.2 if the turn overshoots.
     The speed requests are capped (phidot_max, rhodot_max), so a big turn or long
     drive runs at a steady speed, then slows down smoothly as the error shrinks.

  6) SEQUENCE (state machine)
      WAIT   - stand still for StartDelay s (demo must start within 10 s)
      TURN   - angle loop drives phi to PhiGoalDeg, distance loop holds rho still
      DRIVE  - distance loop drives rho to DistGoalFt; angle loop keeps holding heading
               (this is what keeps it driving straight and self-correcting)
      DONE   - motors off, print final errors
    TURN -> DRIVE once the robot has stopped within TurnTol for SettleTime
    (the Simulink models used a fixed t_drive instead). DRIVE -> DONE the same
    way with DistTol. If friction stalls it outside tolerance for StuckTime, or
    a phase runs past its timeout, it moves on anyway so it can never hang.

  Task 2 (straight line): PhiGoalDeg = 0, DistGoalFt = 1..10
  Task 3 (turn then go):  PhiGoalDeg = angle (+ = left / CCW), DistGoalFt = distance

  Serial output (115200 baud), tab separated, one line per 10 ms:
    t | state | phi_des | phi | rho_des | rho | phi_dot_d | phi_dot | rho_dot_d | rho_dot | Va1 | Va2
  Angles in degrees and distances in feet, so it reads like the demo.
  The run ends with the final errors and a line containing only END.
*/

// ---------------- Demo settings (hard-code before download) ----------------
float PhiGoalDeg = 0.0;      // deg, + = left (CCW). 0 = task 2 straight test
float DistGoalFt = 15.0;      // ft, 1 to 10
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
float Ki_phi_out = 2.0;       // (rad/s) per (rad*s)
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
float phidot_max = 1.5;      // rad/s
float rhodot_max = 0.5;      // m/s

// When each phase ends. Motor friction stops the robot a little short of the
// exact goal, so a phase ends once the robot has STOPPED within its tolerance.
// The heading loop keeps correcting during the drive.
float TurnTol = 3.0 * PI / 180.0;    // rad, start driving once stopped within this
float DistTol = 0.0127;              // m (0.5 in), finish once stopped within this
float StillRate = 0.01;              // m/s and rad/s counted as "stopped"
float SettleTime = 0.3;              // s it must stay stopped
float StuckTime = 3.0;                // s stopped outside tolerance before moving on anyway
float TurnTimeout = 6.0;             // s, start driving after this no matter what
float DriveTimeout = 40.0;           // s, stop after this no matter what (demo limit is 60 s)

// Optional friction kick (V per wheel) to close the last bit of error. 0 = off.
// Try 0.5 if the robot stops short; lower it if it buzzes at the end.
float FrictionVolts = 0.5;

// Robot geometry (same as steering_params.m). r turns encoder angle into distance,
// d turns the wheel difference into heading. If the robot goes consistently too far
// or too short, r is off; if it over- or under-turns, d is off.
float r = 0.075;             // wheel radius (m), 15.4 cm wheel diameter (measured)
float d = 0.34;             // wheel separation (m), 34.7 cm (measured)

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
  Serial.println("t\tstate\tphi_des\tphi\trho_des\trho\tphi_dot_d\tphi_dot\trho_dot_d\trho_dot\tVa1\tVa2");

  digitalWrite(MotorEnable, HIGH);
  last_time = millis();
  start_time = last_time;
}

void loop() {
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

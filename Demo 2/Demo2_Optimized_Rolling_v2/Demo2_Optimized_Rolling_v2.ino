/*
  Demo2_Optimized_Rolling_v2.ino  -  SEED Lab Group 16, Demo 2

  v2 (Nov 12): copy of Demo2_Optimized_Rolling (incl. bob's AlignTol 1 deg) with two
  fixes for stopping too close to the beacon:
    1. While creeping in the 3 ft zone (ARROW), each new Pi distance moves the stop
       point (rho_des), like the old 3 ft stop re-measure but without stopping.
    2. After the arrow is read, APPROACH is capped at ApproachSpeed (0.5) instead of
       rhodot_max (1.6), so the robot can brake before the beacon.
    3. TURN90 aims for 92 deg (Turn90Deg) to fix undershoot, must get within 0.5 deg,
       turns at Turn90Rate (1.0 rad/s) so the wheels don't slip, and may wait up to
       1.5 s if it stalls short.

  (Rolling notes:)

  ROLLING version (Nov 12): copy of Demo2_Optimized_Speed (with bob's 75 deg steps,
  phidot_max 3.0, rhodot_max 1.6). Changes (search "ROLLING:" for the settings):
    1. Much shorter waits: StartDelay 0, SearchPause 0.15 s, SettleTime 0.03 s,
       StuckTime 0.5 s, LookTimeout 0.5 s.
    2. Trial 2 no longer STOPS at 3 ft to read the arrow. It drives toward the
       final stop, and once within ArrowStopFt (3 ft) it slows to ArrowCreepSpeed
       while reading the arrow (ARROW state). As soon as the arrow is decided (one
       red, or the timeout) it speeds back up (APPROACH), stops at StopDistFt, and
       turns 90. No fresh distance re-read at 3 ft any more.


  OPTIMIZED SPEED version (Nov 12): exact copy of Demo2_UPDATED_TwoStage with only
  speed settings changed (search for "SPEED:" to see each one and its old value):
    rhodot_max 0.75 -> 0.8, phidot_max 1.0 -> 1.2, SettleTime 0.2 -> 0.1,
    StartDelay 0.5 -> 0.1, ArrowTimeout 2.0 -> 1.0, DistTol 1 in -> 2 in,
    Turn90Tol 3 -> 5 deg. Logic, gains, and search steps are unchanged.
  If it gets unreliable, put these back one at a time (start with rhodot_max).

  (Based on Demo2_UPDATED_TwoStage:)

  TwoStage version (Nov 10): copy of Demo2_UPDATED_Step45 (60 deg search steps etc.)
  with one change for trial 2: the robot first stops about ArrowStopFt (3 ft) from
  the beacon, reads the arrow there (3 agreeing readings, up to ArrowTimeout = 2 s),
  then drives in to
  StopDistFt and turns 90 deg. The arrow is easier to read from 3 ft than up close.
  Trial 1 (TrialType = 1) is unchanged: it drives straight to StopDistFt.

    TRIAL 2 ORDER: ... ALIGN -> LOOK -> DRIVE (to 3 ft) -> ARROW (read arrow, up to 2 s)
                   -> APPROACH (drive to StopDistFt) -> TURN90 -> DONE

  Step45 version (Nov 9): same as Demo2_UPDATED except SEARCH. The camera can't
  detect the marker while the robot is spinning, so SEARCH now turns 45 deg, stops,
  looks for SearchPause (1 s), and repeats until the Pi reports the beacon.

  Built from bob's Demo 2 draft (Demo1TurnDrive_UPDATED_PI + the I2C starter code).
  The controller (outer PI -> inner P -> wheel voltages) is unchanged from Demo 1.
  What is new is the state machine that gets its goals from the Pi instead of
  hard-coded PhiGoalDeg / DistGoalFt:

    WAIT     stand still for StartDelay (must start moving within 30 s of "go")
    SEARCH   look for SearchPause (1 s) standing still; if nothing, turn SearchStepDeg
             (45 deg) left, stop, look again, repeat until the Pi reports the beacon
    STOP     stop spinning, wait until still, then wait for a FRESH packet
             (one taken after the robot stopped, so no motion blur / old angle)
    ALIGN    turn by the angle the Pi reported, so the beacon is straight ahead
    LOOK     stop, get a fresh packet, read the distance (more accurate when
             the beacon is centered) and any small leftover angle
    DRIVE    drive (distance - StopDistFt), heading loop holds the beacon centered
    ARROW    (trial 2 only) stop and read the arrow direction
    TURN90   (trial 2 only) turn 90 deg toward the arrow
    DONE     motors off

  Changes vs. bob's draft:
    1. Goals come from the Pi through the states above.
    2. I2C packet copied into normal variables every loop (the draft copied them
       into locals inside the if-block, so they disappeared right away).
    3. The ISR's local float was named d, same as the wheel separation d; renamed
       to avoid confusion.
    4. Accepts a 10-byte packet too: smbus2's write_i2c_block_data(addr, cmd, data)
       sends cmd first, so 9 data bytes arrive as 10. The extra first byte is dropped.
    5. A packet counter, so the code can tell "new packet since I stopped" apart
       from "old packet still in the variables".
    6. Integrals reset at every state change so one phase can't kick the next.

  THINGS TO CONFIRM WITH THE VISION TEAM (settings below):
    - distance units (DistToM): this assumes FEET from the camera to the marker
    - angle units and sign (AngleSign): this assumes DEGREES, + = beacon to the LEFT
      (same as Demo 1 task 1)
    - CamOffsetM: how far the camera sits in front of the wheel axle
    - whether the Pi sends packets all the time (with bit 0 = 0 when it sees nothing)
      or only when it sees the marker. Both work, the timeouts cover the second case.
    - Pi side packing should be struct.pack('<Bff', flags, dist, angle) (little-endian,
      same float format as the Arduino).

  Serial log: same 12 tab-separated columns as Demo 1 (state col 2 now goes 0-8).
  Lines starting with '#' are event messages; skip them when importing in MATLAB.
*/

#include <Wire.h>
#define Arduino_Addr 0x08

// ---------------- Demo settings (set before download) ----------------
int TrialType = 2;             // 1 = stop at beacon, 2 = stop then turn 90 deg by arrow
float StartDelay = 0.0;              // ROLLING: was 0.1. s before moving
float StopDistFt = 1.0;        // ft, where to stop (center of rotation to marker). Limit 1.5 ft
float ArrowStopFt = 3.0;       // ft, trial 2: SLOW DOWN here (center to marker) to read the arrow (no stop)
float ApproachSpeed = 0.5;     // m/s speed cap after the arrow is read (APPROACH), so it can brake in time
float ArrowCreepSpeed = 0.15;  // m/s speed request while reading the arrow (~0.1 m/s = 0.33 ft/s actual)
float CamOffsetM = 0.10;       // m the camera sits AHEAD of the wheel axle (MEASURE THIS)
float DistToM = 0.3048;        // Pi distance units -> m (0.3048 if feet, 0.0254 if inches, 1 if m)
float AngleSign = 1.0;         // +1 if Pi angle is + = left, -1 if + = right
float SearchStepDeg = 75.0;    // deg turned between looks while searching
float SearchPause = 0.15;            // ROLLING: was 0.25. s looking after each search step
float SearchDir = 1.0;         // +1 search stepping left (CCW), -1 right
float DefaultTurnDeg = 90.0;   // trial 2: + = left. Used only if no arrow was ever seen
// -----------------------------------------------------------------------

// Inner loop P gains (unchanged from Demo 1). Speed error in, voltage out.
// Kp = 2/K, so each inner loop is about 3x faster than the bare motor and
// reaches 2/3 of its speed setpoint.
float Kp_rho_in = 37.8;
float Kp_phi_in = 6.55;

// Outer loop PI gains (unchanged from the final Demo 1 code, Demo1TurnDrive_UPDATED_PI).
// Position error in, speed request out. The integral (Ki) only runs within
// IntBand of the goal so it fixes the last bit of error without big overshoot.
float Kp_phi_out = 6.0;
float Kd_phi_out = 0.0;
float Kp_rho_out = 6.0;
float Kd_rho_out = 0.0;
float Ki_phi_out = 2.5;
float Ki_rho_out = 0.5;
float IntBandPhi = 5.0 * PI / 180.0;
float IntBandRho = 0.05;

// Speed limits on what the outer loops can request (keeps the wheels from slipping)
float phidot_max = 3.0;              // SPEED: was 1.0. rad/s; 1.5 slipped in Demo 1
float rhodot_max = 1.6;              // SPEED: was 0.75. m/s request, about 0.53 m/s actual (motor max ~0.8)

// Phase end conditions
float AlignTol = 1.0 * PI / 180.0;   // rad, beacon counted as centered (2 deg at 5 ft = 2 in)
float Turn90Deg = 92.0;              // v2: aim 2 deg PAST 90 (it was undershooting)
float Turn90Tol = 0.5 * PI / 180.0;  // v2: was 3 deg. Must get within 0.5 deg of the goal before stopping
float Turn90Rate = 1.0;              // v2: rad/s turn-rate limit for the 90 only. phidot_max (3.0) makes the
                                     // wheels slip, so the encoders think it turned farther than it did
float Turn90StuckTime = 1.5;         // v2: s stalled before giving up on the 90 (integral needs time to close the gap)
float DistTol = 0.0508;              // SPEED: was 0.0254. m (2 in); distance only needs to be within 1.5 ft
float StillRate = 0.01;              // m/s and rad/s counted as "stopped"
float SettleTime = 0.03;             // ROLLING: was 0.1. s (3 loops) stopped before a phase counts as done
float StuckTime = 0.5;               // ROLLING: was 2.0. s stopped outside tolerance before moving on anyway
float PhaseTimeout = 10.0;           // s max for any single turn or drive phase
float LookTimeout = 0.5;             // ROLLING: was 1.5. s to wait for a fresh packet after stopping
float ArrowTimeout = 1.0;            // SPEED: was 2.0. s; green runs wait this long with RedWins
int ArrowConfirm = 3;                // arrow readings in one direction needed to decide (total, not in a row)
bool RedWins = true;                 // true: ANY red reading at the 3 ft stop = turn right right away;
                                     // green (left) only if no red shows up in the whole ArrowTimeout
                                     // (the camera sometimes reads red as green, never the reverse)
float ReAimDeg = 1.0;                // deg, leftover angle in LOOK bigger than this is corrected while driving

// Extra voltage added in the direction of motion so small corrections overcome
// motor friction (same as Demo 1)
float FrictionVolts = 0.5;

// Robot geometry. d = 0.3289 is the final Demo 1 value (the old left/right
// differences turned out to be a loose red wire, not calibration).
float r = 0.075;             // wheel radius (m)
float d = 0.3289;            // distance between the wheels (m)

float BatteryVoltage = 7.8;  // V, full PWM duty cycle
const float FtToM = 0.3048;
const float DegToRad = PI / 180.0;

// Motor driver pins {direction, PWM}. Motor 1 = right wheel, Motor 2 = left wheel
// (mounted mirrored, handled in driveMotors and M2EncISR).
int Motor1[2] = {7, 9};
int Motor2[2] = {8, 10};
int MotorEnable = 4;
bool FlipM1 = false;
bool FlipM2 = false;

// Encoder pins. Channel A is on an interrupt pin (2 or 3).
int M1EncA = 2;
int M1EncB = 5;
int M2EncA = 3;
int M2EncB = 6;

// Encoder counts, changed inside the ISRs (hence volatile)
volatile long M1Pos = 0;
volatile long M2Pos = 0;
const float CountsPerRev = 3200.0;

// Control loop timing: 100 Hz, fixed so the gains and speed estimates stay valid
unsigned long desired_Ts = 10;   // ms
float dt = desired_Ts / 1000.0;
unsigned long last_time, start_time;

// State machine (see the list at the top of the file)
// (APPROACH added before DONE: state numbers 0-7 are the same as before, APPROACH = 8, DONE = 9)
enum State { WAIT, SEARCH, STOP, ALIGN, LOOK, DRIVE, ARROW, TURN90, APPROACH, DONE };
State state = WAIT;

// Controller memory between loops
float PrevTh1 = 0, PrevTh2 = 0;   // last wheel angles, for the speed estimate
float phi_des = 0, rho_des = 0;   // current heading (rad) and distance (m) goals
float settle_timer = 0;           // s the robot has been standing still
float phase_timer = 0;            // s since the current state started
float phi_int = 0;                // integral of heading error (rad*s)
bool search_looking = true;       // SEARCH: true = standing still looking, false = turning a step
float rho_int = 0;                // integral of distance error (m*s)

// Beacon info saved between states
float beacon_dist_m = 0;      // camera distance from the STOP reading (backup for LOOK)
int arrow_seen = 0;           // 0 = never, +1 = left, -1 = right (last arrow the Pi reported)
int turn_dir = 0;             // trial 2: arrow direction chosen at the 3 ft stop
int arrow_left_n = 0;         // ARROW state: LEFT (green) readings so far
int arrow_right_n = 0;        // ARROW state: RIGHT (red) readings so far
unsigned int arrow_pkt = 0;   // ARROW state: last packet already counted
float last_pi_dist = 0;       // ARROW: last Pi distance used to update the stop point
float arrow_dist_m = -1;      // trial 2: camera distance read at the 3 ft stop (-1 = none)
float arrow_ang = 0;          // trial 2: leftover angle read at the 3 ft stop (deg)
unsigned int packet_mark = 0; // packet count when the robot came to a stop

// ---------------- I2C from the Pi ----------------
// Byte 0: bit 0 = beacon detected, bit 1 = left arrow, bit 2 = right arrow
// Bytes 1-4: float distance, bytes 5-8: float angle
volatile bool beacon_detected = 0;
volatile bool arrow_direction = 0;   // 0 = left, 1 = right
volatile bool no_turn = 1;           // 1 = no arrow seen in this packet
volatile float distance_away = 0;
volatile float angle_away = 0;
volatile unsigned int packet_count = 0;   // goes up by 1 for every good packet

void CV_Info(int numBytes) {
  // smbus2 write_i2c_block_data sends a command byte first: drop it
  if (numBytes == 10) {
    Wire.read();
    numBytes = 9;
  }
  if (numBytes != 9) {
    while (Wire.available()) Wire.read();
    return;
  }

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

  float dist_in, ang_in;
  uint8_t geo_values[8];
  for (int i = 0; i < 8; i++) geo_values[i] = Wire.read();
  memcpy(&dist_in, &geo_values[0], sizeof(float));
  memcpy(&ang_in, &geo_values[4], sizeof(float));
  distance_away = dist_in;
  angle_away = ang_in;
  packet_count++;
}

// Latest packet, copied out of the ISR variables every loop
bool b_det = 0, n_turn = 1, a_dir = 0;
float pi_dist = 0, pi_ang = 0;
unsigned int pkt = 0;

// Start a new phase: clear timers and integrals
void enterState(State s) {
  state = s;
  settle_timer = 0;
  phase_timer = 0;
  phi_int = 0;
  rho_int = 0;
  Serial.print("# state ");
  Serial.println((int)s);
}

// Start (or restart) SEARCH: hold this heading and look before turning
void startSearch(float phi_now) {
  phi_des = phi_now;
  search_looking = true;
  packet_mark = packet_count;
  enterState(SEARCH);
}

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

  Wire.begin(Arduino_Addr);
  Wire.onReceive(CV_Info);

  digitalWrite(MotorEnable, HIGH);
  last_time = millis();
  start_time = last_time;
}

void loop() {
  if (state == DONE) return;

  // Copy the latest Pi packet with interrupts off so it can't change mid-copy
  noInterrupts();
  b_det = beacon_detected;
  n_turn = no_turn;
  a_dir = arrow_direction;
  pi_dist = distance_away;
  pi_ang = angle_away;
  pkt = packet_count;
  long M1Count = M1Pos;
  long M2Count = M2Pos;
  interrupts();

  // Remember the last arrow the Pi saw at any point, as a backup for trial 2
  if (b_det && !n_turn) arrow_seen = a_dir ? -1 : 1;

  float t = (float)(last_time - start_time) / 1000.0;

  // Wheel angles (rad) and speeds (rad/s, finite difference over one sample)
  float th1 = 2.0 * PI * (float)M1Count / CountsPerRev;
  float th2 = 2.0 * PI * (float)M2Count / CountsPerRev;
  float thdot1 = (th1 - PrevTh1) / dt;
  float thdot2 = (th2 - PrevTh2) / dt;
  PrevTh1 = th1;
  PrevTh2 = th2;

  // Kinematics: wheels -> robot forward distance/speed (rho) and heading/turn rate (phi, + = left)
  float rho_dot = r * (thdot1 + thdot2) / 2.0;
  float phi_dot = r * (thdot1 - thdot2) / d;
  float rho = r * (th1 + th2) / 2.0;
  float phi = r * (th1 - th2) / d;

  // "Still" = no encoder movement this sample; used to decide when a phase has settled
  bool still = abs(rho_dot) < StillRate && abs(phi_dot) < StillRate;
  if (still) settle_timer += dt; else settle_timer = 0;
  phase_timer += dt;

  // A packet that arrived after the robot settled. Two packets past the mark,
  // so the frame was taken after stopping, not during the motion.
  bool fresh = (unsigned int)(pkt - packet_mark) >= 2;

  // In every state except DRIVE, hold the forward position (turn in place)
  // (ARROW keeps rolling forward now, so it keeps its distance goal too)
  if (state != DRIVE && state != APPROACH && state != ARROW) rho_des = rho;

  switch (state) {
    case WAIT:
      phi_des = phi;
      if (t >= StartDelay) {
        startSearch(phi);   // look first: the beacon may already be in view
      }
      break;

    case SEARCH:
      // Step-and-look search. The camera mostly can't detect the marker while the
      // robot turns (motion blur), so it stops and looks after each step. The first
      // detection counts, whether it comes while looking or while turning.
      if (search_looking) {
        // Looking: hold the heading. phase_timer only starts once the robot is
        // still, and only packets that arrive after that point count.
        if (settle_timer < SettleTime) {
          packet_mark = pkt;
          phase_timer = 0;
        } else if (b_det && pkt != packet_mark) {
          // Seen. STOP waits for 2 fresh packets, then turns to it (ALIGN).
          phi_des = phi;
          enterState(STOP);
        } else if (phase_timer >= SearchPause) {
          // Nothing for a full SearchPause: turn the next step
          search_looking = false;
          phi_des = phi + SearchDir * SearchStepDeg * DegToRad;
          phase_timer = 0;
          packet_mark = pkt;   // so a detection during the turn must be a new packet
          Serial.println("# search step");
        }
      } else {
        // Turning: if the Pi happens to see the beacon mid-turn, take that first
        // detection right away: stop where we are and let STOP get a clean reading.
        if (b_det && pkt != packet_mark) {
          phi_des = phi;
          Serial.println("# seen while turning");
          enterState(STOP);
          break;
        }
        // Turning: done once within 5 deg of the step and stopped, or after 3 s
        if ((abs(phi_des - phi) < 5.0 * DegToRad && settle_timer >= SettleTime) ||
            phase_timer >= 3.0) {
          search_looking = true;
          phase_timer = 0;
          settle_timer = 0;
        }
      }
      break;

    case STOP:
      // Wait until still, then mark the packet count and wait for fresh ones
      if (settle_timer < SettleTime) {
        packet_mark = pkt;
        phase_timer = 0;
      } else if (fresh) {
        if (b_det) {
          beacon_dist_m = pi_dist * DistToM;
          phi_des = phi + AngleSign * pi_ang * DegToRad;
          Serial.print("# beacon at deg ");
          Serial.print(AngleSign * pi_ang, 2);
          Serial.print(" dist m ");
          Serial.println(beacon_dist_m, 3);
          enterState(ALIGN);
        } else {
          startSearch(phi);     // false alarm: keep searching
        }
      } else if (phase_timer > LookTimeout) {
        startSearch(phi);       // Pi said nothing new: keep searching
      }
      break;

    case ALIGN:
      if ((abs(phi_des - phi) < AlignTol && settle_timer >= SettleTime) ||
          settle_timer >= StuckTime || phase_timer >= PhaseTimeout) {
        packet_mark = pkt;
        enterState(LOOK);
      }
      break;

    case LOOK: {
      // Fresh reading with the beacon centered: better distance, small leftover angle
      bool got = fresh && b_det;
      if (got || phase_timer > LookTimeout) {
        float cam_dist = got ? pi_dist * DistToM : beacon_dist_m;
        if (got && abs(pi_ang) > ReAimDeg) phi_des = phi + AngleSign * pi_ang * DegToRad;
        // Camera distance + camera offset = distance from the wheel axle
        // Both trials drive toward the FINAL stop. Trial 2 slows down (ARROW state)
        // once it is within ArrowStopFt of the beacon, without stopping.
        float go = cam_dist + CamOffsetM - StopDistFt * FtToM;
        if (go < 0) go = 0;
        Serial.print("# drive m ");
        Serial.println(go, 3);
        enterState(DRIVE);
        rho_des = rho + go;
      }
      break;
    }

    case DRIVE:
      // Trial 2: once within ArrowStopFt of the beacon, slow to a creep and read the arrow
      if (TrialType == 2 && (rho_des - rho) <= (ArrowStopFt - StopDistFt) * FtToM) {
        arrow_left_n = 0;
        arrow_right_n = 0;
        arrow_pkt = pkt;
        last_pi_dist = pi_dist;   // only distances computed AFTER slowing down are used
        Serial.println("# slowing to read arrow");
        enterState(ARROW);
        break;
      }
      if ((abs(rho_des - rho) < DistTol && settle_timer >= SettleTime) ||
          settle_timer >= StuckTime || phase_timer >= PhaseTimeout) {
        rho_des = rho;
        if (TrialType == 1) {
          enterState(DONE);
        } else {
          packet_mark = pkt;
          arrow_dist_m = -1;
          arrow_left_n = 0;
          arrow_right_n = 0;
          arrow_pkt = pkt;
          enterState(ARROW);   // stopped about ArrowStopFt away: read the arrow
        }
      }
      break;

    // test is we see the arrow when we stop, if not, check if this can cause any errors
    case ARROW: {
      // Use a fresh reading of the arrow if we get one, else the last one seen
      int dir = 0;
      // Rolling slowly toward the beacon. Count every new arrow reading
      // (green = left, red = right); packets with no arrow are skipped.
      if (pkt != arrow_pkt) {
        arrow_pkt = pkt;
        // Re-measure while creeping: the Pi's close-up distance is much better than
        // the one from 5-6 ft, so move the stop point to match it. Only when the Pi's
        // distance value changes (it updates every N_AVG frames).
        if (b_det && pi_dist > 0 && pi_dist != last_pi_dist) {
          last_pi_dist = pi_dist;
          rho_des = rho + pi_dist * DistToM + CamOffsetM - StopDistFt * FtToM;
          Serial.print("# re-measured, m to go ");
          Serial.println(rho_des - rho, 3);
        }
        if (b_det && !n_turn) {
          if (a_dir) arrow_right_n++; else arrow_left_n++;
        }
        Serial.print("# arrow pkt det=");
        Serial.print(b_det);
        Serial.print(" left=");
        Serial.print(arrow_left_n);
        Serial.print(" right=");
        Serial.println(arrow_right_n);
        if (RedWins) {
          if (arrow_right_n >= 1) dir = -1;          // one red is enough
        } else {
          if (arrow_left_n >= ArrowConfirm) dir = 1;
          else if (arrow_right_n >= ArrowConfirm) dir = -1;
        }
      }
      if (dir == 0 && phase_timer > ArrowTimeout) {
        // Timeout: go with whichever side got more readings here, else the last arrow seen.
        // (With RedWins there were no reds, so any green here means left.)
        if (arrow_left_n > arrow_right_n) dir = 1;
        else if (arrow_right_n > arrow_left_n) dir = -1;
        else dir = arrow_seen;
        Serial.println("# arrow timeout");
        if (dir == 0) dir = (DefaultTurnDeg >= 0) ? 1 : -1;   // never saw one: guess
      }
      if (dir != 0) {
        turn_dir = dir;   // remember it, turn after the final approach
        Serial.print("# arrow ");
        Serial.println(dir > 0 ? "LEFT" : "RIGHT");
        // Speed back up and finish the drive to StopDistFt (same distance goal as before)
        Serial.print("# approach m ");
        Serial.println(rho_des - rho, 3);
        enterState(APPROACH);
      }
      break;
    }

    case APPROACH:
      // Drive the last stretch to StopDistFt, then turn toward the arrow
      if ((abs(rho_des - rho) < DistTol && settle_timer >= SettleTime) ||
          settle_timer >= StuckTime || phase_timer >= PhaseTimeout) {
        rho_des = rho;
        phi_des = phi + turn_dir * Turn90Deg * DegToRad;
        enterState(TURN90);
      }
      break;

    case TURN90:
      // Stricter than other turns: within 1 deg of 91 and still. If it stalls short,
      // wait up to Turn90StuckTime so the integral can push it the rest of the way.
      if ((abs(phi_des - phi) < Turn90Tol && settle_timer >= SettleTime) ||
          settle_timer >= Turn90StuckTime || phase_timer >= PhaseTimeout) {
        Serial.print("# turn90 error deg ");
        Serial.println((phi_des - phi) * 180.0 / PI, 2);
        enterState(DONE);
      }
      break;

    default:
      break;
  }

  if (state == DONE) {
    driveMotors(0, 0);
    Serial.println("END");
    return;
  }

  // Outer PI(D) loops, same as Demo 1
  float phi_dot_d = 0, rho_dot_d = 0;
  if (state != WAIT) {
    float e_phi = phi_des - phi;
    float e_rho = rho_des - rho;

    // No angle integral while searching (the goal is a ramp, it would wind up)
    if (state != SEARCH && abs(e_phi) < IntBandPhi) {
      phi_int += e_phi * dt;
    } else {
      phi_int = 0;
    }
    if ((state == DRIVE || state == APPROACH || state == ARROW) && abs(e_rho) < IntBandRho) {
      rho_int += e_rho * dt;
    } else {
      rho_int = 0;
    }

    phi_dot_d = Kp_phi_out * e_phi + Ki_phi_out * phi_int - Kd_phi_out * phi_dot;
    rho_dot_d = Kp_rho_out * e_rho + Ki_rho_out * rho_int - Kd_rho_out * rho_dot;

    // The final 90 turns slower so the wheels don't slip (slip = real undershoot)
    float phi_lim = (state == TURN90) ? Turn90Rate : phidot_max;
    if (abs(phi_dot_d) > phi_lim) {
      phi_int -= e_phi * dt;
      phi_dot_d = constrain(phi_dot_d, -phi_lim, phi_lim);
    }
    // While reading the arrow, cap the forward speed at a slow creep
    float rho_lim = (state == ARROW) ? ArrowCreepSpeed
                  : (state == APPROACH) ? ApproachSpeed : rhodot_max;
    if (abs(rho_dot_d) > rho_lim) {
      rho_int -= e_rho * dt;
      rho_dot_d = constrain(rho_dot_d, -rho_lim, rho_lim);
    }
  }

  // Inner P loops, same as Demo 1: Va_bar (sum of wheel voltages) sets forward
  // speed, dVa (difference) sets turn rate. Then split into right (Va1) / left (Va2).
  float Va_bar = Kp_rho_in * (rho_dot_d - rho_dot);
  float dVa = Kp_phi_in * (phi_dot_d - phi_dot);
  float Va1 = (Va_bar + dVa) / 2.0;
  float Va2 = (Va_bar - dVa) / 2.0;

  if (FrictionVolts > 0) {
    if (abs(Va1) > 0.3) Va1 += (Va1 > 0) ? FrictionVolts : -FrictionVolts;
    if (abs(Va2) > 0.3) Va2 += (Va2 > 0) ? FrictionVolts : -FrictionVolts;
  }

  // Saturation: scale both wheels together so the turn ratio is kept
  float biggest = max(abs(Va1), abs(Va2));
  if (biggest > BatteryVoltage) {
    Va1 *= BatteryVoltage / biggest;
    Va2 *= BatteryVoltage / biggest;
  }
  if (state == WAIT) {
    Va1 = 0;
    Va2 = 0;
  }
  driveMotors(Va1, Va2);

  // 12-column log, same format as Demo 1 (115200 baud)
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

  // Wait out the rest of the 10 ms period so dt stays constant
  while (millis() < last_time + desired_Ts) {
  }
  last_time = millis();
}

// Voltage command -> motor driver. The sign sets the direction pin, the size sets
// the PWM duty (|V| / BatteryVoltage scaled to 0-255). Motor 2 is mirrored, so
// its direction pin is the opposite level.
void driveMotors(float M1Voltage, float M2Voltage) {
  digitalWrite(Motor1[0], ((M1Voltage > 0) != FlipM1) ? HIGH : LOW);
  int M1PWM = (int)(255.0 * abs(M1Voltage) / BatteryVoltage);
  analogWrite(Motor1[1], min(M1PWM, 255));

  digitalWrite(Motor2[0], ((M2Voltage > 0) != FlipM2) ? LOW : HIGH);
  int M2PWM = (int)(255.0 * abs(M2Voltage) / BatteryVoltage);
  analogWrite(Motor2[1], min(M2PWM, 255));
}

// Encoder ISRs: run on every change of channel A. Comparing A to B gives the
// direction. Motor 2's signs are flipped so both wheels count up going forward.
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

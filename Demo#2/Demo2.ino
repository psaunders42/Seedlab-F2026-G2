// Name: Tomas Padilla, Travis C. Plumb
// Class: EENG350 F26 Team 2 - Controls
// Date: 11/09/2026
// Purpose: Finds the ArUco beacon using CV data sent from the Pi over I2C, drives to within 1.5 ft
// of it in one continuous motion, and optionally turns 90 degrees toward the arrow. Uses a steering
// controller (heading and forward speed) for the search and approach, and the Demo 1 nested PI
// position controller for the final stop and the 90 degree turn.
// I2C from Pi: [0x10, flags, seq, angle (0.01 deg, int16), range (mm, uint16)] every frame.
// flags bit 0 = marker found, bits 1-2 = arrow (1 left, 2 right).
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Trial settings
const bool TURN_AT_BEACON = false;  // true = turn 90 deg toward the arrow after stopping
#define ARROW_TEST_MODE 1           // 1 = only test the arrow turn (type L, R, or U in serial monitor)
#define FAKE_BEACON_MODE 0          // 1 = test without the camera using a virtual beacon

// Placeholder measurements, update once the robot is finished
constexpr float CAMERA_X_IN     = 6.0;   // PLACEHOLDER: camera distance in front of the axle (in)
constexpr float CAMERA_Y_IN     = 0.0;   // PLACEHOLDER: camera offset left of center (in)
constexpr float SWEEP_RADIUS_IN = 10.0;  // PLACEHOLDER: axle midpoint to farthest point on robot (in)
constexpr float STANDOFF_IN     = 13.5;  // PLACEHOLDER: stop distance from the beacon (in)
const float SEARCH_RATE_DEG_S   = 30.0;  // PLACEHOLDER: search rotation speed
constexpr unsigned long VISION_LATENCY_MS = 150;  // PLACEHOLDER: camera to I2C delay (ms)

// Robot measureables
constexpr float WHEEL_RADIUS_IN = 3.0;
constexpr float WHEEL_BASE_IN   = 14.0;

// Trim adjustments for uneven wheel movement (left/right swapped from Demo 1)
constexpr float RIGHT_TRIM = 1.023;
constexpr float LEFT_TRIM  = 1.013;
constexpr float TURN_TRIM  = 0.995;

// Wheel radius and wheel base with trims applied
constexpr float R_RIGHT_IN = WHEEL_RADIUS_IN / RIGHT_TRIM;
constexpr float R_LEFT_IN  = WHEEL_RADIUS_IN / LEFT_TRIM;
constexpr float BASE_EFF_IN = (R_RIGHT_IN + R_LEFT_IN) * TURN_TRIM * WHEEL_BASE_IN /
                              (2.0 * WHEEL_RADIUS_IN);

// Stop distance checks
constexpr float LIMIT_IN          = 18.0;
constexpr float LIMIT_MARGIN_IN   = 3.0;
constexpr float TURN_CLEARANCE_IN = 2.0;
static_assert(STANDOFF_IN <= LIMIT_IN - LIMIT_MARGIN_IN,
              "STANDOFF_IN too large: robot could stop outside 1.5 ft");
static_assert(STANDOFF_IN >= SWEEP_RADIUS_IN + TURN_CLEARANCE_IN,
              "STANDOFF_IN too small for SWEEP_RADIUS_IN: robot could hit the beacon");

// Search and approach
constexpr int SEARCH_DIR = +1;             // +1 = turn left, -1 = turn right
static_assert(SEARCH_DIR == 1 || SEARCH_DIR == -1, "SEARCH_DIR must be +1 or -1");

const float DRIVE_SPEED_IN_S   = 8.0;
const float MAX_TURN_RATE_DEG_S = 60.0;
const float K_HEADING          = 3.0;
const float K_DISTANCE         = 2.0;
const float HEADING_GATE_DEG   = 20.0;     // no forward speed until heading error is under this
const float FORWARD_ACCEL_IN_S2 = 15.0;
const float TURN_ACCEL_RAD_S2   = 4.0;
const float TARGET_FILTER_ALPHA = 0.3;
const float MAX_TARGET_DIST_IN  = 100.0;

// Final drive on the position controller
const float FINAL_DRIVE_IN        = 4.0;
const float FINAL_HEADING_TOL_DEG = 4.0;

// Vision timing
const unsigned long STARTUP_WAIT_MS       = 15000;
const unsigned long FIRST_LOOK_FLUSH_MS   = 250;
const int           FIRST_FOUND_FRAMES    = 4;
const int           FIRST_NOT_FOUND_LIMIT = 8;
const unsigned long FIRST_LOOK_TIMEOUT_MS = 3000;

// Pose history for camera delay
constexpr unsigned long POSE_STEP_MS = 20;
constexpr byte          POSE_HISTORY_LEN = 24;
constexpr unsigned long POSE_HISTORY_MS = POSE_STEP_MS * (POSE_HISTORY_LEN - 1);
static_assert(VISION_LATENCY_MS <= POSE_HISTORY_MS,
              "VISION_LATENCY_MS is longer than the pose history; raise POSE_HISTORY_LEN");

// Arrow
const float         ARROW_TURN_DEG        = 90.0;
const float         ARROW_READ_DIST_IN    = 12.0;  // read the arrow within this distance of the stop
const byte          ARROW_READINGS_NEEDED = 2;
const unsigned long ARROW_LOOK_TIMEOUT_MS = 1500;

// Motor control pins
const int enablePin = 4;
const int signPin[2] = {7, 8};
const int pwmPin[2] = {9, 10};

// Encoder pins
const int Apin = 2;  // m1 interrupt
const int Bpin = 5;  // m1 digital
const int Cpin = 3;  // m2 interrupt
const int Dpin = 6;  // m2 digital

// array mapping, robot drives camera first (Demo 1: 0, 1, {1, 1}, {1, 1})
const int RIGHT_IDX = 1;
const int LEFT_IDX = 0;
const int MOTOR_DIR[2] = {-1, -1};
const int ENCODER_DIR[2] = {-1, -1};

// Control
const unsigned long SAMPLE_MS = 10;
const float COUNTS_PER_REV = 3200.0;
const float Kp_pos = 7;
const float Ki_pos = 0.2;
const float Kp = 2;
const float Battery_Voltage = 7.8;
const float INTEGRAL_LIMIT = 2.0;
const float TURN_MAX_WHEEL_SPEED = 1.5;
const float DRIVE_MAX_WHEEL_SPEED = 3.0;
const float MAX_WHEEL_ACCEL = 12.0;
const float K_SYNC = 2.0;
const float SPEED_FF_V_PER_RAD_S = 0.0;    // PLACEHOLDER: 1/K from motor model, 0 = off

// completion conditions
const float POS_TOL_RAD = 0.04;
const float SPEED_TOL_RAD_S = 0.25;
const int SETTLE_SAMPLES = 10;

const float DEG2RAD = PI / 180.0;
const float MM_PER_IN = 25.4;

// I2C communication
const byte ARD_ADDR = 8;
const byte CMD_MEASUREMENT = 0x10;
const byte CMD_STOP = 0x12;
const byte MEAS_PAYLOAD_LEN = 6;

const byte ARROW_UNKNOWN = 0;
const byte ARROW_LEFT = 1;
const byte ARROW_RIGHT = 2;

const byte ARROW_FALLBACK = ARROW_LEFT;  // turn used if no arrow is seen

// Virtual beacon for FAKE_BEACON_MODE, relative to the robot's start
const float         FAKE_BEACON_DIST_IN     = 54.0;
const float         FAKE_BEACON_BEARING_DEG = -90.0;
const byte          FAKE_ARROW              = ARROW_RIGHT;
const unsigned long FAKE_FRAME_MS           = 100;
const float         FAKE_HALF_FOV_DEG       = 30.0;
const float         FAKE_ARROW_RANGE_IN     = 30.0;

volatile byte rx_meas_buf[MEAS_PAYLOAD_LEN];
volatile bool rx_meas_ready = false;
volatile bool rx_stop = false;
volatile byte tx_state = 0;
volatile byte tx_arrow = ARROW_UNKNOWN;

// navigation states for state machine
enum State : byte {
  ST_STARTUP     = 0,
  ST_FIRST_LOOK  = 1,
  ST_SEARCH      = 2,
  ST_APPROACH    = 3,
  ST_FINAL_DRIVE = 4,
  ST_AT_BEACON   = 5,
  ST_TURN_90     = 6,
  ST_DONE        = 7
};

enum ControlMode : byte { CTRL_OFF, CTRL_VELOCITY, CTRL_POSITION };

enum MoveType : byte { MOVE_NONE, MOVE_TURN, MOVE_DRIVE };

enum LookResult : byte { LOOK_PENDING, LOOK_FOUND, LOOK_MISSED };

struct Frame {
  bool found;
  byte arrow;
  float angle_deg;
  float range_in;
};

struct Pose {
  float x, y, phi;
};

// Encoder counts
volatile long rotation_count = 0;
volatile long rotation_count2 = 0;
long prev_pos1_counts = 0;
long prev_pos2_counts = 0;

// Timing variables
unsigned long last_time_ms = 0;

// Wheel control
float desired_speed[2] = {0, 0};
float actual_speed[2] = {0, 0};
float desired_pos[2] = {0, 0};
float actual_pos[2] = {0, 0};
float integral_error[2] = {0, 0};
float pos_error[2] = {0, 0};
float error[2] = {0, 0};
float Voltage[2] = {0, 0};
unsigned int PWM[2] = {0, 0};

// Robot bearings
float robot_x = 0.0;
float robot_y = 0.0;
float robot_phi = 0.0;
float prev_odom_right = 0.0;
float prev_odom_left = 0.0;
bool odom_initialized = false;

// Pose history
Pose pose_hist[POSE_HISTORY_LEN];
byte pose_head = 0;
byte pose_count = 0;
unsigned long last_pose_ms = 0;

// Steering commands
ControlMode control_mode = CTRL_OFF;
float cmd_rho_dot = 0.0;
float cmd_phi_dot = 0.0;

// Position moves
MoveType move_type = MOVE_NONE;
unsigned long move_start_ms = 0;
unsigned long move_timeout_ms = 0;
int settled_samples = 0;

// State machine
State state = ST_STARTUP;
unsigned long trial_start_ms = 0;
unsigned long state_start_ms = 0;
unsigned long first_move_ms = 0;
bool has_moved = false;
bool vision_alive = false;
float search_turned_rad = 0.0;
float search_ref_rad = 0.0;
float prev_search_phi = 0.0;

// First look
int look_found = 0;
int look_missed = 0;

// Beacon position
bool target_valid = false;
int target_frames = 0;
float target_x = 0.0;
float target_y = 0.0;

// Arrow
byte arrow_last = ARROW_UNKNOWN;
byte arrow_streak = 0;
byte arrow_choice = ARROW_UNKNOWN;
bool arrow_guessed = false;

// Frame tracking
byte last_seq = 0;
bool have_last_seq = false;

float wrapAngle(float angle);
float clampf(float v, float lo, float hi);
float rampTo(float current, float target, float step);
void encoder1ISR();
void encoder2ISR();
long MyEnc();
long MyEnc2();
long wheelCounts(int i);
void receiveEvent(int numBytes);
void requestEvent();
void readWheels(float dt);
void updateOdometry();
void recordPose(unsigned long now);
Pose poseAt(unsigned long now, unsigned long age_ms);
void holdPosition();
void motorsOff();
void markMoving(unsigned long now);
void startMove(MoveType type, float dthetaR, float dthetaL, unsigned long now);
void startTurn(float angle_rad, unsigned long now);
void startDrive(float dist_in, unsigned long now);
bool updateMoveStatus(unsigned long now);
void applySpeedLoop(int i, float speed_setpoint, float ff_volts);
void runPositionControl(float dt);
void runVelocityControl(float dt);
bool pollFrame(Frame &f);
void handleFrame(const Frame &f, unsigned long now);
void updateTarget(const Frame &f, unsigned long now);
float distanceLeft();
float headingError();
void readArrow(byte arrow);
void fakeFrame(unsigned long now);
void setState(State s, unsigned long now);
void handleCommands(unsigned long now);
void startTrial(unsigned long now);
void startSearch(unsigned long now);
void startApproach(unsigned long now);
void startFinalDrive(unsigned long now);
void arriveAtBeacon(unsigned long now);
void startArrowTurn(unsigned long now);
void finishTrial(unsigned long now);
void runStateMachine(unsigned long now, bool move_finished, float dt);

// angle wrapping helper, keeps angle between [-pi,+pi].
float wrapAngle(float angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// limits how fast a command speeds up, slowing down is not limited
float rampTo(float current, float target, float step) {
  if (target * current < 0.0) current = 0.0;
  if (fabs(target) <= fabs(current)) return target;
  return clampf(target, current - step, current + step);
}

// ISR
void encoder1ISR() {
  int thisA = digitalRead(Apin);
  int thisB = digitalRead(Bpin);
  if (thisA == thisB) {
    rotation_count += 2;
  } else {
    rotation_count -= 2;
  }
}

void encoder2ISR() {
  int thisC = digitalRead(Cpin);
  int thisD = digitalRead(Dpin);
  if (thisC == thisD) {
    rotation_count2 -= 2;
  } else {
    rotation_count2 += 2;
  }
}

// Encoder reader, fixes counts by using 1 interrupt for encoder
long MyEnc() {
  long count;
  int thisA = digitalRead(Apin);
  int thisB = digitalRead(Bpin);
  noInterrupts();
  count = rotation_count;
  interrupts();
  if (thisA != thisB) {
    count = count + 1;
  }
  return count;
}

// Encoder 2 reader, fixes counts by using 1 interrupt for encoder
long MyEnc2() {
  long count;
  int thisC = digitalRead(Cpin);
  int thisD = digitalRead(Dpin);
  noInterrupts();
  count = rotation_count2;
  interrupts();
  if (thisC != thisD) {
    count = count + 1;
  }
  return count;
}

// Receive I2C data from Raspberry Pi, kept short since it runs in an interrupt
void receiveEvent(int numBytes) {
  if (numBytes < 1) return;
  byte cmd = Wire.read();

  if (cmd == CMD_MEASUREMENT && numBytes == 1 + MEAS_PAYLOAD_LEN) {
    for (byte i = 0; i < MEAS_PAYLOAD_LEN; i++) {
      rx_meas_buf[i] = Wire.read();
    }
    rx_meas_ready = true;
  } else if (cmd == CMD_STOP) {
    rx_stop = true;
  }

  // Clear any extra bytes
  while (Wire.available() > 0) {
    Wire.read();
  }
}

// Send state and arrow to Raspberry Pi
void requestEvent() {
  byte status[2] = {tx_state, tx_arrow};
  Wire.write(status, 2);
}

// encoder counts with forward as positive
long wheelCounts(int i) {
  return ENCODER_DIR[i] * (i == 0 ? MyEnc() : MyEnc2());
}

// Read encoder positions and speeds
void readWheels(float dt) {
  long pos1_counts = wheelCounts(0);
  long pos2_counts = wheelCounts(1);

  // Convert counts to wheel angles
  actual_pos[0] = 2.0 * PI * (float)pos1_counts / COUNTS_PER_REV;
  actual_pos[1] = 2.0 * PI * (float)pos2_counts / COUNTS_PER_REV;

  // Calculate wheel speeds using actual elapsed time
  actual_speed[0] = 2.0 * PI * (float)(pos1_counts - prev_pos1_counts) / (COUNTS_PER_REV * dt);
  actual_speed[1] = 2.0 * PI * (float)(pos2_counts - prev_pos2_counts) / (COUNTS_PER_REV * dt);

  prev_pos1_counts = pos1_counts;
  prev_pos2_counts = pos2_counts;
}

// Uses changes in wheel angles to determine and estimate robot position, i.e. odometry
void updateOdometry() {
  float thetaR = actual_pos[RIGHT_IDX];
  float thetaL = actual_pos[LEFT_IDX];

  // initialize history
  if (!odom_initialized) {
    prev_odom_right = thetaR;
    prev_odom_left = thetaL;
    odom_initialized = true;
    return;
  }

  // incremental wheel travel in inches
  float dsR = R_RIGHT_IN * (thetaR - prev_odom_right);
  float dsL = R_LEFT_IN * (thetaL - prev_odom_left);
  prev_odom_right = thetaR;
  prev_odom_left = thetaL;

  // robot forward displacement and heading change
  float ds = (dsR + dsL) / 2.0;
  float dphi = (dsR - dsL) / BASE_EFF_IN;

  // midpoint integration for x-y position
  robot_x += ds * cos(robot_phi + dphi / 2.0);
  robot_y += ds * sin(robot_phi + dphi / 2.0);
  robot_phi = wrapAngle(robot_phi + dphi);
}

// saves robot position every 20 ms
void recordPose(unsigned long now) {
  if (pose_count > 0 && now - last_pose_ms < POSE_STEP_MS) return;
  last_pose_ms = now;
  pose_head = (pose_head + 1) % POSE_HISTORY_LEN;
  pose_hist[pose_head].x = robot_x;
  pose_hist[pose_head].y = robot_y;
  pose_hist[pose_head].phi = robot_phi;
  if (pose_count < POSE_HISTORY_LEN) pose_count++;
}

// robot position from age_ms ago
Pose poseAt(unsigned long now, unsigned long age_ms) {
  Pose p = {robot_x, robot_y, robot_phi};
  if (pose_count == 0) return p;
  unsigned long since_last = now - last_pose_ms;
  unsigned long back = (age_ms > since_last) ? (age_ms - since_last + POSE_STEP_MS / 2) / POSE_STEP_MS : 0;
  if (back >= pose_count) back = pose_count - 1;
  byte idx = (pose_head + POSE_HISTORY_LEN - (byte)back) % POSE_HISTORY_LEN;
  return pose_hist[idx];
}

// holds the wheels where they are
void holdPosition() {
  for (int i = 0; i < 2; i++) {
    desired_pos[i] = actual_pos[i];
    integral_error[i] = 0.0;
    desired_speed[i] = 0.0;
  }
  move_type = MOVE_NONE;
  settled_samples = 0;
  control_mode = CTRL_POSITION;
}

void motorsOff() {
  analogWrite(pwmPin[0], 0);
  analogWrite(pwmPin[1], 0);
  desired_speed[0] = 0.0;
  desired_speed[1] = 0.0;
}

// marks when the timer starts
void markMoving(unsigned long now) {
  if (!has_moved) {
    has_moved = true;
    first_move_ms = now;
  }
}

// sets new wheel targets from the current wheel positions
void startMove(MoveType type, float dthetaR, float dthetaL, unsigned long now) {
  desired_pos[RIGHT_IDX] = actual_pos[RIGHT_IDX] + dthetaR;
  desired_pos[LEFT_IDX] = actual_pos[LEFT_IDX] + dthetaL;

  // reset integral for each move
  integral_error[0] = 0.0;
  integral_error[1] = 0.0;

  control_mode = CTRL_POSITION;
  move_type = type;
  move_start_ms = now;
  settled_samples = 0;
  markMoving(now);

  // timeout in case a wheel never settles
  float max_speed = (type == MOVE_TURN) ? TURN_MAX_WHEEL_SPEED : DRIVE_MAX_WHEEL_SPEED;
  float largest = fabs(dthetaR) > fabs(dthetaL) ? fabs(dthetaR) : fabs(dthetaL);
  move_timeout_ms = 1500UL + (unsigned long)(2000.0 * largest / max_speed);
}

// Wheel-angle changes for an in-place turn, positive = left
void startTurn(float angle_rad, unsigned long now) {
  float dthetaR = TURN_TRIM * (WHEEL_BASE_IN * angle_rad) / (2.0 * WHEEL_RADIUS_IN);
  startMove(MOVE_TURN, dthetaR, -dthetaR, now);
}

// Wheel-angle changes for a straight drive
void startDrive(float dist_in, unsigned long now) {
  startMove(MOVE_DRIVE,
            RIGHT_TRIM * dist_in / WHEEL_RADIUS_IN,
            LEFT_TRIM * dist_in / WHEEL_RADIUS_IN,
            now);
}

// Check whether both wheels have reached their targets
bool updateMoveStatus(unsigned long now) {
  if (control_mode != CTRL_POSITION || move_type == MOVE_NONE) return false;

  bool position_ok =
      fabs(desired_pos[RIGHT_IDX] - actual_pos[RIGHT_IDX]) < POS_TOL_RAD &&
      fabs(desired_pos[LEFT_IDX] - actual_pos[LEFT_IDX]) < POS_TOL_RAD;
  bool speed_ok =
      fabs(actual_speed[RIGHT_IDX]) < SPEED_TOL_RAD_S &&
      fabs(actual_speed[LEFT_IDX]) < SPEED_TOL_RAD_S;

  if (position_ok && speed_ok) {
    settled_samples++;
  } else {
    settled_samples = 0;
  }

  unsigned long elapsed = now - move_start_ms;
  bool timed_out = (elapsed > move_timeout_ms && speed_ok) || elapsed > 2 * move_timeout_ms;

  // Require stable readings over multiple control samples
  if (settled_samples >= SETTLE_SAMPLES || timed_out) {
    if (settled_samples < SETTLE_SAMPLES) {
      Serial.println(F("Move timed out, continuing"));
    }
    move_type = MOVE_NONE;
    settled_samples = 0;
    return true;
  }
  return false;
}

// speed controller, sets motor voltage and direction
void applySpeedLoop(int i, float speed_setpoint, float ff_volts) {
  error[i] = speed_setpoint - actual_speed[i];
  Voltage[i] = clampf(Kp * error[i] + ff_volts, -Battery_Voltage, Battery_Voltage);

  if (Voltage[i] * MOTOR_DIR[i] >= 0.0) {
    digitalWrite(signPin[i], HIGH);
  } else {
    digitalWrite(signPin[i], LOW);
  }

  PWM[i] = (unsigned int)(255.0 * fabs(Voltage[i]) / Battery_Voltage);
  if (PWM[i] > 255) PWM[i] = 255;
  analogWrite(pwmPin[i], PWM[i]);
}

// mini project existing nested wheel controllers
void runPositionControl(float dt) {
  float max_speed = (move_type == MOVE_DRIVE) ? DRIVE_MAX_WHEEL_SPEED : TURN_MAX_WHEEL_SPEED;
  float accel_step = MAX_WHEEL_ACCEL * dt;

  // keeps both wheels moving together
  float eR = desired_pos[RIGHT_IDX] - actual_pos[RIGHT_IDX];
  float eL = desired_pos[LEFT_IDX] - actual_pos[LEFT_IDX];
  float sync_adj[2] = {0.0, 0.0};
  if (move_type == MOVE_DRIVE) {
    float c = K_SYNC * (eR / RIGHT_TRIM - eL / LEFT_TRIM);
    sync_adj[RIGHT_IDX] = c;
    sync_adj[LEFT_IDX] = -c;
  } else if (move_type == MOVE_TURN) {
    float c = K_SYNC * (eR + eL);
    sync_adj[RIGHT_IDX] = c;
    sync_adj[LEFT_IDX] = c;
  }

  for (int i = 0; i < 2; i++) {
    pos_error[i] = desired_pos[i] - actual_pos[i];

    // Position-loop anti-windup
    integral_error[i] += pos_error[i] * dt;
    integral_error[i] = clampf(integral_error[i], -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float cmd = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];
    cmd = clampf(cmd, -max_speed, max_speed);
    desired_speed[i] = rampTo(desired_speed[i], cmd, accel_step);

    applySpeedLoop(i, desired_speed[i] + sync_adj[i], 0.0);
  }
}

// steering control, forward speed and turn rate to wheel speeds
void runVelocityControl(float dt) {
  (void)dt;
  float half_base = BASE_EFF_IN / 2.0;
  float wR = (cmd_rho_dot + cmd_phi_dot * half_base) / R_RIGHT_IN;
  float wL = (cmd_rho_dot - cmd_phi_dot * half_base) / R_LEFT_IN;

  // scale both wheels down if one is over the limit
  float biggest = fabs(wR) > fabs(wL) ? fabs(wR) : fabs(wL);
  if (biggest > DRIVE_MAX_WHEEL_SPEED) {
    float k = DRIVE_MAX_WHEEL_SPEED / biggest;
    wR *= k;
    wL *= k;
  }

  desired_speed[RIGHT_IDX] = wR;
  desired_speed[LEFT_IDX] = wL;
  applySpeedLoop(RIGHT_IDX, wR, SPEED_FF_V_PER_RAD_S * wR);
  applySpeedLoop(LEFT_IDX, wL, SPEED_FF_V_PER_RAD_S * wL);
}

// Reads a new CV packet from the I2C buffer
bool pollFrame(Frame &f) {
  if (!rx_meas_ready) return false;

  byte b[MEAS_PAYLOAD_LEN];
  noInterrupts();
  for (byte i = 0; i < MEAS_PAYLOAD_LEN; i++) {
    b[i] = rx_meas_buf[i];
  }
  rx_meas_ready = false;
  interrupts();

  // skip repeated frames
  if (have_last_seq && b[1] == last_seq) return false;
  have_last_seq = true;
  last_seq = b[1];

  int16_t angle_cdeg = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
  uint16_t range_mm = (uint16_t)b[4] | ((uint16_t)b[5] << 8);

  f.found = (b[0] & 0x01) && range_mm > 0;
  f.arrow = (b[0] >> 1) & 0x03;
  if (f.arrow > ARROW_RIGHT) f.arrow = ARROW_UNKNOWN;
  f.angle_deg = angle_cdeg / 100.0;
  f.range_in = range_mm / MM_PER_IN;
  return true;
}

// uses a CV frame for the arrow and beacon position
void handleFrame(const Frame &f, unsigned long now) {
  bool arrow_window =
      (state == ST_APPROACH && distanceLeft() <= ARROW_READ_DIST_IN) ||
      state == ST_FINAL_DRIVE || state == ST_AT_BEACON;
  if (arrow_window) readArrow(f.arrow);

  switch (state) {
    case ST_FIRST_LOOK:
      if (now - state_start_ms < FIRST_LOOK_FLUSH_MS) return;
      if (f.found) {
        look_found++;
        updateTarget(f, now);
      } else {
        look_missed++;
      }
      break;

    case ST_SEARCH:
    case ST_APPROACH:
      if (f.found) updateTarget(f, now);
      break;

    default:
      break;
  }
}

// beacon position from the camera, using where the robot was when the frame was taken
void updateTarget(const Frame &f, unsigned long now) {
  if (f.range_in > MAX_TARGET_DIST_IN) return;

  Pose p = poseAt(now, VISION_LATENCY_MS);
  float c = cos(p.phi);
  float s = sin(p.phi);
  float cam_x = p.x + CAMERA_X_IN * c - CAMERA_Y_IN * s;
  float cam_y = p.y + CAMERA_X_IN * s + CAMERA_Y_IN * c;
  float ray = p.phi + f.angle_deg * DEG2RAD;
  float mx = cam_x + f.range_in * cos(ray);
  float my = cam_y + f.range_in * sin(ray);

  // average the new reading into the beacon position
  target_frames++;
  float alpha = 1.0 / target_frames;
  if (alpha < TARGET_FILTER_ALPHA) alpha = TARGET_FILTER_ALPHA;
  if (!target_valid) {
    target_x = mx;
    target_y = my;
    target_valid = true;
  } else {
    target_x += alpha * (mx - target_x);
    target_y += alpha * (my - target_y);
  }
}

// distance left to the stop point
float distanceLeft() {
  float dx = target_x - robot_x;
  float dy = target_y - robot_y;
  return sqrt(dx * dx + dy * dy) - STANDOFF_IN;
}

// angle from robot heading to the beacon, positive = left
float headingError() {
  return wrapAngle(atan2(target_y - robot_y, target_x - robot_x) - robot_phi);
}

// makes a fake CV frame for FAKE_BEACON_MODE
unsigned long last_fake_ms = 0;
void fakeFrame(unsigned long now) {
  if (now - last_fake_ms < FAKE_FRAME_MS) return;
  last_fake_ms = now;

  float bx = FAKE_BEACON_DIST_IN * cos(FAKE_BEACON_BEARING_DEG * DEG2RAD);
  float by = FAKE_BEACON_DIST_IN * sin(FAKE_BEACON_BEARING_DEG * DEG2RAD);

  Pose p = poseAt(now, VISION_LATENCY_MS);
  float c = cos(p.phi);
  float s = sin(p.phi);
  float cam_x = p.x + CAMERA_X_IN * c - CAMERA_Y_IN * s;
  float cam_y = p.y + CAMERA_X_IN * s + CAMERA_Y_IN * c;
  float rel = wrapAngle(atan2(by - cam_y, bx - cam_x) - p.phi);
  float range = sqrt((bx - cam_x) * (bx - cam_x) + (by - cam_y) * (by - cam_y));

  Frame f;
  f.found = fabs(rel) < FAKE_HALF_FOV_DEG * DEG2RAD;
  f.angle_deg = rel / DEG2RAD;
  f.range_in = range;
  f.arrow = (f.found && range < FAKE_ARROW_RANGE_IN) ? FAKE_ARROW : ARROW_UNKNOWN;

  vision_alive = true;
  if (state != ST_STARTUP) handleFrame(f, now);
}

// picks the arrow after enough matching readings in a row
void readArrow(byte arrow) {
  if (arrow != ARROW_LEFT && arrow != ARROW_RIGHT) return;

  if (arrow == arrow_last) {
    if (arrow_streak < 255) arrow_streak++;
  } else {
    arrow_last = arrow;
    arrow_streak = 1;
  }

  if (arrow_streak >= ARROW_READINGS_NEEDED) {
    arrow_choice = arrow;
  }
}

void setState(State s, unsigned long now) {
  state = s;
  state_start_ms = now;
  tx_state = (byte)s;
  Serial.print(F("t="));
  Serial.print((now - trial_start_ms) / 1000.0, 2);
  Serial.print(F(" s  state "));
  Serial.println((int)s);
}

// stop command from the Pi, and serial input for arrow test mode
void handleCommands(unsigned long now) {
  noInterrupts();
  bool stop = rx_stop;
  rx_stop = false;
  interrupts();

  if (stop && state != ST_DONE) {
    Serial.println(F("ABORT from Pi"));
    motorsOff();
    control_mode = CTRL_OFF;
    setState(ST_DONE, now);
  }

#if ARROW_TEST_MODE
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (state != ST_AT_BEACON) continue;
    if (c == 'L' || c == 'l') {
      Serial.println(F("Test input: LEFT"));
      arrow_choice = ARROW_LEFT;
    } else if (c == 'R' || c == 'r') {
      Serial.println(F("Test input: RIGHT"));
      arrow_choice = ARROW_RIGHT;
    } else if (c == 'U' || c == 'u') {
      Serial.println(F("Test input: NO ARROW"));
      startArrowTurn(now);
    }
  }
#endif
}

// resets everything at the start of a trial
void startTrial(unsigned long now) {
  robot_x = 0.0;
  robot_y = 0.0;
  robot_phi = 0.0;
  odom_initialized = false;
  pose_count = 0;
  target_valid = false;
  target_frames = 0;
  arrow_last = ARROW_UNKNOWN;
  arrow_streak = 0;
  arrow_choice = ARROW_UNKNOWN;
  arrow_guessed = false;
  look_found = 0;
  look_missed = 0;
  holdPosition();

  Serial.println(vision_alive ? F("Pi frames arriving, starting")
                              : F("No frames from the Pi, starting anyway"));
#if ARROW_TEST_MODE
  Serial.println(F("ARROW TEST: show an arrow to the camera or type L / R / U"));
  setState(ST_AT_BEACON, now);
#else
  setState(ST_FIRST_LOOK, now);
#endif
}

void startSearch(unsigned long now) {
  Serial.println(F("Beacon not in view, searching"));
  control_mode = CTRL_VELOCITY;
  cmd_rho_dot = 0.0;
  cmd_phi_dot = 0.0;
  search_turned_rad = 0.0;
  search_ref_rad = 0.0;
  prev_search_phi = robot_phi;
  markMoving(now);
  setState(ST_SEARCH, now);
}

void startApproach(unsigned long now) {
  Serial.print(F("Beacon found: heading error "));
  Serial.print(headingError() / DEG2RAD, 1);
  Serial.print(F(" deg, distance to stop point "));
  Serial.print(distanceLeft(), 1);
  Serial.println(F(" in"));
  // keep the current turn rate if coming from the search
  if (control_mode != CTRL_VELOCITY) {
    cmd_rho_dot = 0.0;
    cmd_phi_dot = 0.0;
  }
  control_mode = CTRL_VELOCITY;
  markMoving(now);
  setState(ST_APPROACH, now);
}

// last few inches on the position controller
void startFinalDrive(unsigned long now) {
  float d = distanceLeft();
  if (d < 0.0) d = 0.0;
  Serial.print(F("Final drive "));
  Serial.print(d, 2);
  Serial.println(F(" in"));
  startDrive(d, now);
  setState(ST_FINAL_DRIVE, now);
}

void arriveAtBeacon(unsigned long now) {
  Serial.print(F("At beacon, odometry distance to marker "));
  Serial.print(distanceLeft() + STANDOFF_IN, 1);
  Serial.println(F(" in"));
  if (!TURN_AT_BEACON) {
    finishTrial(now);
  } else if (arrow_choice != ARROW_UNKNOWN) {
    startArrowTurn(now);
  } else {
    holdPosition();
    setState(ST_AT_BEACON, now);
  }
}

void startArrowTurn(unsigned long now) {
  if (arrow_choice == ARROW_UNKNOWN) {
    arrow_choice = ARROW_FALLBACK;
    arrow_guessed = true;
  }
  tx_arrow = arrow_choice;

  Serial.print(arrow_guessed ? F("NO ARROW REPORTED, fallback: ") : F("Arrow reported: "));
  Serial.println(arrow_choice == ARROW_LEFT ? F("turn LEFT") : F("turn RIGHT"));

  float turn = ARROW_TURN_DEG * DEG2RAD;
  startTurn(arrow_choice == ARROW_LEFT ? turn : -turn, now);
  setState(ST_TURN_90, now);
}

// stop after the done with motion
void finishTrial(unsigned long now) {
  motorsOff();
  control_mode = CTRL_OFF;
  setState(ST_DONE, now);
  Serial.print(F("DONE. Time since first motion: "));
  Serial.print(has_moved ? (now - first_move_ms) / 1000.0 : 0.0, 2);
  Serial.println(F(" s"));
  Serial.print(F("Search rotation (deg): "));
  Serial.println(fabs(search_turned_rad) / DEG2RAD, 0);
  Serial.print(F("Odometry x, y, phi(deg): "));
  Serial.print(robot_x, 2);
  Serial.print(F(", "));
  Serial.print(robot_y, 2);
  Serial.print(F(", "));
  Serial.println(robot_phi / DEG2RAD, 1);
}

// advances through the states in state machine
void runStateMachine(unsigned long now, bool move_finished, float dt) {
  switch (state) {
    case ST_STARTUP:
      // wait for the Pi to start sending frames
      if (vision_alive || now - trial_start_ms >= STARTUP_WAIT_MS) {
        startTrial(now);
      }
      break;

    case ST_FIRST_LOOK: {
      unsigned long elapsed = now - state_start_ms;
      if (look_found >= FIRST_FOUND_FRAMES ||
          (elapsed >= FIRST_LOOK_FLUSH_MS + FIRST_LOOK_TIMEOUT_MS && look_found > 0)) {
        startApproach(now);
      } else if (look_missed >= FIRST_NOT_FOUND_LIMIT ||
                 elapsed >= FIRST_LOOK_FLUSH_MS + FIRST_LOOK_TIMEOUT_MS) {
        startSearch(now);
      }
      break;
    }

    case ST_SEARCH: {
      // turn at a steady rate until the beacon is seen
      search_turned_rad += wrapAngle(robot_phi - prev_search_phi);
      prev_search_phi = robot_phi;
      if (target_valid) {
        startApproach(now);
        break;
      }
      float rate = SEARCH_DIR * SEARCH_RATE_DEG_S * DEG2RAD;
      float ref_rate = rampTo(cmd_phi_dot, rate, TURN_ACCEL_RAD_S2 * dt);
      search_ref_rad += ref_rate * dt;
      float lag = clampf(search_ref_rad - search_turned_rad, -0.3, 0.3);
      float max_rate = MAX_TURN_RATE_DEG_S * DEG2RAD;
      cmd_phi_dot = clampf(ref_rate + K_HEADING * lag, -max_rate, max_rate);
      cmd_rho_dot = 0.0;
      break;
    }

    case ST_APPROACH: {
      float d_left = distanceLeft();
      float h_err = headingError();

      // close and lined up, switch to the position controller
      if (d_left <= FINAL_DRIVE_IN && fabs(h_err) <= FINAL_HEADING_TOL_DEG * DEG2RAD) {
        startFinalDrive(now);
        break;
      }

      // turn rate from heading error
      float max_rate = MAX_TURN_RATE_DEG_S * DEG2RAD;
      float phi_dot = clampf(K_HEADING * h_err, -max_rate, max_rate);

      // forward speed from distance left
      float gate = 1.0 - fabs(h_err) / (HEADING_GATE_DEG * DEG2RAD);
      gate = clampf(gate, 0.0, 1.0);
      float rho_dot = clampf(K_DISTANCE * d_left, 0.0, DRIVE_SPEED_IN_S) * gate;

      cmd_phi_dot = rampTo(cmd_phi_dot, phi_dot, TURN_ACCEL_RAD_S2 * dt);
      cmd_rho_dot = rampTo(cmd_rho_dot, rho_dot, FORWARD_ACCEL_IN_S2 * dt);
      break;
    }

    case ST_FINAL_DRIVE:
      if (move_finished) arriveAtBeacon(now);
      break;

    case ST_AT_BEACON:
#if ARROW_TEST_MODE
      if (arrow_choice != ARROW_UNKNOWN) startArrowTurn(now);
#else
      if (arrow_choice != ARROW_UNKNOWN ||
          now - state_start_ms >= ARROW_LOOK_TIMEOUT_MS) {
        startArrowTurn(now);
      }
#endif
      break;

    case ST_TURN_90:
      if (move_finished) finishTrial(now);
      break;

    case ST_DONE:
      break;
  }
}

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(signPin[0], OUTPUT);
  pinMode(signPin[1], OUTPUT);
  pinMode(pwmPin[0], OUTPUT);
  pinMode(pwmPin[1], OUTPUT);

  digitalWrite(enablePin, HIGH);
  digitalWrite(signPin[0], HIGH);
  digitalWrite(signPin[1], HIGH);
  analogWrite(pwmPin[0], 0);
  analogWrite(pwmPin[1], 0);

  // establish 115200 baud rate
  Serial.begin(115200);

  // establish I2C communication with Raspberry Pi
  Wire.begin(ARD_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  pinMode(Apin, INPUT_PULLUP);
  pinMode(Bpin, INPUT_PULLUP);
  pinMode(Cpin, INPUT_PULLUP);
  pinMode(Dpin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Apin), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Cpin), encoder2ISR, CHANGE);

  // Establish initial speed measurement baseline
  prev_pos1_counts = wheelCounts(0);
  prev_pos2_counts = wheelCounts(1);

  last_time_ms = millis();
  trial_start_ms = last_time_ms;
  state_start_ms = last_time_ms;
  tx_state = ST_STARTUP;

  Serial.print(F("Demo 2: "));
  Serial.print(TURN_AT_BEACON ? F("stop + arrow turn") : F("stop at beacon"));
  Serial.print(F(", search "));
  Serial.println(SEARCH_DIR > 0 ? F("CCW (left)") : F("CW (right)"));
#if ARROW_TEST_MODE
  Serial.println(F("*** ARROW TEST MODE: no search or drive ***"));
#endif
#if FAKE_BEACON_MODE
  Serial.print(F("*** FAKE BEACON MODE: virtual beacon "));
  Serial.print(FAKE_BEACON_DIST_IN, 1);
  Serial.print(F(" in at "));
  Serial.print(FAKE_BEACON_BEARING_DEG, 1);
  Serial.println(F(" deg ***"));
#endif
}

void loop() {
  unsigned long now = millis();

  // 10 ms control period
  if (now - last_time_ms < SAMPLE_MS) {
    return;
  }
  float dt = (now - last_time_ms) / 1000.0;
  last_time_ms = now;

  // Read encoder positions and estimate robot pose
  readWheels(dt);
  updateOdometry();
  recordPose(now);
  handleCommands(now);

#if FAKE_BEACON_MODE
  fakeFrame(now);
  rx_meas_ready = false;
#else
  Frame f;
  if (pollFrame(f)) {
    vision_alive = true;
    handleFrame(f, now);
  }
#endif

  // Update the navigation command
  bool move_finished = updateMoveStatus(now);
  runStateMachine(now, move_finished, dt);

  if (control_mode == CTRL_VELOCITY) {
    runVelocityControl(dt);
  } else if (control_mode == CTRL_POSITION) {
    runPositionControl(dt);
  } else {
    motorsOff();
  }
}

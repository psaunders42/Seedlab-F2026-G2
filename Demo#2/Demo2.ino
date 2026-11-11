// ===================================================
// Name: Tomas Padilla & Travis C. Plumb
// Class: EENG350 F26 Team 2 - Controls
// Date: 11/11/2026
// Purpose: Finds the ArUco beacon using CV data sent from the Pi over I2C, drives to within 1.5 ft
// of it, and optionally turns 90 degrees toward the arrow near the marker. Uses a steering
// controller for the search and approach, and the Demo 1 nested PI position controller
// for the final stop and the 90 degree turn.
//
// How a run works (press the Arduino reset button to start):
//   1. STARTUP       motors off, wait for the Pi to start sending frames
//   2. FIRST LOOK    stand still and check if the beacon is already in view
//   3. SEARCH        turn left at a steady speed until the camera sees the beacon
//   4. APPROACH      turn toward the beacon and drive at it without stopping
//   5. FINAL DRIVE   last few inches on the Demo 1 position controller so it stops accurately
//   6. AT BEACON     (only if TURN_AT_BEACON true) - wait for the arrow
//   7. TURN 90       (only if TURN_AT_BEACON true) - turn 90 degrees toward the arrow
//   8. DONE          motors off
//
// Two control modes:
//   Velocity mode (search, approach): Uses P control for forward speed and turn rate sent to wheel speed
//   Position mode (final drive, 90 deg turn): Uses Demo 1 wheel PI controller on final stretch and P control to turn 90 degrees
//
// Robot position: odometry tracks the midpoint between the wheels in inches (x forward, y left,
// phi counterclockwise) starting from where the robot was when the run started. The beacon's
// position is estimated in the same frame from each camera frame, and the distance and angle
// left to go are recalculated from odometry every 10 ms.
//
// I2C from Pi (demo2.py send_target): [0x03, angle (0.01 deg, int16), distance (0.1 in, uint16), arrow]
// angle positive = beacon left of center
// distance is from the camera, 0 = marker not found.
// !!!!!!!!! Arrow byte is a placeholder until CV adds it (1 = left, 2 = right). !!!!!!! untested until CV adds arrows!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// Settings most likely to need changing are marked PLACEHOLDER.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Demo trial setting
const bool TURN_AT_BEACON = false;  // true = turn according to the arrow after stopping

// Placeholder measurements, update once the robot is finished
constexpr float CAMERA_X_IN     = 6.0;   // PLACEHOLDER: camera distance in front of the axle (in) !!! measure and change accordingly
constexpr float CAMERA_Y_IN     = 0.0;   // PLACEHOLDER: camera offset left of center (in) !!! probably not necessary
constexpr float STANDOFF_IN     = 15.0;  // PLACEHOLDER: stop distance from the beacon (in), set to 1.25 ft to be within demo range
const float SEARCH_RATE_DEG_S   = 30.0;  // PLACEHOLDER: search rotation speed !!! should be okay.
constexpr unsigned long VISION_LATENCY_MS = 150;  // PLACEHOLDER: camera to I2C delay (ms)

// Robot measureables
constexpr float WHEEL_RADIUS_IN = 3.0;   // wheel radius (in)
constexpr float WHEEL_BASE_IN   = 14.0;  // distance between the wheels (in)

// Trim adjustments for uneven wheel movement
constexpr float RIGHT_TRIM = 1.023;  // raise to make the right wheel go farther on a drive
constexpr float LEFT_TRIM  = 1.013;  // raise to make the left wheel go farther on a drive
constexpr float TURN_TRIM  = 0.995;  // raise if 90 deg turns come up short

// Wheel radius and wheel base with trims applied (used by odometry and velocity mode)
constexpr float R_RIGHT_IN = WHEEL_RADIUS_IN / RIGHT_TRIM;
constexpr float R_LEFT_IN  = WHEEL_RADIUS_IN / LEFT_TRIM;
constexpr float BASE_EFF_IN = (R_RIGHT_IN + R_LEFT_IN) * TURN_TRIM * WHEEL_BASE_IN / (2.0 * WHEEL_RADIUS_IN);

// Drive and turning speeds/rates variables
const float DRIVE_SPEED_IN_S   = 8.0;      // top forward speed while approaching (in/s)
const float MAX_TURN_RATE_DEG_S = 60.0;    // top turn rate while lining up (deg/s)
const float K_HEADING          = 3.0;      // turn rate per radian of heading error
const float K_DISTANCE         = 2.0;      // forward speed per inch left, slows down near the stop
const float HEADING_GATE_DEG   = 20.0;     // no forward speed until heading error is under this
const float FORWARD_ACCEL_IN_S2 = 15.0;    // how fast forward speed can increase (lower if wheels slip)
const float TURN_ACCEL_RAD_S2   = 4.0;     // how fast turn rate can increase
const float TARGET_FILTER_ALPHA = 0.3;     // weight of each new camera frame in the beacon position
const float MAX_TARGET_DIST_IN  = 100.0;   // ignore camera distances farther than this

// Final drive using the position controller
const float FINAL_DRIVE_IN        = 4.0;   // switch to position control this far from the stop
const float FINAL_HEADING_TOL_DEG = 4.0;   // only switch once lined up within this angle

// Vision Pi timing
const unsigned long STARTUP_WAIT_MS = 15000; // max wait for the Pi's first frame
const unsigned long FIRST_LOOK_FLUSH_MS = 250; // ignore frames this soon after the first look starts
const int FIRST_FOUND_FRAMES = 4; // frames that must see the beacon on the first look
const int FIRST_NOT_FOUND_LIMIT = 8; // empty frames before giving up and searching
const unsigned long FIRST_LOOK_TIMEOUT_MS = 3000; // max length of the first look

// Accounts for camera delay while robot is moving, keeps where the robot was over the last ~0.5 s
constexpr unsigned long POSE_STEP_MS = 20;
constexpr byte POSE_HISTORY_LEN = 24;
constexpr unsigned long POSE_HISTORY_MS = POSE_STEP_MS * (POSE_HISTORY_LEN - 1);

// Arrow (PLACEHOLDER: CV does not send the arrow yet, robot uses ARROW_FALLBACK until it does) !!!!!!!
const float         ARROW_TURN_DEG        = 90.0;  // size of the final turn
const float         ARROW_READ_DIST_IN    = 12.0;  // read the arrow within this distance of the stop
const byte          ARROW_READINGS_NEEDED = 2;     // matching readings in a row before trusting it
const unsigned long ARROW_LOOK_TIMEOUT_MS = 1500;  // max wait at the beacon before using the fallback

// Motor control pins
const int enablePin = 4;
const int signPin[2] = {7, 8};
const int pwmPin[2] = {9, 10};

// Encoder pins
const int Apin = 2;  // m1 interrupt
const int Bpin = 5;  // m1 digital
const int Cpin = 3;  // m2 interrupt
const int Dpin = 6;  // m2 digital

// array mapping, robot drives camera first
// index 0 = motor on pins 7/9 and encoder pins 2/5, index 1 = motor on pins 8/10 and encoder pins 3/6
const int RIGHT_IDX = 1; // which index is the right wheel
const int LEFT_IDX = 0; // which index is the left wheel
const int MOTOR_DIR[2] = {-1, -1}; // flips the sign pin so positive voltage drives forward
const int ENCODER_DIR[2] = {-1, -1}; // flips the counts so forward is positive

// Control variables
const unsigned long SAMPLE_MS = 10; // control loop period (ms)
const float COUNTS_PER_REV = 3200.0; // encoder counts per wheel revolution
const float Kp_pos = 7; // position loop proportional gain (Demo 1)
const float Ki_pos = 0.2; // position loop integral gain (Demo 1)
const float Kp = 2; // speed loop proportional gain (Demo 1)
const float Battery_Voltage = 7.8;
const float INTEGRAL_LIMIT = 2.0; // anti-windup limit
const float TURN_MAX_WHEEL_SPEED = 1.5; // wheel speed limit for the 90 deg turn (rad/s)
const float DRIVE_MAX_WHEEL_SPEED = 3.0; // wheel speed limit for driving and steering (rad/s)
const float MAX_WHEEL_ACCEL = 12.0; // how fast wheel speed can increase in position mode
const float K_SYNC = 2.0; // keeps the wheels together in position mode
const float SPEED_FF_V_PER_RAD_S = 0.0; // PLACEHOLDER: 1/K from motor model, 0 = off

// completion conditions, a position move is done when both wheels are within these
const float POS_TOL_RAD = 0.04;
const float SPEED_TOL_RAD_S = 0.25;
const int SETTLE_SAMPLES = 10; // for this many samples in a row

const float DEG2RAD = PI / 180.0;

// I2C communication
const byte ARD_ADDR = 8; // Arduino address on the I2C bus
const byte CMD_TARGET = 0x03; // command byte demo2.py sends with each target

// arrow codes sent by CV
const byte ARROW_UNKNOWN = 0;
const byte ARROW_LEFT = 1;
const byte ARROW_RIGHT = 2;

const byte ARROW_FALLBACK = ARROW_LEFT; // turn used if no arrow is seen

// Latest values from the Pi, written in receiveEvent and read in pollFrame
volatile int16_t received_angle = 0;
volatile uint16_t received_dist = 0;
volatile byte received_arrow = 0;
volatile bool new_target_received = false;

// navigation states for state machine (see the run steps at the top)
enum State : byte {
  ST_STARTUP = 0,  // waiting for the Pi
  ST_FIRST_LOOK = 1,  // standing still, is the beacon in view?
  ST_SEARCH = 2,  // turning to find the beacon
  ST_APPROACH = 3,  // steering toward and driving at the beacon
  ST_FINAL_DRIVE = 4,  // last few inches on the position controller
  ST_AT_BEACON = 5,  // stopped, waiting for the arrow
  ST_TURN_90 = 6,  // turning toward the arrow
  ST_DONE = 7   // finished, motors off
};

// which controller is driving the motors
enum ControlMode : byte { CTRL_OFF, CTRL_VELOCITY, CTRL_POSITION };

// type of position move, sets the speed limit and wheel sync
enum MoveType : byte { MOVE_NONE, MOVE_TURN, MOVE_DRIVE };

enum LookResult : byte { LOOK_PENDING, LOOK_FOUND, LOOK_MISSED };

// one camera frame from the Pi
struct Frame {
  bool found;       // marker seen
  byte arrow;       // ARROW_UNKNOWN, ARROW_LEFT, or ARROW_RIGHT
  float angle_deg;  // angle to the marker, positive = left
  float range_in;   // distance from the camera to the marker (in)
};

// robot position and heading
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

// Wheel control (index 0 and 1, see array mapping)
float desired_speed[2] = {0, 0};   // rad/s
float actual_speed[2] = {0, 0};    // rad/s
float desired_pos[2] = {0, 0};     // rad
float actual_pos[2] = {0, 0};      // rad
float integral_error[2] = {0, 0};
float pos_error[2] = {0, 0};
float error[2] = {0, 0};
float Voltage[2] = {0, 0};
unsigned int PWM[2] = {0, 0};

// Robot bearings from odometry
float robot_x = 0.0;
float robot_y = 0.0;
float robot_phi = 0.0;
float prev_odom_right = 0.0;
float prev_odom_left = 0.0;
bool odom_initialized = false;

// Pose history
Pose pose_hist[POSE_HISTORY_LEN];
byte pose_head = 0;                // newest entry
byte pose_count = 0;
unsigned long last_pose_ms = 0;

// Steering commands for velocity mode
ControlMode control_mode = CTRL_OFF;
float cmd_rho_dot = 0.0;           // forward speed (in/s)
float cmd_phi_dot = 0.0;           // turn rate (rad/s), positive = left

// Position moves
MoveType move_type = MOVE_NONE;
unsigned long move_start_ms = 0;
unsigned long move_timeout_ms = 0;
int settled_samples = 0;

// State machine
State state = ST_STARTUP;
unsigned long trial_start_ms = 0;  // when the Arduino started
unsigned long state_start_ms = 0;  // when the current state started
unsigned long first_move_ms = 0;   // when the robot first moved (timer start)
bool has_moved = false;
bool vision_alive = false;         // true once any frame has come from the Pi
float search_turned_rad = 0.0;     // how far the search has turned
float search_ref_rad = 0.0;        // how far the search should have turned by now
float prev_search_phi = 0.0;

// First look
int look_found = 0;
int look_missed = 0;

// Beacon position estimate (same frame as odometry)
bool target_valid = false;
int target_frames = 0;
float target_x = 0.0;
float target_y = 0.0;

// Arrow
byte arrow_last = ARROW_UNKNOWN;   // last left/right reading
byte arrow_streak = 0;             // matching readings in a row
byte arrow_choice = ARROW_UNKNOWN; // decided turn direction
bool arrow_guessed = false;        // true if the fallback was used

// ---------------- Function declarations ----------------

// Helpers
float wrapAngle(float angle);                          // keeps an angle between -pi and pi
float clampf(float v, float lo, float hi);             // limits a value to a range
float rampTo(float current, float target, float step); // limits how fast a command speeds up

// Encoders and I2C (interrupts)
void encoder1ISR();                                    // counts encoder 1 (pin 2)
void encoder2ISR();                                    // counts encoder 2 (pin 3)
long MyEnc();                                          // reads encoder 1 counts
long MyEnc2();                                         // reads encoder 2 counts
long wheelCounts(int i);                               // encoder counts for wheel i, forward positive
void receiveEvent(int numBytes);                       // stores a target packet from the Pi

// Odometry
void readWheels(float dt);                             // wheel angles and speeds from the encoders
void updateOdometry();                                 // updates robot x, y, and heading
void recordPose(unsigned long now);                    // saves the pose every 20 ms
Pose poseAt(unsigned long now, unsigned long age_ms);  // pose from age_ms ago (when a frame was taken)

// Motion and wheel control
void holdPosition();                                   // hold the wheels where they are
void motorsOff();                                      // cut motor power
void markMoving(unsigned long now);                    // records when the robot first moves
void startMove(MoveType type, float dthetaR, float dthetaL, unsigned long now); // new position move
void startTurn(float angle_rad, unsigned long now);    // in-place turn, positive = left
void startDrive(float dist_in, unsigned long now);     // straight drive in inches
bool updateMoveStatus(unsigned long now);              // true once the current position move is done
void applySpeedLoop(int i, float speed_setpoint, float ff_volts); // speed P loop, sets PWM and sign pin
void runPositionControl(float dt);                     // Demo 1 position PI for both wheels
void runVelocityControl(float dt);                     // steering: forward speed and turn rate to wheels

// Camera data
bool pollFrame(Frame &f);                              // gets the newest packet from the Pi
void handleFrame(const Frame &f, unsigned long now);   // uses a frame for the beacon and arrow
void updateTarget(const Frame &f, unsigned long now);  // updates the beacon position estimate
float distanceLeft();                                  // inches left to the stop point
float headingError();                                  // angle to the beacon, positive = left
void readArrow(byte arrow);                            // decides the arrow from matching readings

// State machine
void setState(State s, unsigned long now);             // changes state and prints it
void startTrial(unsigned long now);                    // resets everything, starts the first look
void startSearch(unsigned long now);                   // starts turning to find the beacon
void startApproach(unsigned long now);                 // starts steering toward the beacon
void startFinalDrive(unsigned long now);               // last few inches on position control
void arriveAtBeacon(unsigned long now);                // stopped: finish, or go to the arrow turn
void startArrowTurn(unsigned long now);                // 90 deg turn toward the arrow (or fallback)
void finishTrial(unsigned long now);                   // motors off, prints results
void runStateMachine(unsigned long now, bool move_finished, float dt); // runs the current state

// ---------------- Helpers -----------------

// angle wrapping helper, keeps angle between [-pi,+pi].
float wrapAngle(float angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

// limits v to between lo and hi
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// limits how fast a command speeds up, slowing down is not limited
float rampTo(float current, float target, float step) {
  if (target * current < 0.0) current = 0.0;  // changing direction, start from zero
  if (fabs(target) <= fabs(current)) return target;
  return clampf(target, current - step, current + step);
}

// ---------------- Encoders and I2C ----------------

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

// Receive I2C target from Raspberry Pi
// Runs as an interrupt, so it only stores the bytes. pollFrame() reads them in the loop.
void receiveEvent(int numBytes) {
  if (Wire.available() > 0) {
    byte cmd = Wire.read();

    // Only accept target packets (cmd + angle + distance, arrow optional)
    if (cmd == CMD_TARGET && numBytes >= 5) {
      byte a_lo = Wire.read();
      byte a_hi = Wire.read();
      byte d_lo = Wire.read();
      byte d_hi = Wire.read();
      received_angle = (int16_t)((uint16_t)a_lo | ((uint16_t)a_hi << 8));
      received_dist = (uint16_t)d_lo | ((uint16_t)d_hi << 8);
      received_arrow = (numBytes >= 6) ? Wire.read() : 0;  // ARROW PLACEHOLDER: 6th byte from CV
      new_target_received = true;
    }
  }

  // Clear any extra bytes
  while (Wire.available() > 0) {
    Wire.read();
  }
}

// encoder counts with forward as positive
long wheelCounts(int i) {
  return ENCODER_DIR[i] * (i == 0 ? MyEnc() : MyEnc2());
}

// ---------------- Odometry ----------------

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

// saves robot position every 20 ms in a ring buffer
void recordPose(unsigned long now) {
  if (pose_count > 0 && now - last_pose_ms < POSE_STEP_MS) return;
  last_pose_ms = now;
  pose_head = (pose_head + 1) % POSE_HISTORY_LEN;
  pose_hist[pose_head].x = robot_x;
  pose_hist[pose_head].y = robot_y;
  pose_hist[pose_head].phi = robot_phi;
  if (pose_count < POSE_HISTORY_LEN) pose_count++;
}

// robot position from age_ms ago, used because a camera frame is already old when it arrives
Pose poseAt(unsigned long now, unsigned long age_ms) {
  Pose p = {robot_x, robot_y, robot_phi};
  if (pose_count == 0) return p;
  unsigned long since_last = now - last_pose_ms;
  unsigned long back = (age_ms > since_last) ? (age_ms - since_last + POSE_STEP_MS / 2) / POSE_STEP_MS : 0;
  if (back >= pose_count) back = pose_count - 1;
  byte idx = (pose_head + POSE_HISTORY_LEN - (byte)back) % POSE_HISTORY_LEN;
  return pose_hist[idx];
}

// ---------------- Motion and wheel control ----------------

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

// cuts power to both motors
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

  // MOTOR_DIR flips the sign pin for how the motor is mounted
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

    // position PI gives the wheel speed setpoint
    float cmd = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];
    cmd = clampf(cmd, -max_speed, max_speed);
    desired_speed[i] = rampTo(desired_speed[i], cmd, accel_step);

    applySpeedLoop(i, desired_speed[i] + sync_adj[i], 0.0);
  }
}

// steering control, forward speed and turn rate to wheel speeds
void runVelocityControl(float dt) {
  (void)dt;
  // right wheel = (forward + turn * half base) / radius, left wheel = (forward - turn * half base) / radius
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

// ---------------- Camera data ----------------

// Check for new target from Raspberry Pi, converts it to degrees and inches
bool pollFrame(Frame &f) {
  if (!new_target_received) return false;

  noInterrupts();
  int16_t angle = received_angle;
  uint16_t dist = received_dist;
  byte arrow = received_arrow;
  new_target_received = false;
  interrupts();

  f.found = dist > 0;
  f.angle_deg = angle / 100.0;
  f.range_in = dist / 10.0;
  // ARROW PLACEHOLDER: stays unknown until CV sends the arrow byte
  f.arrow = (arrow == ARROW_LEFT || arrow == ARROW_RIGHT) ? arrow : ARROW_UNKNOWN;
  return true;
}

// uses a CV frame for the arrow and beacon position
void handleFrame(const Frame &f, unsigned long now) {
  // arrow is only read close to the beacon
  bool arrow_window =
      (state == ST_APPROACH && distanceLeft() <= ARROW_READ_DIST_IN) ||
      state == ST_FINAL_DRIVE || state == ST_AT_BEACON;
  if (arrow_window) readArrow(f.arrow);

  // beacon position is updated while looking, searching, and approaching
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
      break;  // final drive and later: beacon position is locked in
  }
}

// beacon position from the camera, using where the robot was when the frame was taken
void updateTarget(const Frame &f, unsigned long now) {
  if (f.range_in > MAX_TARGET_DIST_IN) return;

  // camera position at the time of the frame
  Pose p = poseAt(now, VISION_LATENCY_MS);
  float c = cos(p.phi);
  float s = sin(p.phi);
  float cam_x = p.x + CAMERA_X_IN * c - CAMERA_Y_IN * s;
  float cam_y = p.y + CAMERA_X_IN * s + CAMERA_Y_IN * c;

  // marker position = camera position + distance along the camera angle
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

// ---------------- State machine ----------------

// changes state and prints the time and state number
void setState(State s, unsigned long now) {
  state = s;
  state_start_ms = now;
  Serial.print(F("t="));
  Serial.print((now - trial_start_ms) / 1000.0, 2);
  Serial.print(F(" s  state "));
  Serial.println((int)s);
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
  setState(ST_FIRST_LOOK, now);
}

// beacon not in view, start turning to find it
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

// beacon found, start steering toward it
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

// last few inches on the position controller, the robot keeps rolling into it
void startFinalDrive(unsigned long now) {
  float d = distanceLeft();
  if (d < 0.0) d = 0.0;
  Serial.print(F("Final drive "));
  Serial.print(d, 2);
  Serial.println(F(" in"));
  startDrive(d, now);
  setState(ST_FINAL_DRIVE, now);
}

// stopped at the beacon: finish, turn now, or wait for the arrow
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

// 90 deg turn toward the arrow, uses ARROW_FALLBACK if no arrow was read
void startArrowTurn(unsigned long now) {
  if (arrow_choice == ARROW_UNKNOWN) {
    arrow_choice = ARROW_FALLBACK;
    arrow_guessed = true;
  }

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

// advances through the states in state machine, called every 10 ms
void runStateMachine(unsigned long now, bool move_finished, float dt) {
  switch (state) {
    case ST_STARTUP:
      // wait for the Pi to start sending frames
      if (vision_alive || now - trial_start_ms >= STARTUP_WAIT_MS) {
        startTrial(now);
      }
      break;

    case ST_FIRST_LOOK: {
      // drive at the beacon if it's in view, otherwise start searching
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
      // the reference angle moves at the search rate, the turn rate is corrected so the
      // robot keeps up with it (the speed P loop alone would turn slower than asked)
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

      // forward speed from distance left, reduced until the robot is facing the beacon
      float gate = 1.0 - fabs(h_err) / (HEADING_GATE_DEG * DEG2RAD);
      gate = clampf(gate, 0.0, 1.0);
      float rho_dot = clampf(K_DISTANCE * d_left, 0.0, DRIVE_SPEED_IN_S) * gate;

      cmd_phi_dot = rampTo(cmd_phi_dot, phi_dot, TURN_ACCEL_RAD_S2 * dt);
      cmd_rho_dot = rampTo(cmd_rho_dot, rho_dot, FORWARD_ACCEL_IN_S2 * dt);
      break;
    }

    case ST_FINAL_DRIVE:
      // wait for the position move to finish
      if (move_finished) arriveAtBeacon(now);
      break;

    case ST_AT_BEACON:
      // turn once the arrow is known, or use the fallback after the timeout
      if (arrow_choice != ARROW_UNKNOWN ||
          now - state_start_ms >= ARROW_LOOK_TIMEOUT_MS) {
        startArrowTurn(now);
      }
      break;

    case ST_TURN_90:
      // wait for the turn to finish
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

  Serial.print(F("Demo 2: "));
  Serial.print(TURN_AT_BEACON ? F("stop + arrow turn") : F("stop at beacon"));
  Serial.print(F(", search "));
  Serial.println(SEARCH_DIR > 0 ? F("CCW (left)") : F("CW (right)"));
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

  // Check for new target from Raspberry Pi
  Frame f;
  if (pollFrame(f)) {
    vision_alive = true;
    handleFrame(f, now);
  }

  // Update the navigation command
  bool move_finished = updateMoveStatus(now);
  runStateMachine(now, move_finished, dt);

  // Run whichever controller is active
  if (control_mode == CTRL_VELOCITY) {
    runVelocityControl(dt);
  } else if (control_mode == CTRL_POSITION) {
    runPositionControl(dt);
  } else {
    motorsOff();
  }
}

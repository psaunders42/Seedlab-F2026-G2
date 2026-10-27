// Name: Tomas Padilla, Travis C. Plumb
// Class: EENG350 F26 Team 2 - Controls
// Date: 10/04/2026
// Purpose: Implements a PI controller for two motors on the robot to spin the wheels to their correct
// locations given a input of a "quadrant region" from the CV scanner. Utilizes an anti-windup to prevent
// overshoot upon start of wheel motion.
#include <Arduino.h>
#include <math.h>

// Motor control pins
int enablePin = 4;
int signPin[2] = {7,8};
int pwmPin[2] = {9,10};

// Encoder pins
const int Apin = 2; // m1 interrupt
const int Bpin = 5; // m1 digital
const int Cpin = 3; // m2 interrupt
const int Dpin = 6; // m2 digital

// Encoder counts
volatile long rotation_count = 0;
volatile long rotation_count2 = 0;

// Timing variables
unsigned long desired_Ts_ms = 10; // sample time in ms
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
float Ts = desired_Ts_ms / 1000.0; // sample period

// Encoder math
const float COUNTS_PER_REV = 3200.0;
long prev_pos1_counts = 0;
long prev_pos2_counts = 0;

// Control
float target_pos = 0;
float desired_speed[2] = {0,0};
float actual_speed[2] = {0,0};
float desired_pos[2] = {0,0};
float actual_pos[2] = {0,0};
float integral_error[2] = {0,0};
float Kp_pos = 7;
float Ki_pos = 0.2;
float pos_error[2] = {0,0};
float Kp = 2;
float base_PWM = 60;
float Battery_Voltage = 7.8;
float error[2] = {0,0};
float Voltage[2] = {0,0};
unsigned int PWM[2] = {0,0};

// Demo 1 position and angle variables
/*
//Test 1
float desired_rho = 108; // inches
float desired_phi = 0; // radians
*/
/*
//Test 2
float desired_rho = 76.367; // inches
float desired_phi = PI/4; // radians
*/

//Test 3
float desired_rho = 12; // inches
float desired_phi = PI; // radians


// Trim adjustments for uneven wheel movement
float right_trim = 1.013;
float left_trim = 1.023;
float turn_trim = 0.995;

// Robot measureables
const float wheel_radius = 3.0; // inches
const float wheel_base = 14.0; // inches

// Robot bearings
float robot_x = 0.0;
float robot_y = 0.0;
float robot_phi = 0.0;
float target_x = 0.0;
float target_y = 0.0;

// array mapping
const int RIGHT_IDX = 0;
const int LEFT_IDX = 1;

// Odometery
float prev_odom_right = 0.0;
float prev_odom_left = 0.0;
bool odom_initialized = false;
bool result_reported = false;

// navigation states for state machine
enum NavState {
  NAV_INIT,
  NAV_TURN,
  NAV_DRIVE,
  NAV_DONE
};

NavState nav_state = NAV_INIT;

// completion conditions
const float POS_TOL_RAD = 0.04;
const float SPEED_TOL_RAD_S = 0.25;
const int SETTLE_SAMPLES = 10;

int settled_samples = 0;

// angle wrapping helper, keeps angle between [-pi,+pi].
float wrapAngle(float angle) {
  while(angle > PI) {
    angle -= 2.0 * PI;
  }

  while(angle < -PI) {
    angle += 2.0 * PI;
  }

  return angle;
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

  // incremental wheel rotations in rads
  float dthetaR = thetaR - prev_odom_right;
  float dthetaL = thetaL - prev_odom_left;

  prev_odom_right = thetaR;
  prev_odom_left = thetaL;

  // incremental wheel travel in inches
  float dsR = wheel_radius * dthetaR;
  float dsL = wheel_radius * dthetaL;

  // robot forward displacement and heading change
  float ds = (dsR + dsL) / 2.0;
  float dphi = (dsR - dsL) / wheel_base;

  // midpoint integration for x-y position
  robot_x += ds * cos(robot_phi + dphi / 2.0);
  robot_y += ds * sin(robot_phi + dphi / 2.0);
  robot_phi = wrapAngle(robot_phi + dphi);
}

// Calculates target wheel positions and advances through turn, drive, and stop states in state machine
void updateNavigation() {
  if (nav_state == NAV_INIT) {
    // Capture the current wheel positions as the baseline
    desired_pos[RIGHT_IDX] = actual_pos[RIGHT_IDX];
    desired_pos[LEFT_IDX] = actual_pos[LEFT_IDX];

    // Expected final position
    target_x = desired_rho * cos(desired_phi);
    target_y = desired_rho * sin(desired_phi);

    // Shortest turn from current heading to target bearing
    float turn_angle = wrapAngle(desired_phi - robot_phi);

    // Wheel-angle changes for an in-place turn
    float dthetaR =
        turn_trim * (wheel_base * turn_angle) / (2.0 * wheel_radius);

    float dthetaL = -dthetaR;

    desired_pos[RIGHT_IDX] += dthetaR;
    desired_pos[LEFT_IDX] += dthetaL;

    settled_samples = 0;
    nav_state = NAV_TURN;

    Serial.println("Navigation: TURN");
  }

  else if (nav_state == NAV_TURN ||
           nav_state == NAV_DRIVE) {

    // Check whether both wheels have reached their targets
    bool position_ok =
        fabs(desired_pos[RIGHT_IDX] - actual_pos[RIGHT_IDX])
            < POS_TOL_RAD &&
        fabs(desired_pos[LEFT_IDX] - actual_pos[LEFT_IDX])
            < POS_TOL_RAD;

    bool speed_ok =
        fabs(actual_speed[RIGHT_IDX]) < SPEED_TOL_RAD_S &&
        fabs(actual_speed[LEFT_IDX]) < SPEED_TOL_RAD_S;

    if (position_ok && speed_ok) {
      settled_samples++;
    } else {
      settled_samples = 0;
    }

    // Require stable readings over multiple control samples
    if (settled_samples >= SETTLE_SAMPLES) {

      if (nav_state == NAV_TURN) {
        // Begin straight-line drive from current wheel positions
        desired_pos[RIGHT_IDX] =
            actual_pos[RIGHT_IDX] + right_trim * desired_rho / wheel_radius;

        desired_pos[LEFT_IDX] =
            actual_pos[LEFT_IDX] + left_trim * desired_rho / wheel_radius;

        settled_samples = 0;

        if (desired_rho > 0.0) {
          nav_state = NAV_DRIVE;
          Serial.println("Navigation: DRIVE");
        } else {
          nav_state = NAV_DONE;
        }
      }

      else if (nav_state == NAV_DRIVE) {
        nav_state = NAV_DONE;
      }
    }
  }
}

// ISR
void encoder1ISR(){
  int thisA = digitalRead(Apin);
  int thisB = digitalRead(Bpin);

  if (thisA == thisB) {
    rotation_count += 2;
  } 
  else {
    rotation_count -= 2;
  }
}

void encoder2ISR(){
  int thisC = digitalRead(Cpin);
  int thisD = digitalRead(Dpin);

  if (thisC == thisD) {
    rotation_count2 -= 2;
  } 
  else {
    rotation_count2 += 2;
  }
}

// Encoder reader, fixes counts by using 1 interrupt for encoder
long MyEnc(){
  long count;
  
  int thisA = digitalRead(Apin);
  int thisB = digitalRead(Bpin);

  noInterrupts();
  count = rotation_count;
  interrupts();

  if(thisA != thisB){
    count = count + 1;
  }

  return count;
}

// Encoder 2 reader, fixes counts by using 1 interrupt for encoder
long MyEnc2(){
  long count;
  
  int thisC = digitalRead(Cpin);
  int thisD = digitalRead(Dpin);

  noInterrupts();
  count = rotation_count2;
  interrupts();

  if(thisC != thisD){
    count = count + 1;
  }

  return count;
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

  // establish 115200 baud rate
  Serial.begin(115200);

  pinMode(Apin, INPUT_PULLUP);
  pinMode(Bpin, INPUT_PULLUP);
  pinMode(Cpin, INPUT_PULLUP);
  pinMode(Dpin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Apin), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Cpin), encoder2ISR, CHANGE);

  // Establish initial speed measurement baseline
  prev_pos1_counts = MyEnc();
  prev_pos2_counts = MyEnc2();

  delay(10000);

  last_time_ms = millis();
  start_time_ms = last_time_ms;
}


void loop() {
  unsigned long now = millis();

  // 10 ms control period
  if (now - last_time_ms < desired_Ts_ms) {
    return;
  }

  float dt = (now - last_time_ms) / 1000.0;
  last_time_ms = now;

  // Read encoder positions
  long pos1_counts = MyEnc();
  long pos2_counts = MyEnc2();

  // Convert counts to wheel angles
  actual_pos[0] = 2.0 * PI * (float)pos1_counts / COUNTS_PER_REV;
  actual_pos[1] = 2.0 * PI * (float)pos2_counts / COUNTS_PER_REV;

  // Calculate wheel speeds using actual elapsed time
  actual_speed[0] = 2.0 * PI * (pos1_counts - prev_pos1_counts) / (COUNTS_PER_REV * dt);

  actual_speed[1] = 2.0 * PI * (pos2_counts - prev_pos2_counts) / (COUNTS_PER_REV * dt);

  prev_pos1_counts = pos1_counts;
  prev_pos2_counts = pos2_counts;

  // estimating robot pose from incremental wheel motion
  updateOdometry();

  // Update the navigation command
  updateNavigation();

  // stop after the done with motion
  if (nav_state == NAV_DONE) {
    analogWrite(pwmPin[0], 0);
    analogWrite(pwmPin[1], 0);

    if (!result_reported) {
      result_reported = true;

      Serial.print("Estimated final x: ");
      Serial.println(robot_x, 3);

      Serial.print("Estimated final y: ");
      Serial.println(robot_y, 3);

      Serial.print("Estimated final heading: ");
      Serial.println(robot_phi, 4);

      float ex = target_x - robot_x;
      float ey = target_y - robot_y;

      Serial.print("Odometry target error: ");
      Serial.println(sqrt(ex * ex + ey * ey), 3);
    }

    return;
  }

  // mini project existing nested wheel controllers
  for (int i = 0; i < 2; i++) {
    pos_error[i] = desired_pos[i] - actual_pos[i];

    integral_error[i] += pos_error[i] * dt;

    // Position-loop anti-windup
    if (integral_error[i] > 2.0) {
      integral_error[i] = 2.0;
    }

    if (integral_error[i] < -2.0) {
      integral_error[i] = -2.0;
    }

    desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];

    const float MAX_WHEEL_SPEED = (nav_state == NAV_TURN) ? 1.5 : 3.0;

    if (desired_speed[i] > MAX_WHEEL_SPEED) {
      desired_speed[i] = MAX_WHEEL_SPEED;
    }

    if (desired_speed[i] < -MAX_WHEEL_SPEED) {
      desired_speed[i] = -MAX_WHEEL_SPEED;
    }

    error[i] = desired_speed[i] - actual_speed[i];

    Voltage[i] = Kp * error[i];

    // Limit requested voltage to the available supply
    if (Voltage[i] > Battery_Voltage) {
      Voltage[i] = Battery_Voltage;
    }

    if (Voltage[i] < -Battery_Voltage) {
      Voltage[i] = -Battery_Voltage;
    }

    if (Voltage[i] >= 0.0) {
      digitalWrite(signPin[i], HIGH);
    } else {
      digitalWrite(signPin[i], LOW);
    }

    PWM[i] = (unsigned int)(255.0 * fabs(Voltage[i]) / Battery_Voltage);

    if (PWM[i] > 255) {
      PWM[i] = 255;
    }

    analogWrite(pwmPin[i], PWM[i]);
  }
}



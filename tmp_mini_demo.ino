// Tomas Padilla - EENG350 Team 2 F26 - Controls
// Date: 10/04/2026
// Purpose: Implements a PI controller for two motors on the robot to spin the wheels to their correct
// locations given a input of a "quadrant region" from the CV scanner. Utilizes an anti-windup to prevent
// overshoot upon start of wheel motion. 
// Lines 108-133 are remnants from testing and should be removed/changed to agree with communication with the Pi

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
//float actual_speed2 = 0;
float Kp = 2;
float base_PWM = 60;
float Battery_Voltage = 7.8;
float error[2] = {0,0};
//float error2 = 0;
float Voltage[2] = {0,0};
//float Voltage2 = 0;
unsigned int PWM[2] = {0,0};
//unsigned int PWM2 = 0;

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

  last_time_ms = millis();
  start_time_ms = last_time_ms;
}

void loop() {
  long pos1_counts;
  long pos2_counts;
  float pos1_rad;
  float pos2_rad;

  pos1_counts = MyEnc();
  pos2_counts = MyEnc2();

  // speed calculations: (final pos - inital pos) / time
  actual_speed[0] = (2*PI*(float)(pos1_counts - prev_pos1_counts) / 3200) / Ts;
  actual_speed[1] = (2*PI*(float)(pos2_counts - prev_pos2_counts) / 3200) / Ts;

  // declaring variables for next cycle speed caluclations
  prev_pos1_counts = pos1_counts;
  prev_pos2_counts = pos2_counts;

  // counts -> radians conversion calculation
  pos1_rad = 2*PI*(float)pos1_counts / 3200;
  pos2_rad = 2*PI*(float)pos2_counts / 3200;

  actual_pos[0] = pos1_rad;
  actual_pos[1] = pos2_rad;

  // time in seconds
  current_time = float(last_time_ms - start_time_ms) / 1000;

  // Quadrant input testing via serial monitor
  if (Serial.available() > 0) {
    char quadrant = Serial.read();
    switch (quadrant) {
      case 'N': // NE: L=0, R=0
        desired_pos[0] = 0.0;
        desired_pos[1] = 0.0;
        Serial.println("NE -> L:0 R:0");
        break;
      case 'W': // NW: L=0, R=1
        desired_pos[0] = 0.0;
        desired_pos[1] = 3.14159;
        Serial.println("NW -> L:0 R:1");
        break;
      case 'S': // SW: L=1, R=1
        desired_pos[0] = 3.14159;
        desired_pos[1] = 3.14159;
        Serial.println("SW -> L:1 R:1");
        break;
      case 'E': // SE: L=1, R=0
        desired_pos[0] = 3.14159;
        desired_pos[1] = 0.0;
        Serial.println("SE -> L:1 R:0");
        break;
    }
  }

  // Motor integral control
  for(int i = 0; i < 2; i++) {
    pos_error[i] = desired_pos[i] - actual_pos[i];
    integral_error[i] = integral_error[i] + pos_error[i]*((float)desired_Ts_ms /1000);

    // Anti-windup
    if (integral_error[i] > 2.0) {
      integral_error[i] = 2.0;
    }
    if (integral_error[i] < -2.0) {
      integral_error[i] = -2.0;
    }

    desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];

    error[i] = desired_speed[i] - actual_speed[i];
    Voltage[i] = Kp * error[i];

    if(Voltage[i] > 0) {
      digitalWrite(signPin[i], HIGH);
    } else {
      digitalWrite(signPin[i], LOW);
    }
    PWM[i] = (unsigned int)(255 * abs(Voltage[i]) / Battery_Voltage);
    analogWrite(pwmPin[i], min(PWM[i], 255));
  }
  
  while (millis() < last_time_ms + desired_Ts_ms) {
    // wait
  }
  last_time_ms = millis();
}

// ISR
void encoder1ISR(){
  noInterrupts();
  int thisA = digitalRead(Apin);
  int thisB = digitalRead(Bpin);

  if (thisA == thisB) {
    rotation_count += 2;
  } else {
    rotation_count -= 2;
  }
  interrupts();
}

void encoder2ISR(){
  noInterrupts();
  int thisC = digitalRead(Cpin);
  int thisD = digitalRead(Dpin);

  if (thisC == thisD) {
    rotation_count2 -= 2;
  } else {
    rotation_count2 += 2;
  }
  interrupts();
}

// Encoder reader, fixes counts by using 1 interrupt for encoder
int MyEnc(){
  long count;
  
  int thisA = digitalRead(Apin);
  int thisB = digitalRead(Bpin);

  count = rotation_count;

  if(thisA != thisB){
    count = rotation_count + 1;
  }

  return count;
}

// Encoder 2 reader, fixes counts by using 1 interrupt for encoder
int MyEnc2(){
  long count;
  
  int thisC = digitalRead(Cpin);
  int thisD = digitalRead(Dpin);

  count = rotation_count2;

  if(thisC != thisD){
    count = rotation_count2 + 1;
  }

  return count;
}

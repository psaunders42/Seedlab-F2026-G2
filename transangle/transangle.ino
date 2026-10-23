#include <Wire.h>
#define ARD_ADDR 8
#define CMD_QUADRANT 0x01
#define CMD_ANGLE 0x02

volatile byte current_goal_byte = 0;
volatile float target_angle = 0.0;

void setup(){
  Wire.begin(ARD_ADDR);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);
}

void loop(){}

void receiveEvent(int howMany){
  if (howMany<2)return;

  byte command = Wire.read();

  if (command ==CMD_ANGLE && howMany >=3){
    byte lowByte = Wire.read();
    byte highByte = Wire.read();

    int16_t angle_raw = (int16_t)((highByte <<8)| lowByte);

    target_angle = angle_raw/100.0;
  }
}

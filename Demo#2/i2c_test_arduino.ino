// I2C receive test: Raspberry Pi (master) -> Arduino (slave, address 8)
// Prints every packet to the Serial Monitor (115200 baud). No motors/encoders used.
// Packet format (same as demo1.ino): [cmd=0x03][angle_lo][angle_hi][dist_lo][dist_hi]
//   angle = int16 degrees*100, dist = uint16 inches*10
#include <Arduino.h>
#include <Wire.h>

const uint8_t I2C_ADDR = 8;
const uint8_t CMD_TARGET = 0x03;

// Written in the I2C interrupt, read in loop()
volatile int16_t rx_angle_x100 = 0;
volatile uint16_t rx_dist_x10 = 0;
volatile unsigned long rx_time_ms = 0;
volatile unsigned long rx_count = 0;      // good packets
volatile unsigned long bad_count = 0;     // unexpected/short packets
volatile uint8_t last_cmd = 0;
volatile int last_len = 0;

unsigned long last_print_ms = 0;
unsigned long last_seen_count = 0;

void receiveEvent(int numBytes) {
  if (numBytes < 1) return;
  uint8_t cmd = Wire.read();
  numBytes--;
  last_cmd = cmd;
  last_len = numBytes + 1;

  if (cmd == CMD_TARGET && numBytes >= 4) {
    uint8_t b0 = Wire.read();
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    uint8_t b3 = Wire.read();
    rx_angle_x100 = (int16_t)((uint16_t)b0 | ((uint16_t)b1 << 8));
    rx_dist_x10 = (uint16_t)b2 | ((uint16_t)b3 << 8);
    rx_time_ms = millis();
    rx_count++;
  } else {
    bad_count++;
  }
  while (Wire.available()) Wire.read();   // discard leftovers
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Serial.println("I2C test ready. Listening at address 8...");
}

void loop() {
  unsigned long now = millis();
  if (now - last_print_ms < 250) return;   // print 4x per second
  last_print_ms = now;

  // atomic copy of the shared values
  noInterrupts();
  int16_t a = rx_angle_x100;
  uint16_t d = rx_dist_x10;
  unsigned long t = rx_time_ms;
  unsigned long good = rx_count;
  unsigned long bad = bad_count;
  uint8_t cmd = last_cmd;
  int len = last_len;
  interrupts();

  if (good == 0 && bad == 0) {
    Serial.println("Waiting for data from Pi... (nothing received yet)");
    return;
  }

  Serial.print("angle: ");
  Serial.print(a / 100.0, 2);
  Serial.print(" deg   dist: ");
  Serial.print(d / 10.0, 1);
  Serial.print(" in   age: ");
  Serial.print(now - t);
  Serial.print(" ms   good: ");
  Serial.print(good);
  Serial.print(" (+");
  Serial.print(good - last_seen_count);
  Serial.print(")   bad: ");
  Serial.print(bad);
  if (bad > 0) {
    Serial.print("   last cmd: 0x");
    Serial.print(cmd, HEX);
    Serial.print(" len: ");
    Serial.print(len);
  }
  Serial.println();
  last_seen_count = good;
}

/* 
 *  To control servos:
 *  pwm.setPWM(servo number, 0(always zero), pulse width);
 *  pulse width = a2p(servo number, degrees);
*/


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Min & Max Pulse Width for Servos
const int SERVOMIN0 = 180;
const int SERVOMAX0 = 600;
const int SERVOMIN1 = 180;
const int SERVOMAX1 = 600;
const int SERVOMIN2 = 180;
const int SERVOMAX2 = 650;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);


//float p2a(int n, float pulse) {
//  float angle;
//  switch (n) {
//    case 0: // base
//      angle = map(pulse, SERVOMIN, SERVOMAX, 0, 180);
//      break;
//    case 1: // shoulder
//      angle = map(pulse, SERVOMIN, SERVOMAX, 0, 180);
//      break;
//    case 2: // elbow
//      angle = map(pulse, SERVOMIN, SERVOMAX, 0, 180);
//      break;
//    case 3: // wrist
//      angle = map(pulse, SERVOMIN, SERVOMAX, 0, 180);
//      break;
//  }
//  return angle;
//}

// angle to pulse width
float a2p(int n, float angle) {
  float pulse;
  switch (n) {
    case 0: // base
      pulse = map(angle, 0, 180, SERVOMIN0, SERVOMAX0);
      break;
    case 1: // shoulder
      pulse = map(angle, 0, 180, SERVOMIN1, SERVOMAX1);
      break;
    case 2: // elbow
      pulse = map(angle, 0, 180, SERVOMIN2, SERVOMAX2);
      break;
//    case 3: // wrist
//      pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
//      break;
  }
  return pulse;
}


void setup() {
  Serial.begin(115200);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  yield();
}

void loop() {
//  // servo0 from 0-180 degrees
//  for (int i = a2p(0, 0); i <= a2p(0, 180); i++) {
//    pwm.setPWM(0, 0, i);
//    delay(15);
//  }
//  delay(1000);
//  for (int i = a2p(0, 180); i >= a2p(0, 0); i--) {
//    pwm.setPWM(0, 0, i);
//    delay(15);
//  }
//  delay(1000);

//  // servo1 from 0-180 degrees
//  for (int i = a2p(1, 0); i <= a2p(1, 180); i++) {
//    pwm.setPWM(1, 0, i);
//    delay(15);
//  }
//  delay(1000);
//  for (int i = a2p(1, 180); i >= a2p(1, 0); i--) {
//    pwm.setPWM(1, 0, i);
//    delay(15);
//  }
//  delay(1000);

  // servo2 from 0-180 degrees
  for (int i = a2p(2, 0); i <= a2p(2, 180); i++) {
    pwm.setPWM(2, 0, i);
    delay(15);
  }
  delay(1000);
  for (int i = a2p(2, 180); i >= a2p(2, 0); i--) {
    pwm.setPWM(2, 0, i);
    delay(15);
  }
  delay(1000);
}

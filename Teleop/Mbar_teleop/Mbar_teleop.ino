#include <JY901.h>
#include "Adafruit_VL53L0X.h"

// Arm 0 Pins
const int MOTOR0PIN1 = 12;
const int MOTOR0PIN2 = 14;
const int ENABLEPIN0 = 27;

// Arm 1 Pins
const int MOTOR1PIN1 = 26;
const int MOTOR1PIN2 = 25;
const int ENABLEPIN1 = 33;

// Arm 2 Pins
const int MOTOR2PIN1 = 5;
const int MOTOR2PIN2 = 18;
const int ENABLEPIN2 = 19;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 255;

Adafruit_VL53L0X lidar = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;


void setup() {
  Serial.begin(115200);
//  Serial2.begin(9600);
//  JY901.attach(Serial2);
//  lidar.begin();
  
  pinMode(MOTOR0PIN1, OUTPUT);
  pinMode(MOTOR0PIN2, OUTPUT);
  pinMode(ENABLEPIN0, OUTPUT);
  pinMode(MOTOR1PIN1, OUTPUT);
  pinMode(MOTOR1PIN2, OUTPUT);
  pinMode(ENABLEPIN1, OUTPUT);
  pinMode(MOTOR2PIN1, OUTPUT);
  pinMode(MOTOR2PIN2, OUTPUT);
  pinMode(ENABLEPIN2, OUTPUT);
  
  ledcSetup(pwmChannel0, freq, resolution);
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(ENABLEPIN0, pwmChannel0);
  ledcAttachPin(ENABLEPIN1, pwmChannel1);
  ledcAttachPin(ENABLEPIN2, pwmChannel2);

//  ledcWrite(pwmChannel0, dutyCycle);
//  ledcWrite(pwmChannel1, dutyCycle);
//  ledcWrite(pwmChannel2, dutyCycle);
}

void loop() {
}

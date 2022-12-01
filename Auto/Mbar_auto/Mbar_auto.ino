#include <Wire.h>
#include <JY901.h>
#include <Adafruit_PWMServoDriver.h>
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

// Min & Max Pulse Width for Servos
const int SERVOMIN0 = 150;
const int SERVOMAX0 = 600;
const int SERVOMIN1 = 150;
const int SERVOMAX1 = 600;
const int SERVOMIN2 = 150;
const int SERVOMAX2 = 650;
const int SERVOMIN3 = 150;
const int SERVOMAX3 = 555;

// PWM Properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 255;

// Servo Driver Board I2C Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Lidar Variables
Adafruit_VL53L0X lidar0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lidar1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lidar2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
float distance;

// General Variables
const float distance_to_bar = 120;
int track_sequence = 0;
int bar_count = 0;

// Boolean Flags
bool shouldChangeTrack = true;
bool baseHasArrived = false;
bool hasOpenHook0 = false;
bool hasOpenHook1 = true;
bool hasOpenHook2 = false;
bool isUnderBar = false;

struct track {
    int motorpin1;
    int motorpin2;
    bool hasOpenHook;
    Adafruit_VL53L0X lidar;
};

struct track track0 = {MOTOR0PIN1, MOTOR0PIN2, hasOpenHook0, lidar0};
struct track track1 = {MOTOR1PIN1, MOTOR1PIN2, hasOpenHook1, lidar1};
struct track track2 = {MOTOR2PIN1, MOTOR2PIN2, hasOpenHook2, lidar2};
track my_track;


//---------------------------------------FUNCTIONS---------------------------------------//


// Switch track variables according to the active track
struct track trackInfo(int track_num) {
    switch (track_num) {
        case 0:
            return track0;
        case 1:
            return track1;
        case 2:
            return track2;
    }
}

// Function to convert degrees to pulse width
float degToPulse(int servo_num, float angle) {
  float pulse;
  switch (servo_num) {
    case 0:
      pulse = map(angle, 0, 180, SERVOMIN0, SERVOMAX0);
      break;
    case 1:
      pulse = map(angle, 0, 180, SERVOMIN1, SERVOMAX1);
      break;
    case 2:
      pulse = map(angle, 0, 180, SERVOMIN2, SERVOMAX2);
      break;
  }
  return pulse;
}

// Stop all DC motors
void stopAll() {
  dutyCycle = 0;
  digitalWrite(MOTOR0PIN1, LOW);
  digitalWrite(MOTOR0PIN2, LOW);
  digitalWrite(MOTOR1PIN1, LOW);
  digitalWrite(MOTOR1PIN2, LOW);
  digitalWrite(MOTOR2PIN1, LOW);
  digitalWrite(MOTOR2PIN2, LOW);
}

// Move base forward or backward
void moveBase(int track_num, bool goForward) {
    moveTrack(((track_num + 1) % 3), !goForward);
    moveTrack(((track_num + 2) % 3), !goForward);
}

// Move individual tracks forward or backward
void moveTrack(int track_num, bool goForward) {
    if (track_num == 2) {
        digitalWrite(my_track.motorpin1, !goForward);
        digitalWrite(my_track.motorpin2, goForward);
    } else {
        digitalWrite(my_track.motorpin1, goForward);
        digitalWrite(my_track.motorpin2, !goForward);
    }
}

// Open or close hook
void hook(int track_num, String open_close) {
    if (open_close == "open") {
        pwm.setPWM(track_num, 0, degToPulse(track_num, 90));
        my_track.hasOpenHook = true;
    } else if (open_close == "close") {
        pwm.setPWM(track_num, 0, degToPulse(track_num, 0));
        my_track.hasOpenHook = false;
    }
}


void setup() {
    Serial.begin(115200);
  //  Serial2.begin(9600);
  //  JY901.attach(Serial2);
  //  lidar0.begin(0x29);
  //  lidar1.begin(0x2A);
  //  lidar2.begin(0x2B);
    
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
    if (shouldChangeTrack) {
        track_sequence = (track_sequence + 1) % 3;
        my_track = trackInfo(track_sequence);
        hook(track_sequence, "open");
        shouldChangeTrack = false;
    }

    my_track.lidar.rangingTest(&measure, false);
    distance = measure.RangeMilliMeter;
    bool hasDetectedBar = distance <= distance_to_bar;
        
    if (!baseHasArrived) {
        moveBase(track_sequence, true);

        if (!hasDetectedBar) isUnderBar = false;
        if (!isUnderBar && hasDetectedBar) {
            isUnderBar = true;
            baseHasArrived = true;
        }
        return;
    }
    
    if (!isUnderBar && hasDetectedBar && bar_count != 1) {
        bar_count += 1;
        isUnderBar = true;
    } else if (!isUnderBar && hasDetectedBar && bar_count == 1) {
        isUnderBar = true;
        stopAll();
        hook(track_sequence, "close");
        shouldChangeTrack = true;
        return;
    } else if (isUnderBar && !hasDetectedBar) {
        isUnderBar = false;
    } else moveTrack(track_sequence, true);

    delay(15);
}

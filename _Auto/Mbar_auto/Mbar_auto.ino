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

// Lidar Pins
const int LIDAR0PIN = 4;
const int LIDAR1PIN = 0;
const int LIDAR2PIN = 15;

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
    int pwmChannel;
    bool hasOpenHook;
    Adafruit_VL53L0X lidar;
};

struct track track0 = {MOTOR0PIN1, MOTOR0PIN2, pwmChannel0, hasOpenHook0, lidar0};
struct track track1 = {MOTOR1PIN1, MOTOR1PIN2, pwmChannel1, hasOpenHook1, lidar1};
struct track track2 = {MOTOR2PIN1, MOTOR2PIN2, pwmChannel2, hasOpenHook2, lidar2};
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

void something_imu() {
    JY901.receiveSerialData();
    float tilt_angle = JY901.getRoll();
    // float tilt_angle = JY901.getPitch();
    if (tilt_angle >= 30 || tilt_angle <= -30) stopAll();
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
    ledcWrite(pwmChannel0, dutyCycle);
    ledcWrite(pwmChannel1, dutyCycle);
    ledcWrite(pwmChannel2, dutyCycle);
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
    ledcWrite(my_track.pwmChannel, dutyCycle);
}

// Move base forward or backward
void moveBase(int track_num, bool goForward) {
    moveTrack(((track_num + 1) % 3), !goForward);
    moveTrack(((track_num + 2) % 3), !goForward);
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

//------------------------------------------SETUP & LOOP-------------------------------------------//

void setup() {
    Serial.begin(115200);
  //  Serial2.begin(9600);
  //  JY901.attach(Serial2);

    pinMode(MOTOR0PIN1, OUTPUT);
    pinMode(MOTOR0PIN2, OUTPUT);
    pinMode(ENABLEPIN0, OUTPUT);
    pinMode(MOTOR1PIN1, OUTPUT);
    pinMode(MOTOR1PIN2, OUTPUT);
    pinMode(ENABLEPIN1, OUTPUT);
    pinMode(MOTOR2PIN1, OUTPUT);
    pinMode(MOTOR2PIN2, OUTPUT);
    pinMode(ENABLEPIN2, OUTPUT);
    pinMode(LIDAR0PIN, OUTPUT);
    pinMode(LIDAR1PIN, OUTPUT);
    pinMode(LIDAR2PIN, OUTPUT);

    digitalWrite(LIDAR0PIN, LOW);
    digitalWrite(LIDAR1PIN, LOW);
    digitalWrite(LIDAR2PIN, LOW);

    digitalWrite(LIDAR0PIN, HIGH);
    lidar0.begin(0x2A);
    digitalWrite(LIDAR1PIN, HIGH);
    lidar1.begin(0x2B);
    digitalWrite(LIDAR2PIN, HIGH);
    lidar2.begin(0x2C);
    
    ledcSetup(pwmChannel0, freq, resolution);
    ledcSetup(pwmChannel1, freq, resolution);
    ledcSetup(pwmChannel2, freq, resolution);
    ledcAttachPin(ENABLEPIN0, pwmChannel0);
    ledcAttachPin(ENABLEPIN1, pwmChannel1);
    ledcAttachPin(ENABLEPIN2, pwmChannel2);
}

void loop() { 
    // defining the track sequence and opening the hook
    if (shouldChangeTrack) {
        track_sequence = (track_sequence + 1) % 3;
        my_track = trackInfo(track_sequence);
        hook(track_sequence, "open");
        shouldChangeTrack = false;
    }

    // constantly check whether the current track is detecting a bar
    my_track.lidar.rangingTest(&measure, false);
    distance = measure.RangeMilliMeter;
    bool hasDetectedBar = (distance <= distance_to_bar);
        
    // move the base until current track detects next bar
    if (!baseHasArrived) {
        moveBase(track_sequence, true);

        if (!hasDetectedBar) isUnderBar = false;                        // has not detected the bar
        if (!isUnderBar && hasDetectedBar) {                            // under the bar & detected the bar
            isUnderBar = true;
            baseHasArrived = true;
        }
        return;                                                         // looping, not under the bar or no bar detected
    }

    // move current track until it reaches second bar from initial position
    if (!isUnderBar && hasDetectedBar && bar_count != 1) {              // detects the bar, but not correct one
        bar_count += 1;
        isUnderBar = true;
    } else if (!isUnderBar && hasDetectedBar && bar_count == 1) {       // detects correct bar, close hook, & change track (RESET)
        isUnderBar = true;
        stopAll();
        hook(track_sequence, "close");
        shouldChangeTrack = true;
        baseHasArrived = false;
        bar_count = 0;
        return;
    } else if (isUnderBar && !hasDetectedBar) {                         // detects same bar as last iteration, ignore reading
        isUnderBar = false;
    } else moveTrack(track_sequence, true);                             // move current track

    delay(15);
}

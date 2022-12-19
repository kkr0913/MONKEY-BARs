#include <Wire.h>
#include <JY901.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_PWMServoDriver.h>

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

// Lidar XSHUT Pins
const int LIDAR0PIN = 4;
const int LIDAR1PIN = 0;
const int LIDAR2PIN = 15;

// LED Pins
const int LEDPIN0 = 13;
const int LEDPIN1 = 32;
const int LEDPIN2 = 23;
const int LEDPIN3 = 17;
const int LEDPIN4 = 2;

// Min & Max Pulse Width for Servos
const int SERVOMIN0 = 120;
const int SERVOMAX0 = 610;
const int SERVOMIN1 = 115;
const int SERVOMAX1 = 605;
const int SERVOMIN2 = 120;
const int SERVOMAX2 = 610;

// PWM Properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 180;

// Servo Driver Board I2C Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Lidar Variables
Adafruit_VL53L0X lidar0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lidar1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lidar2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
float distance;

// General Variables
const float dist_to_bar = 200;
const int SERVOMID0 = SERVOMIN0 + (SERVOMAX0 - SERVOMIN0) / 2;
const int SERVOMID1 = SERVOMIN1 + (SERVOMAX1 - SERVOMIN0) / 2;
const int SERVOMID2 = SERVOMIN2 + (SERVOMAX2 - SERVOMIN0) / 2;
int hook_angles[] = {SERVOMID0, SERVOMIN1, SERVOMID2};
int track_sequence = 0;
int bar_count = 0;

// Boolean Flags
bool shouldChangeTrack = true;  // flag to change track sequence
bool baseHasArrived = false;    // flag to stop base and move track
bool hasDetectedBar = false;    // ground truth lidar detection
bool isUnderPrevBar = true;     // flag to avoid closing hook right after opening it

// Track Info Struct
struct track {
    int motorpin1;
    int motorpin2;
    int pwmChannel;
    int servomin;
    int servomax;
    int servomid;
    Adafruit_VL53L0X lidar;
};

// Variables Associated with Each Track
struct track track0 = {MOTOR0PIN1, MOTOR0PIN2, pwmChannel0, SERVOMIN0, SERVOMAX0, SERVOMID0, lidar0};
struct track track1 = {MOTOR1PIN1, MOTOR1PIN2, pwmChannel1, SERVOMIN1, SERVOMAX1, SERVOMID1, lidar1};
struct track track2 = {MOTOR2PIN1, MOTOR2PIN2, pwmChannel2, SERVOMIN2, SERVOMAX2, SERVOMID2, lidar2};
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

// Function to control the LEDs
void ledControl() {
    int led0 = (random(0, 101) <= 55) ? 1 : 0;
    int led1 = (random(0, 101) <= 60) ? 1 : 0;
    int led2 = (random(0, 101) <= 65) ? 1 : 0;
    int led3 = (random(0, 101) <= 70) ? 1 : 0;
    int led4 = (random(0, 101) <= 75) ? 1 : 0;
    digitalWrite(LEDPIN0, led0);
    digitalWrite(LEDPIN1, led1);
    digitalWrite(LEDPIN2, led2);
    digitalWrite(LEDPIN3, led3);
    digitalWrite(LEDPIN4, led4);
}

// Detect the tilt with IMU
bool isTilted() {
    JY901.receiveSerialData();
    float roll = JY901.getRoll() + 180.0;
    float pitch = JY901.getPitch();
    if (roll >= 30 && roll <= 330) return true;
    if (pitch >= 30 || pitch <= -30) return true;
    return false;
}

// Stop all DC motors
void stopAll() {
    digitalWrite(MOTOR0PIN1, LOW);
    digitalWrite(MOTOR0PIN2, LOW);
    digitalWrite(MOTOR1PIN1, LOW);
    digitalWrite(MOTOR1PIN2, LOW);
    digitalWrite(MOTOR2PIN1, LOW);
    digitalWrite(MOTOR2PIN2, LOW);
}

// Move individual tracks forward or backward
void moveTrack(int track_num, bool goForward) {
    track current_track = trackInfo(track_num);
    stopAll();
    digitalWrite(current_track.motorpin1, !goForward);
    digitalWrite(current_track.motorpin2, goForward);
    ledcWrite(current_track.pwmChannel, dutyCycle);
}

// Move base forward or backward
void moveBase(int track_num, bool goForward) {
    track other_track1 = trackInfo((track_num + 1) % 3);
    track other_track2 = trackInfo((track_num + 2) % 3);

    // change the speed of the motors to zero
    ledcWrite(other_track1.pwmChannel, 0);
    ledcWrite(other_track2.pwmChannel, 0);

    // set the pins to the desired motor directions
    digitalWrite(other_track1.motorpin1, goForward);
    digitalWrite(other_track1.motorpin2, !goForward);
    digitalWrite(other_track2.motorpin1, goForward);
    digitalWrite(other_track2.motorpin2, !goForward);

    // start the motors motions to make them move simultaneously
    ledcWrite(other_track1.pwmChannel, dutyCycle);
    ledcWrite(other_track2.pwmChannel, dutyCycle);
}

// Open or close hook (open = servomin, close = servomax)
void moveHook(int track_num, String open_close) {
    track current_track = trackInfo(track_num);
    if (open_close == "open") {
        while (hook_angles[track_num] >= current_track.servomin) {
            hook_angles[track_num] -= 1;
            pwm.setPWM(track_num, 0, hook_angles[track_num]);
            delay(5);
        }
    } else if (open_close == "close") {
        while (hook_angles[track_num] <= current_track.servomid) {
            hook_angles[track_num] += 1;
            pwm.setPWM(track_num, 0, hook_angles[track_num]);
            delay(5);
        }
    }
}

//------------------------------------------SETUP & LOOP-------------------------------------------//

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);
    JY901.attach(Serial2);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(60);
    yield();

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
    pinMode(LEDPIN0, OUTPUT);
    pinMode(LEDPIN1, OUTPUT);
    pinMode(LEDPIN2, OUTPUT);
    pinMode(LEDPIN3, OUTPUT);
    pinMode(LEDPIN4, OUTPUT);

    digitalWrite(LEDPIN0, LOW);
    digitalWrite(LEDPIN1, LOW);
    digitalWrite(LEDPIN2, LOW);
    digitalWrite(LEDPIN3, LOW);
    digitalWrite(LEDPIN4, LOW);
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

    moveHook(0, "closed");
    moveHook(1, "open");
    moveHook(2, "closed");
}

void loop() {
    ledControl();
    // stop everything until the robot is stabilized
    if (isTilted()) {
      stopAll();
      return;
    }
    
    // defining the track sequence and opening the hook
    if (shouldChangeTrack) {
        track_sequence = (track_sequence + 1) % 3;
        my_track = trackInfo(track_sequence);
        moveHook(track_sequence, "open");
        moveHook((track_sequence + 1) % 3, "close");
        moveHook((track_sequence + 2) % 3, "close");
        shouldChangeTrack = false;
    }
    
    // constantly check whether the current track is detecting a bar
    my_track.lidar.rangingTest(&measure, false);
    distance = measure.RangeMilliMeter;
    hasDetectedBar = (distance <= dist_to_bar);

    // move the base until current track detects next bar
    if (!baseHasArrived) {
        moveBase(track_sequence, true);

        if (!hasDetectedBar) isUnderPrevBar = false;                             // moved away from previous bar (update flag)
        if (!isUnderPrevBar && hasDetectedBar) {                                 // detected next bar
            isUnderPrevBar = true;                                               // update the flag
            baseHasArrived = true;                                               // stop moving the base
        }
        return;                                                                  // loop (move the base) until the next bar is detected
    }

    // move current track until it reaches second bar from initial position
    if (!hasDetectedBar) isUnderPrevBar = false;                                 // moved away from previous bar (update flag)
    if (!isUnderPrevBar && hasDetectedBar && bar_count != 1) {                   // detects the bar, but not correct one
        bar_count += 1;
        isUnderPrevBar = true;
    } else if (!isUnderPrevBar && hasDetectedBar && bar_count == 1) {            // detects correct bar, close hook & change track (RESET)
        stopAll();
        moveHook(track_sequence, "close");
        isUnderPrevBar = true;
        shouldChangeTrack = true;
        baseHasArrived = false;
        bar_count = 0;
        return;
    } else moveTrack(track_sequence, true);                                      // move current track

    delay(50);
}

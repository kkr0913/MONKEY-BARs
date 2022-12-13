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

// Lidar Pins
const int LIDAR0PIN = 4;
const int LIDAR1PIN = 0;
const int LIDAR2PIN = 15;

// Min & Max Pulse Width for Servos
const int SERVOMIN0 = 120;
const int SERVOMAX0 = 625;
const int SERVOMIN1 = 115;
const int SERVOMAX1 = 615;
const int SERVOMIN2 = 120;
const int SERVOMAX2 = 635;

// PWM Properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 150;

// Servo Driver Board I2C Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Lidar Variables
Adafruit_VL53L0X lidar0 = Adafruit_VL53L0X();
Adafruit_VL53L0X lidar1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lidar2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
float distance;

// General Variables
const float dist_to_bar = 90;
int track_sequence = 0;
int bar_count = 0;
int hook_angles[] = {SERVOMAX0, SERVOMIN1, SERVOMAX2};

// Boolean Flags
bool shouldChangeTrack = true;
bool baseHasArrived = false;
bool hasOpenHook0 = false;
bool hasOpenHook1 = true;
bool hasOpenHook2 = false;
bool hasDetectedBar = false;   // ground truth lidar detection
bool isUnderPrevBar = false;       // flag to avoid closing hook right after opening it

struct track {
    int motorpin1;
    int motorpin2;
    int pwmChannel;
    int servomin;
    int servomax;
    bool hasOpenHook;
    Adafruit_VL53L0X lidar;
};

struct track track0 = {MOTOR0PIN1, MOTOR0PIN2, pwmChannel0, SERVOMIN0, SERVOMAX0, hasOpenHook0, lidar0};
struct track track1 = {MOTOR1PIN1, MOTOR1PIN2, pwmChannel1, SERVOMIN1, SERVOMAX1, hasOpenHook1, lidar1};
struct track track2 = {MOTOR2PIN1, MOTOR2PIN2, pwmChannel2, SERVOMIN2, SERVOMAX2, hasOpenHook2, lidar2};
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

bool emergencyBrake() {
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
            hook_angles[track_num] -= 5;
            pwm.setPWM(track_num, 0, hook_angles[track_num]);
        }
    } else if (open_close == "close") {
        while (hook_angles[track_num] <= current_track.servomax) {
            hook_angles[track_num] += 5;
            pwm.setPWM(track_num, 0, hook_angles[track_num]);
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
    Serial.print("Sequence: "); Serial.print(track_sequence); Serial.println();
    if (emergencyBrake()) {
      stopAll();
      return;
    }
    // defining the track sequence and opening the hook
    if (shouldChangeTrack) {
        track_sequence = (track_sequence + 1) % 3;
        my_track = trackInfo(track_sequence);
        moveHook(track_sequence, "open");
        shouldChangeTrack = false;
    }
    
    // constantly check whether the current track is detecting a bar
    my_track.lidar.rangingTest(&measure, false);
    distance = measure.RangeMilliMeter;
    hasDetectedBar = (distance <= dist_to_bar);
    Serial.print(distance); Serial.print(", "); Serial.print(hasDetectedBar); Serial.println();

    // move the base until current track detects next bar
    if (!baseHasArrived) {
        moveBase(track_sequence, true);

        if (!hasDetectedBar) isUnderPrevBar = false;                             // moved away from previous bar (update flag)
        if (!isUnderPrevBar && hasDetectedBar) {                                 // detected the bar
            isUnderPrevBar = true;                                               // update the flag
            baseHasArrived = true;                                               // stop moving the base
        }
        return;                                                                  // loop (move the base) until the next bar is detected
    }

    // move current track until it reaches second bar from initial position
    if (!hasDetectedBar) {                                                       // moved away from previous bar (update flag)
        isUnderPrevBar = false;
        Serial.println("moved away from previous bar (update flag)");
    }
    if (!isUnderPrevBar && hasDetectedBar && bar_count != 1) {                   // detects the bar, but not correct one
        bar_count += 1;
        isUnderPrevBar = true;
        Serial.println("detects the bar, but not correct one");
    } else if (!isUnderPrevBar && hasDetectedBar && bar_count == 1) {            // detects correct bar, close hook & change track (RESET)
        stopAll();
        moveHook(track_sequence, "close");
        isUnderPrevBar = true;
        shouldChangeTrack = true;
        baseHasArrived = false;
        bar_count = 0;
        Serial.println("detects correct bar, close hook & change track (RESET)");
        return;
    } else moveTrack(track_sequence, true);                                      // move current track

    delay(15);
}

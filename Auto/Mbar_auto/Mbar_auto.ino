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

// PWM Properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 255;

// Min & Max Pulse Width for Servos
const int SERVOMIN0 150;
const int SERVOMAX0 600;
const int SERVOMIN1 150;
const int SERVOMAX1 600;
const int SERVOMIN2 150;
const int SERVOMAX2 650;
const int SERVOMIN3 150;
const int SERVOMAX3 555;

// Servo Driver Board I2C Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Lidar Variables
float distance;
Adafruit_VL53L0X lidar0 = Adafruit_VL53L0X(0x41);
Adafruit_VL53L0X lidar1 = Adafruit_VL53L0X(0x42);
Adafruit_VL53L0X lidar2 = Adafruit_VL53L0X(0x43);
VL53L0X_RangingMeasurementData_t measure;

// General
const float distance_to_bar = 120;
int track_sequence = 0;

// Boolean Flags
bool hook0_open = false;
bool hook1_open = true;
bool hook2_open = false;
bool underbar = false;

struct track {
    int motorpin1;
    int motorpin2;
    bool hook_open;
    Adafruit_VL53L0X lidar;
};

struct track track0 = {MOTOR0PIN1, MOTOR0PIN2, hook0_open, lidar0};
struct track track1 = {MOTOR1PIN1, MOTOR1PIN2, hook1_open, lidar1};
struct track track2 = {MOTOR2PIN1, MOTOR2PIN2, hook2_open, lidar2};
track my_track;


//---------------------------------------FUNCTIONS---------------------------------------//


struct track track_info(int track_num) {
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
float deg2pulse(int servo_num, float angle) {
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
void stop_all() {
  dutyCycle = 0;
  digitalWrite(MOTOR0PIN1, LOW);
  digitalWrite(MOTOR0PIN2, LOW);
  digitalWrite(MOTOR1PIN1, LOW);
  digitalWrite(MOTOR1PIN2, LOW);
  digitalWrite(MOTOR2PIN1, LOW);
  digitalWrite(MOTOR2PIN2, LOW);
}

// Move individual tracks
void move_track(int track_num, bool goForward) {
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
        pwm.setPWM(track_num, 0, deg2pulse(track_num, 90));
        my_track.hook_open = true;
    } else if (open_close == "close") {
        pwm.setPWM(track_num, 0, deg2pulse(track_num, 0));
        my_track.hook_open = false;
    }
}

// Move basement forward until the back hook detects a bar
void move_basement(int track_num) {
        move_track(((track_num + 1) % 3), FALSE);
        move_track(((track_num + 2) % 3), FALSE);
}


void setup() {
    Serial.begin(115200);
  //  Serial2.begin(9600);
  //  JY901.attach(Serial2);
  //  lidar0.begin();
  //  lidar1.begin();
  //  lidar2.begin();
    
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
    track_sequence = (track_sequence + 1) % 3;
    my_track = track_info(track_sequence);

    hook(track_sequence, "open");
    
    while(1) {
        my_track.lidar.rangingTest(&measure, false);
        distance = measure.RangeMilliMeter;
        bool bar_detected = distance <= distance_to_bar;
        int count = 0;
        
        if (base_not_home) {
            move_basement(track_sequence);
            if (!bar_detected) underbar = false;

            if (!underbar && bar_detected) {
                underbar = true;
                base_not_home = !base_not_home;
            }
            continue;
        }
        
        if (!underbar && bar_detected && count != 1) {
            cnt += 1;
            underbar = true;
        } else if (!underbar && bar_detected && count == 1) {
            underbar = true;
            // delay(100);
            stop_all();
            hook(track_num, "close");
            break;
        } else if (underbar && !bar_detected) {
            underbar = false;
        } else move_track(track_sequence);
    }
}

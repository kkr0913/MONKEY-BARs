#include <WiFi.h>
#include <Wire.h>
#include <JY901.h>
#include <HTTPClient.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_PWMServoDriver.h>

/*
// WiFi Credentials for ESP32 to Connect
const char* ssid     = "smsphone";
const char* password = "helloworld";

// Server Routes Used by ESP32
HTTPClient http;
const char* URL = "http://172.20.10.11";
const char* serverGetSignal = "http://172.20.10.11:83/getSignal";
*/

// WiFi Credentials for ESP32 to Connect
const char* ssid     = "Tufts_Robot";
const char* password = "";

// Server Routes Used by ESP32
HTTPClient http;
const char* serverGetSignal = "http://10.247.137.35:83/getSignal";

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
const int SERVOMIN0 = 180;
const int SERVOMAX0 = 600;
const int SERVOMIN1 = 180;
const int SERVOMAX1 = 600;
const int SERVOMIN2 = 180;
const int SERVOMAX2 = 650;

// PWM Properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 150;

// Servo Driver Board I2C Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// General Variables
int track_sequence = 0;
int hook_angles[] = {90, 0, 90};

// Boolean Flags
bool trackChanged = false;

struct track {
    int motorpin1;
    int motorpin2;
    int pwmChannel;
};

struct track track0 = {MOTOR0PIN1, MOTOR0PIN2, pwmChannel0};
struct track track1 = {MOTOR1PIN1, MOTOR1PIN2, pwmChannel1};
struct track track2 = {MOTOR2PIN1, MOTOR2PIN2, pwmChannel2};


//-----------------------------------FUNCTIONS-----------------------------------//


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

// Get data from the server route
String httpGETRequest(const char* serverName) {
    http.begin(serverName);

    // send HTTP POST request
    int httpResponseCode = http.GET();

    String payload = "--";

    if (httpResponseCode>0) {
        payload = http.getString();
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }

    // free resources
    http.end();
    return payload;
}

// Post data to the server route
void httpPOSTRequest(const char* serverName, String postString) {
    http.begin(serverName);
    http.POST(postString);
    http.end();
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

// Control all actions according to the received data
void actionControl(String command) {
    Serial.println(command);
    if (command != "Increase Sequence" && command != "Decrease Sequence") trackChanged = false;

    // Move track forward (yolo front)
    if (command == "Move Track Forward")         moveTrack(track_sequence, true);
    // Move track backward (yolo back)
    else if (command == "Move Track Backward")   moveTrack(track_sequence, false);
    // Move base forward (fist front)
    else if (command == "Move Base Forward")     moveBase(track_sequence, true);
    // Move base backward (fist back)
    else if (command == "Move Base Backward")    moveBase(track_sequence, false);
    // Open hook (open peace)
    else if (command == "Open Hook")             moveHook(track_sequence, "open");
    // Close hook (closed peace)
    else if (command == "Close Hook")            moveHook(track_sequence, "close");
    // Change track sequence (thumbs up)
    else if (command == "Increase Sequence")     changeTrack("increment");
    // Change track sequence (thumbs down)
    else if (command == "Decrease Sequence")     changeTrack("decrement");
    // Stop all motors (palm open)
    else                                         stopAll();
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
    digitalWrite(other_track1.motorpin1, !goForward);
    digitalWrite(other_track1.motorpin2, goForward);
    digitalWrite(other_track2.motorpin1, !goForward);
    digitalWrite(other_track2.motorpin2, goForward);

    // start the motors motions to make them move simultaneously
    ledcWrite(other_track1.pwmChannel, dutyCycle);
    ledcWrite(other_track2.pwmChannel, dutyCycle);
}

// Open or close hook
void moveHook(int track_num, String open_close) {
    if (open_close == "open" && hook_angles[track_num] > 0) {
        hook_angles[track_num] -= 3;
        pwm.setPWM(track_num, 0, degToPulse(track_num, hook_angles[track_num]));
    } else if (open_close == "close" && hook_angles[track_num] < 90) {
        hook_angles[track_num] += 3;
        pwm.setPWM(track_num, 0, degToPulse(track_num, hook_angles[track_num]));
    }
}

// Change track sequence
void changeTrack(String updown) {
    if (!trackChanged) {
        if (updown == "increment")       track_sequence = (track_sequence + 1) % 3;
        else if (updown == "decrement")  track_sequence = (track_sequence + 2) % 3;
        trackChanged = true;
    }
}


//-----------------------------------SETUP & LOOP-----------------------------------//


void setup() {
    Serial.begin(115200);
    // Serial2.begin(9600);
    // JY901.attach(Serial2);
    // lidar.begin();
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
    
    ledcSetup(pwmChannel0, freq, resolution);
    ledcSetup(pwmChannel1, freq, resolution);
    ledcSetup(pwmChannel2, freq, resolution);
    ledcAttachPin(ENABLEPIN0, pwmChannel0);
    ledcAttachPin(ENABLEPIN1, pwmChannel1);
    ledcAttachPin(ENABLEPIN2, pwmChannel2);

    // Initialize and connect to WiFi
    WiFi.mode(WIFI_STA);
    // WiFi.begin(ssid, password);              // hotspot
    WiFi.begin(ssid);                           // tufts_robot
    WiFi.setSleep(false);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
}

void loop() {
    String rxdata = httpGETRequest(serverGetSignal);
    actionControl(rxdata);
    delay(15);
}

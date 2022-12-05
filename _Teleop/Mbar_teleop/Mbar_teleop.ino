#include <WiFi.h>
#include <Wire.h>
#include <JY901.h>
#include <HTTPClient.h>
#include "AsyncTCP.h"
#include "Adafruit_VL53L0X.h"
#include "ESPAsyncWebServer.h"
#include <Adafruit_PWMServoDriver.h>

// WiFi credentials for ESP32 to connect
const char* ssid = "smsphone";
const char* password = "helloworld";

// Server routes used by ESP32
HTTPClient http;
const char* URL = "http://172.20.10.11";
const char* serverGetSignal = "http://172.20.10.11:83/getSignal";

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
int dutyCycle = 150;

// Servo Driver Board I2C Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// General Variables
int track_sequence = 0;

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
track my_track;


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
    else if (command == "Open Hook")             hook(track_sequence, "open");
    // Close hook (closed peace)
    else if (command == "Close Hook")            hook(track_sequence, "close");
    // Change track sequence (thumbs up)
    else if (command == "Increase Sequence")     changeTrack("increment");
    // Change track sequence (thumbs down)
    else if (command == "Decrease Sequence")     changeTrack("decrement");
    // Stop all motors (palm open)
    else                                         stopAll();
}

// Change track sequence
void changeTrack(String updown) {
    if (!trackChanged) {
        if (updown == "increment") track_sequence = (track_sequence + 1) % 3;
        else if (updown == "decrement") track_sequence = (track_sequence + 2) % 3;
        my_track = trackInfo(track_sequence);
        trackChanged = true;
    }
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
    if (track_num == 2) {
        digitalWrite(my_track.motorpin1, goForward);
        digitalWrite(my_track.motorpin2, !goForward);
    } else {
        digitalWrite(my_track.motorpin1, !goForward);
        digitalWrite(my_track.motorpin2, goForward);
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


//-----------------------------------SETUP & LOOP-----------------------------------//


void setup() {
    Serial.begin(115200);
    // Serial2.begin(9600);
    // JY901.attach(Serial2);
    // lidar.begin();

    my_track = trackInfo(track_sequence);
    
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
    WiFi.begin(ssid, password);
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
    
    // ledcWrite(pwmChannel0, dutyCycle);
    // ledcWrite(pwmChannel1, dutyCycle);
    // ledcWrite(pwmChannel2, dutyCycle);
    delay(15);
}

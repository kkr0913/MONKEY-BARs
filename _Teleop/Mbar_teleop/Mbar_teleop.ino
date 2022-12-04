#include <JY901.h>
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include "AsyncTCP.h"
#include <HTTPClient.h>

// Hotspot for ESP32 to connect to: Wifi SSID and Password
const char* ssid = "smsphone";
const char* password = "helloworld";

// Server routes used by ESP32 microcontroller
HTTPClient http;
const char* URL = "http://172.20.10.11";
const char* serverGetSignal = URL + ":83/getSignal";

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

// Setting PWM properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 255;

// Servo Driver Board I2C Setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// General Variables
int track_sequence = 0;

struct track {
    int motorpin1;
    int motorpin2;
    int pwmChannel;
    bool hasOpenHook;
};

struct track track0 = {MOTOR0PIN1, MOTOR0PIN2, pwmChannel0, hasOpenHook0}
struct track track1 = {MOTOR1PIN1, MOTOR1PIN2, pwmChannel1, hasOpenHook1};
struct track track2 = {MOTOR2PIN1, MOTOR2PIN2, pwmChannel2, hasOpenHook2};
track my_track;


//-----------------------------------FUNCTIONS-----------------------------------//


// Get from the server route
String httpGETRequest(const char* serverName) {
    http.begin(serverName);

    // send HTTP POST request
    int httpResponseCode = http.GET();

    String payload = "--";

    if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        payload = http.getString();
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }

    // free resources
    http.end();
    return payload;
}

// Post to the server route
void httpPOSTRequest(const char* serverName, String postString) {
    http.begin(serverName);
    http.POST(postString);
    http.end();
}

void teleop_switching(char signalCase) {
    swtich(signalCase) {        
        case 'Stop':                                            // Stop all motors              palm open
            stopAll();
            break;
        case 'Move Track Forward':                              // Track forward                yolo front
            moveTrack(track_sequence, true);
            break;
        case 'Move Track Backward':                             // Track backward               yolo back
            moveTrack(track_sequence, false);
            break;
        case 'Move Base Forward':                               // Basement forward             fist front
            moveBase(track_sequence, true);
            break;
        case 'Move Base Backward':                              // Basement backward            fist back
            moveBase(track_sequence, false);
            break;
        case 'Open Hook':                                       // Open hook                    open peace sign
            hook(track_sequence, "open");
            break;
        case 'Close Hook':                                      // Close hook                   closed peace sign
            hook(track_sequence, "close");
            break;
        case 'Increase Sequence':                               // Change track sequence        thumbs up
            changeTrack("increment");
            break;
        case 'Decrease Sequence':                               // Change track sequence        thumbs down
            changeTrack("decrement");
            break;
    }
}

// Change track sequence
void changeTrack(String status) {
    if (!trackChanged) {
        if ( == "increment") track_sequence = (track_sequence + 1) % 3;
        else if ( == "decrement") track_sequence = (track_sequence + 2) % 3;
        trackChanged = true;
    }
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


//-----------------------------------SETUP & LOOP-----------------------------------//


void setup() {
    Serial.begin(115200);
    // Serial2.begin(9600);
    // JY901.attach(Serial2);
    // lidar.begin();
    
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

    // initialize the WiFi to connect to phone hotspot
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);

    Serial.print("Connecting to hotspot");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
}

void loop() {
    String signal = httpGETRequest(serverGetSignal);
    teleop_switching((char)signal);
}

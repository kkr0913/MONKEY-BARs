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

// Setting PWM properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int pwmChannel2 = 2;
const int resolution = 8;
int dutyCycle = 255;

Adafruit_VL53L0X lidar = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

//-----------------------------------FUNCTIONS-----------------------------------//

// get from the server route
String httpGETRequest(const char* serverName) {
 http.begin(serverName);

 // Send HTTP POST request
 int httpResponseCode = http.GET();

 String payload = "--";

 if (httpResponseCode>0) {
   Serial.print("HTTP Response code: ");
   Serial.println(httpResponseCode);
   payload = http.getString();
 }
 else {
   Serial.print("Error code: ");
   Serial.println(httpResponseCode);
 }

 // Free resources
 http.end();
 return payload;
}

// post to the server route
void httpPOSTRequest(const char* serverName, String postString) {
  http.begin(serverName);
  http.POST(postString);
  http.end();
}

/*

FUNCTIONS NEEDED THAT MAY BE THE SIMILAR TO AUTO: 
  void stopAllMotors()
  void moveTrack(int track_num, bool goForward)
  void moveBase(int track_num, bool goForward)
  void hook(int track_num, String open_close) 

*/

void teleop_switching(int caseNumber) {
  swtich(caseNumber):
  case 0:   //stop all motors

  case 100:   //move track 0 stop
  case 101:   //move track 0 forward
  case 102:   //move track 0 backward
  case 103:   //open hook 0
  case 104:   //close hook 0
  case 105:   //move base forwards - all motors but track 0
  case 106:   //move base backwards - all motors but track 0
  
  case 110:   //move track 1 stop
  case 111:   //move track 1 forward
  case 112:   //move track 1 backward
  case 113:   //open hook 1
  case 114:   //close hook 1
  case 115:   //move base forwards - all motors but track 1
  case 116:   //move base backwards - all motors but track 1

  case 120:   //move track 2 stop
  case 121:   //move track 2 forward
  case 122:   //move track 2 backward
  case 123:   //open hook 2
  case 124:   //close hook 2
  case 125:   //move base forwards - all motors but track 2
  case 126:   //move base backwards - all motors but track 2

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
  String caseNumber = httpGETRequest(serverGetSignal);
  teleop_switching((int)caseNumber);
}

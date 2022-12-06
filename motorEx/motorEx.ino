/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

// Arm 0 Pins
const int MOTOR0PIN1 = 12;
const int MOTOR0PIN2 = 14;
const int ENABLEPIN0 = 27;

// Arm 1 Pins
const int MOTOR1PIN1 = 26;
const int MOTOR1PIN2 = 25;
const int ENABLEPIN1 = 33;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
int dutyCycle = 150;

void setup() {
  // sets the pins as outputs:
  pinMode(MOTOR0PIN1, OUTPUT);    // out1
  pinMode(MOTOR0PIN2, OUTPUT);    // out2
  pinMode(ENABLEPIN0, OUTPUT);
  pinMode(MOTOR1PIN1, OUTPUT);
  pinMode(MOTOR1PIN2, OUTPUT);
  pinMode(ENABLEPIN1, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel0, freq, resolution);
  ledcSetup(pwmChannel1, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENABLEPIN0, pwmChannel0);
  ledcAttachPin(ENABLEPIN1, pwmChannel1);

  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
  ledcWrite(pwmChannel0, dutyCycle); 
  ledcWrite(pwmChannel1, dutyCycle); 
}

void loop() {
  ledcWrite(pwmChannel0, dutyCycle); 
  ledcWrite(pwmChannel1, dutyCycle);

  digitalWrite(MOTOR0PIN1, LOW);
  digitalWrite(MOTOR0PIN2, HIGH);
  digitalWrite(MOTOR1PIN1, LOW);
  digitalWrite(MOTOR1PIN2, LOW);
  delay(500);

  digitalWrite(MOTOR0PIN1, LOW);
  digitalWrite(MOTOR0PIN2, LOW);
  digitalWrite(MOTOR1PIN1, LOW);
  digitalWrite(MOTOR1PIN2, HIGH); 
  delay(500);
}

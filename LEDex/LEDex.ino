#define BUTTONPIN 25
#define LEDPIN 27

int buttonState_p = 0;
int buttonState_c = 0;
bool finished = true;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
}

void loop() {
  buttonState_c = digitalRead(BUTTONPIN);

  // Prevent consecutive button press and button hold during servo motions
  if (buttonState_c == LOW || buttonState_c == buttonState_p || finished == false) {
    buttonState_p = buttonState_c;
    return;
  }

  Serial.println("pressed");
  digitalWrite(LEDPIN, HIGH);

  // Servos actions are finished
  buttonState_p = buttonState_c;
  finished = true;
}

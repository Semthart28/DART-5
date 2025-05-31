/ PWM Traffic Light Example
const int redPin    = 9;   // must be PWM pin
const int yellowPin = 10;  // must be PWM pin
const int greenPin  = 11;  // must be PWM pin

void setup() {
  pinMode(redPin,    OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin,  OUTPUT);
}

void loop() {
  // Turn red on at 100% brightness for 5 s
  analogWrite(redPin, 255);
  delay(5000);
  analogWrite(redPin, 0);

  // Turn green on at 100% brightness for 5 s
  analogWrite(greenPin, 255);
  delay(5000);
  analogWrite(greenPin, 0);

  // Turn yellow on at 100% brightness for 2 s
  analogWrite(yellowPin, 255);
  delay(2000);
  analogWrite(yellowPin, 0);
}

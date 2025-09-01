const int pwmChannel = 0;
const int pwmFreq = 5000;     // PWM frequency
const int pwmResolution = 8;  // 8-bit resolution (0–255)
const int pwmPin = 13;
const int digitalPin = 12;

void setup() {
  // Set up the digital pin
  pinMode(digitalPin, OUTPUT);
  // digitalWrite(digitalPin, LOW);

  // Set up the PWM pin
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);  // configure channel
  ledcAttachPin(pwmPin, pwmChannel);              // attach channel to pin
  // ledcWrite(pwmChannel, 0);                        // initial duty cycle
}

void loop() {
  ledcWrite(pwmChannel, 100);    // 100/255 PWM duty cycle
  digitalWrite(digitalPin, LOW);
  delay(5000);

  ledcWrite(pwmChannel, 50);     // 50/255 PWM duty cycle
  delay(5000);

  ledcWrite(pwmChannel, 100);
  digitalWrite(digitalPin, HIGH);
  delay(5000);

  ledcWrite(pwmChannel, 50);
  delay(5000);
}

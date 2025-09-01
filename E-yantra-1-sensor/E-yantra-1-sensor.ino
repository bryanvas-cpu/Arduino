#define TRIG 3
#define ECHO 2
#define IR_LED 4
#define BUZZER 5

#define THRESHOLD 15  // Distance threshold in cm

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(IR_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  long duration;
  float distance;

  // Send a pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Read echo pulse duration
  duration = pulseIn(ECHO, HIGH);
  distance = (duration * 0.034) / 2;  // Convert to cm

  if (distance > 0 && distance < THRESHOLD) {
    digitalWrite(IR_LED, HIGH);     // Turn on IR LED
    tone(BUZZER, 1000);             // Passive buzzer with 1kHz frequency
  } else {
    digitalWrite(IR_LED, LOW);      // Turn off IR LED
    noTone(BUZZER);                 // Stop buzzer
  }

  delay(100);  // Small delay for stability
}

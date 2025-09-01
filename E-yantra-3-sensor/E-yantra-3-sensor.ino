#define TRIG1 3
#define ECHO1 2
#define LED1 8
#define BUZZER1 11

#define TRIG2 5
#define ECHO2 4
#define LED2 9
#define BUZZER2 12

#define TRIG3 7
#define ECHO3 6
#define LED3 10
#define BUZZER3 13

#define THRESHOLD 15  // Distance threshold in cm

void setup() {
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(BUZZER1, OUTPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(LED2, OUTPUT);
  pinMode(BUZZER2, OUTPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUZZER3, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  checkSensor(TRIG1, ECHO1, LED1, BUZZER1);
  checkSensor(TRIG2, ECHO2, LED2, BUZZER2);
  checkSensor(TRIG3, ECHO3, LED3, BUZZER3);

  delay(100);  // Small delay to avoid overlapping sensor signals
}

void checkSensor(int trigPin, int echoPin, int ledPin, int buzzerPin) {
  long duration;
  float distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * 0.034) / 2;

  if (distance > 0 && distance < THRESHOLD) {
    digitalWrite(ledPin, HIGH);
    tone(buzzerPin, 1000);  // 1kHz frequency
  } else {
    digitalWrite(ledPin, LOW);
    noTone(buzzerPin);
  }
}

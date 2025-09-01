// Simple Encoder Tick Counter for ESP32
// Counts ticks on a single encoder and displays them in Serial Monitor

#define r_enc_a 13  // Encoder channel A
#define r_enc_b 12  // Encoder channel B

volatile long encoderTicks = 0;

void IRAM_ATTR encoderISR() {
  int dir = digitalRead(r_enc_b) ? 1 : -1;  // Direction from channel B
  encoderTicks += dir;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(r_enc_a, INPUT);
  pinMode(r_enc_b, INPUT);

  // Trigger ISR on rising edge of channel A
  attachInterrupt(digitalPinToInterrupt(r_enc_a), encoderISR, RISING);
}

void loop() {
  static long lastTicks = 0;
  if (encoderTicks != lastTicks) {
    lastTicks = encoderTicks;
    Serial.print("Encoder ticks: ");
    Serial.println(encoderTicks);
  }
}

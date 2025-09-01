#include "servo_ik.h"

void setup(){
  Serial.begin(115200);
  setupServoIK();
}

void loop(){
  ServoIK(-15, 250);
  delay(1000);
  ServoIK(-10, 250);
  delay(1000);
}

#include "dualshock.h"

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // in milliseconds

void setup(){
  Serial.begin(115200);
  setupJOY();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;
    JOY_CMD joy_cmd = UpdateJoyloop();
    Serial.println(String(joy_cmd.on_off_state) + " " + 
                   String(joy_cmd.left_joy_cmd) + " " + 
                   String(joy_cmd.right_joy_cmd));
  }
}
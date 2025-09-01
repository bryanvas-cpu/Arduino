#include "motor_driver_cytron.h"

void setup() {
  Serial.begin(115200);
  motor_driver_setup();
}

void loop() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    command = inputString.toDouble();
    Serial.print("Target RPM set to: ");
    Serial.println(command, 2);
    inputString = "";
    stringComplete = false;
  }

  motor_driver_loop((int)command, (int)(command/2));
  DriveVel drive_vel = query_vel();
  Serial.println("lin" + String(drive_vel.lin_vel) + " ang " + String(drive_vel.ang_vel));
}

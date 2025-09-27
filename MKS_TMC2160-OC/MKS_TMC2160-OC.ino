#include "stepper_driver_MKS_TMC2160-OC.h"

long rpm1 = 0;
long rpm2 = 0;
long step = 5;          // change per iteration
long maxRpm1 = 200;     // maximum RPM Motor 1
long maxRpm2 = 200;      // maximum RPM Motor 2

bool increasing = true;  // direction of sweep

void setup() {
  Serial.begin(115200);
  motor_driver_init();
}

void loop() {
  // --- Sweep Motor 1 ---
  if (increasing) {
    rpm1 += step;
    rpm2 += step;
    if (rpm1 >= maxRpm1 || rpm2 >= maxRpm2) {
      increasing = false;
    }
  } else {
    rpm1 -= step;
    rpm2 -= step;
    if (rpm1 <= -maxRpm1 || rpm2 <= -maxRpm2) {
      increasing = true;
    }
  }

  motor_driver_set_omega(-6.28, 6.28);

  Serial.print(" | Motor1 RPM: ");
  Serial.print(rpm1);
  // Serial.print(" | consecutve pulse interval: ");
  // Serial.println(consecutive_pulse_interval);
  // Serial.print(" | Motor2 RPM: ");
  // Serial.println(rpm2);

  delay(100); // adjust speed of sweep
}

#include "imu_sensor.h"

void setup() {
  Serial.begin(115200);
  delay(2000);
  setupIMU();
}

void loop() {
  IMUData imuData = updateIMU();

  Serial.print("Roll: ");
  Serial.print(imuData.roll);
  Serial.print("°, Pitch: ");
  Serial.print(imuData.pitch);
  Serial.print("°, Yaw: ");
  Serial.print(imuData.yaw);
  Serial.print("°, ");
  Serial.print("Angular Vel X (rad/s): ");
  Serial.print(imuData.angularVelX);
  Serial.print(", Angular Vel Y (rad/s): ");
  Serial.print(imuData.angularVelY);
  Serial.print(", Angular Vel Z (rad/s): ");
  Serial.println(imuData.angularVelZ);
  delay(50);
}

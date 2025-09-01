#include <Wire.h>
#include <SparkFunMPU9250-DMP.h>

MPU9250_DMP imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (imu.begin() != INV_SUCCESS) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  imu.setGyroFSR(2000);  // deg/s
  imu.setAccelFSR(2);    // g
  imu.setLPF(5);         // Hz
  imu.setSampleRate(50); // Hz
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 10);

  Serial.println("IMU initialized.");
}

void loop() {
  if (imu.fifoAvailable()) {
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      // Get quaternion
      float q0 = imu.calcQuat(0);
      float q1 = imu.calcQuat(1);
      float q2 = imu.calcQuat(2);
      float q3 = imu.calcQuat(3);

      // Convert quaternion to Euler angles (in radians)
      float roll  = atan2(2.0f * (q0 * q1 + q2 * q3),
                          1.0f - 2.0f * (q1 * q1 + q2 * q2));
      float pitch = asin(2.0f * (q0 * q2 - q3 * q1));
      float yaw   = atan2(2.0f * (q0 * q3 + q1 * q2),
                          1.0f - 2.0f * (q2 * q2 + q3 * q3));

      // Convert radians to degrees
      roll  *= 180.0 / PI;
      pitch *= 180.0 / PI;
      yaw   *= 180.0 / PI;

      Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", roll, pitch, yaw);
    }
  }
}

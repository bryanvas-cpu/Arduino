#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <MadgwickAHRS.h>

MPU9250_asukiaaa mySensor;
Madgwick filter;

float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;

void calibrateGyro(int numSamples = 500) {
  Serial.println("Calibrating gyroscope... Keep the sensor still.");
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    mySensor.gyroUpdate();
    sumX += mySensor.gyroX();
    sumY += mySensor.gyroY();
    sumZ += mySensor.gyroZ();
    delay(5);
  }

  gyroBiasX = sumX / numSamples;
  gyroBiasY = sumY / numSamples;
  gyroBiasZ = sumZ / numSamples;

  Serial.println("Calibration complete.");
  Serial.printf("Gyro Bias: X=%.2f, Y=%.2f, Z=%.2f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();

  mySensor.beginMag();
  Serial.println("Magnetometer initialized (no return check).");

  mySensor.magUpdate();
  Serial.printf("Magnetometer values -> X: %.2f, Y: %.2f, Z: %.2f\n", mySensor.magX(), mySensor.magY(), mySensor.magZ());


  filter.begin(1000); // 100 Hz update rate

  calibrateGyro();
  delay(1000);
}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  // Get sensor values
  float ax = mySensor.accelX();
  float ay = mySensor.accelY();
  float az = mySensor.accelZ();

  float gx = (mySensor.gyroX() - gyroBiasX) * DEG_TO_RAD;
  float gy = (mySensor.gyroY() - gyroBiasY) * DEG_TO_RAD;
  float gz = (mySensor.gyroZ() - gyroBiasZ) * DEG_TO_RAD;

  float mx = mySensor.magX();
  float my = mySensor.magY();
  float mz = mySensor.magZ();

  // Update the filter
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  // Get Euler angles (in degrees)
  float roll  = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw   = filter.getYaw();

  Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, %.2f %.2f %.2f\n", roll, pitch, yaw, mx, my, mz);

  delay(1); // run at ~100Hz
}

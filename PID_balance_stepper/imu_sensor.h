#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa imu;

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

float gx, gy, gz;
float ax, ay, az;

const float alpha = 0.05;
static float f_ax = 0, f_ay = 0, f_az = 0;
static float f_gx = 0, f_gy = 0, f_gz = 0;
static float f_roll = 0, f_pitch = 0, f_yaw = 0;

struct IMUData {
    float linearAccX;
    float roll;
    float pitch;
    float yaw;
    float angularVelX;
    float angularVelY;
    float angularVelZ;  // in rad/s
  };
  
void calibrateGyro(int numSamples = 500) {
  Serial.println("Calibrating gyroscope... Keep the sensor still.");

  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    imu.gyroUpdate();
    sumX += imu.gyroX();
    sumY += imu.gyroY();
    sumZ += imu.gyroZ();
    delay(5);  // wait for next reading
  }

  gyroBiasX = sumX / numSamples;
  gyroBiasY = sumY / numSamples;
  gyroBiasZ = sumZ / numSamples;

  Serial.println("Calibration complete.");
  Serial.printf("Gyro Bias: X=%.2f, Y=%.2f, Z=%.2f\n\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void setupIMU() {
    delay(100);
    imu.setWire(&Wire);
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag(); // even if not working, harmless
  
    delay(100);
    
    calibrateGyro();
}

IMUData updateIMU() {
  float gx = 0, gy = 0, gz = 0;
  float ax = 0, ay = 0, az = 0;

  imu.gyroUpdate();
  gx += imu.gyroX() - gyroBiasX;
  gy += imu.gyroY() - gyroBiasY;
  gz += imu.gyroZ() - gyroBiasZ;

  imu.accelUpdate();
  ax += imu.accelX();
  ay += imu.accelY();
  az += imu.accelZ();

  // --- raw orientation ---
  float roll  = atan2(ay, -az) * 180.0 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float yaw = 0;

  // --- exponential smoothing (alpha = 0.5) ---


  f_ax = alpha * ax + (1 - alpha) * f_ax;
  f_ay = alpha * ay + (1 - alpha) * f_ay;
  f_az = alpha * az + (1 - alpha) * f_az;

  f_gx = alpha * gx + (1 - alpha) * f_gx;
  f_gy = alpha * gy + (1 - alpha) * f_gy;
  f_gz = alpha * gz + (1 - alpha) * f_gz;

  f_roll  = alpha * roll + (1 - alpha) * f_roll;
  f_pitch = alpha * pitch + (1 - alpha) * f_pitch;
  f_yaw   = alpha * yaw + (1 - alpha) * f_yaw;

  // --- return filtered values ---
  IMUData result = {f_ax, f_roll, f_pitch, f_yaw, f_gx, f_gy, f_gz};
  return result;
}

#endif
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

float gx, gy, gz;
float ax, ay, az;

void calibrateGyro(int numSamples = 500) {
  Serial.println("Calibrating gyroscope... Keep the sensor still.");

  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < numSamples; i++) {
    mySensor.gyroUpdate();
    sumX += mySensor.gyroX();
    sumY += mySensor.gyroY();
    sumZ += mySensor.gyroZ();
    delay(5);  // wait for next reading
  }

  gyroBiasX = sumX / numSamples;
  gyroBiasY = sumY / numSamples;
  gyroBiasZ = sumZ / numSamples;

  Serial.println("Calibration complete.");
  Serial.printf("Gyro Bias: X=%.2f, Y=%.2f, Z=%.2f\n\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(); // SDA = GPIO21, SCL = GPIO22 on ESP32
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag(); // even if not working, harmless

  delay(100);

  calibrateGyro();
}

void loop() {
  // Gyro data averaging (radians/sec)
  gx = 0; gy = 0; gz = 0;
  for (int i = 0; i < 10; i++) {
    mySensor.gyroUpdate();
    gx += mySensor.gyroX() - gyroBiasX;
    gy += mySensor.gyroY() - gyroBiasY;
    gz += mySensor.gyroZ() - gyroBiasZ;
  }
  gx *= 3.1415 / (10 * 180);
  gy *= 3.1415 / (10 * 180);
  gz *= 3.1415 / (10 * 180);

  // Accelerometer data averaging (m/s^2)
  ax = 0; ay = 0; az = 0;
  for (int i = 0; i < 10; i++) {
    mySensor.accelUpdate();
    ax += mySensor.accelX();
    ay += mySensor.accelY();
    az += mySensor.accelZ();
  }
  ax *= 9.81 / 10;
  ay *= 9.81 / 10;
  az *= 9.81 / 10;

  // === Calculate Roll and Pitch ===
  float roll  = atan2(ay, -az) * 180.0 / PI;                          // X-axis tilt
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;   // Y-axis tilt

  // === Output ===
Serial.print(roll);
Serial.print("\t");
Serial.print(pitch);
Serial.print("\t");
Serial.print(ax);
Serial.print("\t");
Serial.print(ay);
Serial.print("\t");
Serial.print(az);
Serial.print("\t");
Serial.print(gx);
Serial.print("\t");
Serial.print(gy);
Serial.print("\t");
Serial.println(gz);


  delay(50);
}

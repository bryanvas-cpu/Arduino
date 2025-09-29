#include "stepper_driver_MKS_TMC2160-OC.h"
#include "servo_ik.h"
#include "imu_sensor.h"
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Timing diagnostics (in microseconds)
volatile unsigned long loop_dt = 0;
volatile unsigned long servo_dt = 0;
volatile unsigned long imu_dt   = 0;

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // ms
double body_height_command = 330;

struct STATE{
  double x_dot;
  double theta;
  double theta_dot;
  double alpha;
  double alpha_dot;
};

STATE state;

//--- PID gains for theta ---
float Kp_theta = 3.0;
float Kd_theta = 0.00005;
float Ki_theta = 0.000001;
float theta_integral = 0.0;
double theta_filtered = 0.0;
float alpha_theta = 0.1;
float alpha_theta_dot = 0.1;

//--- PID gains for velocity ---
float Kp_vel = 2.1;
float Kd_vel = 0.00005;
float Ki_vel = 0.000001;
double vel_integral = 0.0;
double vel_prev_error = 0.0;

double freq = 750;

// ----- FreeRTOS queue for IMU samples -----
static QueueHandle_t imuQueue = NULL; // holds the latest IMUData (queue length 1)

//shared variables
float roll_cmd_global = 0;
float body_height_global = 330;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;

void ServoTask(void *pvParameters) {
  unsigned long prev = micros();
  while (true) {
    unsigned long now = micros();
    servo_dt = now - prev;   // how often ServoTask runs
    prev = now;

    float roll, height;
    portENTER_CRITICAL(&servoMux);
    roll = roll_cmd_global;
    height = body_height_global;
    portEXIT_CRITICAL(&servoMux);

    ServoIK(roll, height);

    vTaskDelay(10 / portTICK_PERIOD_MS); // ~100 Hz
  }
}

// ---- IMU acquisition task ----
// Reads IMU at ~200 Hz and prints values
void IMUTask(void *pvParameters) {
  const TickType_t imuDelay = pdMS_TO_TICKS(5); // ~200 Hz
  IMUData imuSample;
  unsigned long prev = micros();

  while (true) {
    unsigned long now = micros();
    imu_dt = now - prev;   // how often IMUTask runs
    prev = now;

    imuSample = updateIMU();
    if (imuQueue) {
      xQueueOverwrite(imuQueue, &imuSample);
    }

    // ---- Print IMU values here ----
    Serial.print(imuSample.linearAccX); Serial.print("\t");
    Serial.print(imuSample.roll);       Serial.print("\t");
    Serial.print(imuSample.pitch);      Serial.print("\t");
    Serial.print(imuSample.yaw);        Serial.print("\t");
    Serial.print(imuSample.angularVelX); Serial.print("\t");
    Serial.print(imuSample.angularVelY); Serial.print("\t");
    Serial.print(imuSample.angularVelZ); Serial.print("\t");
    Serial.println(imu_dt);

    vTaskDelay(imuDelay);
  }
}


void setup() {
  Wire.begin(21, 22, 400000);
  Serial.begin(115200);

  setupServoIK();
  setupIMU();
  delay(1000);
  motor_driver_init();

  // create queue with length 1 to hold latest IMUData
  imuQueue = xQueueCreate(1, sizeof(IMUData));
  if (imuQueue == NULL) {
    Serial.println("Failed to create IMU queue!");
  }

  // Create IMU task on Core 0 with higher priority than servo if desired
  xTaskCreatePinnedToCore(
    IMUTask,           // task function
    "IMUTask",         // name
    4096,              // stack size
    NULL,              // params
    3,                 // priority (adjust as needed)
    NULL,              // handle
    0                  // core 0
  );

  // ServoTask pinned to core 1 as before
  xTaskCreatePinnedToCore(
    ServoTask,
    "ServoTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  static unsigned long prev = micros();
  unsigned long now = micros();
  loop_dt = now - prev;   // how often loop() runs
  prev = now;

  double loop_start_time = millis();

  // Servo shared updates
  portENTER_CRITICAL(&servoMux);
  roll_cmd_global = 0;       // update as needed
  body_height_global = 330;
  portEXIT_CRITICAL(&servoMux);

  // Try to read latest IMU sample from queue (non-blockinga)
  IMUData imuData;
  bool gotIMU = false;
  if (imuQueue && xQueueReceive(imuQueue, &imuData, 0) == pdPASS) {
    gotIMU = true;
  }

  if (gotIMU) {
    double accel = imuData.linearAccX;     // m/s^2
    double omega_gyro = -imuData.angularVelZ;

    // Print IMU values (printing here keeps print frequency controlled by loop)
    // Serial.print(imuData.linearAccX); Serial.print(",");
    // Serial.print(imuData.roll);       Serial.print(",");
    // Serial.print(imuData.pitch);      Serial.print(",");
    // Serial.print(imuData.yaw);        Serial.print(",");
    // Serial.print(imuData.angularVelX); Serial.print(",");
    // Serial.print(imuData.angularVelY); Serial.print(",");
    // Serial.print(imuData.angularVelZ);
    // Serial.print(" ");

    // Set Target Linear and Angular Velocities
    double lin_vel_cmd = 0;
    double ang_vel_cmd = 0;
    if(lin_vel_cmd >= -1 && lin_vel_cmd <= 1){
      lin_vel_cmd = 0;
    }
    if(ang_vel_cmd >= -1 && ang_vel_cmd <= 1){
      ang_vel_cmd = 0;
    }

    // Velocity PID
    double vel_error = lin_vel_cmd - drive_vel.lin_vel;
    vel_integral += vel_error * (1.0/freq);
    vel_integral = constrain(vel_integral, -1, 1); // anti-windup
    double vel_derivative = (vel_error - vel_prev_error) / (1.0/freq);
    vel_prev_error = vel_error;
    double vel_pid_output = Kp_vel * vel_error
                          + Ki_vel * vel_integral
                          + Kd_vel * vel_derivative;

    // Target Pitch
    double target_pitch = vel_pid_output;
    target_pitch = constrain(target_pitch, -3.0, 3.0);

    // Angle PID
    state.theta = alpha_theta * state.theta + (1-alpha_theta) * imuData.pitch;
    state.theta_dot = alpha_theta_dot * state.theta_dot + (1-alpha_theta_dot) * imuData.angularVelY;
    double error = target_pitch - state.theta;
    theta_integral += error * ( 1.0 / freq);
    double derivative = -state.theta_dot;
    if(theta_integral >= 0.5 ) theta_integral = 0.5;
    if(theta_integral <= -0.5) theta_integral = -0.5;
    double theta_output = Kp_theta * error + Kd_theta * derivative + Ki_theta * theta_integral;
    // Serial.print(error); Serial.print(",");
    // Serial.print(Kp_theta * error); Serial.print(",");       // P term
    // Serial.print(Ki_theta * theta_integral); Serial.print(","); // I term
    // Serial.print(Kd_theta * derivative); Serial.print(",");  // D term
    // Serial.print(theta_output);
    // Serial.print("  ");

    // Motor Commands
    double steering_output = ang_vel_cmd;
    double left_omega = theta_output + steering_output;
    double right_omega = theta_output - steering_output;
    left_omega = constrain(left_omega, -5.56, 5.56);
    right_omega = constrain(right_omega, -5.56, 5.56);
    motor_driver_set_omega(left_omega, right_omega);
  } else {
    // If no IMU data available this loop, you may choose to skip control or use last known values
    // motor_driver_set_omega(0,0); // optional fallback
  }

  freq = (1000.0 / (millis() - loop_start_time + 1)); // +1 avoids division by zero
  static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 100) {
  //   lastPrint = millis();
  //   Serial.print("loop_dt: ");  Serial.print(loop_dt);
  //   Serial.print(" us, servo_dt: "); Serial.print(servo_dt);
  //   Serial.print(" us, imu_dt: ");   Serial.println(imu_dt);
  // }
  // // Serial.println("");
  // small delay so loop doesn't become a busy-wait (tune as needed)
  vTaskDelay(1 / portTICK_PERIOD_MS);
}

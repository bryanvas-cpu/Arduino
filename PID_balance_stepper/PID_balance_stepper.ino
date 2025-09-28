#include "stepper_driver_MKS_TMC2160-OC.h"
// #include "dualshock.h"
#include "servo_ik.h"
#include "imu_sensor.h"
#include <Wire.h>


unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // in milliseconds
double body_height_command=330;
/////////////////////////////////
struct STATE{
  double x_dot;
  double theta;
  double theta_dot;
  double alpha;
  double alpha_dot;
};

STATE state;

///////////////////////////////////////////////////////////////// --- PID gains for theta ---
float Kp_theta = 2.0;   // proportional gain (tune)12
float Kd_theta = 0.0;    // derivative gain (tune)0.85
float Ki_theta = 0;    // optional integral2
float theta_integral = 0.0;
double theta_filtered = 0.0;
float alpha_theta = 0.1;  // smoothing for pitch angle
float alpha_theta_dot = 0.1;

///////////////////////////////////////////////////////////////// --- PID gains for velocity ---
float Kp_vel = 0.0;   // tune0.2
float Ki_vel = 0.0;   // tune0.08
float Kd_vel = 0.00000;   // tune0.00000001
double vel_integral = 0.0;
double vel_prev_error = 0.0;

double freq = 750;

void setup() {
  Wire.begin(21, 22, 200000);
  Serial.begin(115200);
  // setupJOY();
  setupServoIK();
  setupIMU();
  delay(1000);
  motor_driver_init(); 

  xTaskCreatePinnedToCore(
    ServoTask,         // Task function
    "ServoTask",       // Name
    4096,              // Stack size in bytes
    NULL,              // Task parameters
    1,                 // Priority (1 low, 5 high)
    NULL,              // Task handle
    1                  // Core 1
  );
}
////////////////////////////////////////////////////////////////////////shared variables
float roll_cmd_global = 0;
float body_height_global = 330;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;

void ServoTask(void *pvParameters) {
  while (true) {
    // Copy shared variables safely
    float roll, height;
    portENTER_CRITICAL(&servoMux);
    roll = roll_cmd_global;
    height = body_height_global;
    portEXIT_CRITICAL(&servoMux);

    ServoIK(roll, height);  // call your existing function

    vTaskDelay(10 / portTICK_PERIOD_MS); // 100 Hz update rate
  }
}
////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  double loop_start_time = millis();

///////////////////////////////////////////////////////////////////////////////// Servo
  portENTER_CRITICAL(&servoMux);
  roll_cmd_global = 0;       // update as needed
  body_height_global = 330;
  portEXIT_CRITICAL(&servoMux);
///////////////////////////////////////////////////////////////////////////////// Acquire IMU Data
  IMUData imuData = updateIMU();

  double accel = imuData.linearAccX;     // m/s^2

  double omega_gyro = -imuData.angularVelZ;

// Serial.print(imuData.linearAccX); Serial.print(",");
// Serial.print(imuData.roll);       Serial.print(",");
// Serial.print(imuData.pitch);      Serial.print(",");
// Serial.print(imuData.yaw);        Serial.print(",");
// Serial.print(imuData.angularVelX); Serial.print(",");
// Serial.print(imuData.angularVelY); Serial.print(",");
// Serial.println(imuData.angularVelZ);

///////////////////////////////////////////////////////////////////////////////// Set Target Linear and Angular Velocities
  double lin_vel_cmd = 0;
  double ang_vel_cmd = 0;
  if(lin_vel_cmd >= -1 && lin_vel_cmd <= 1){
    lin_vel_cmd = 0;
  }
  if(ang_vel_cmd >= -1 && ang_vel_cmd <= 1){
    ang_vel_cmd = 0;
  }
  
// //////////////////////////////////////////////////////////////////////////////// Velocity PID
  double vel_error = lin_vel_cmd - drive_vel.lin_vel;
  vel_integral += vel_error * (1.0/freq);   // dt ~0.01s
  vel_integral = constrain(vel_integral, -1, 1); // anti-windup
  double vel_derivative = (vel_error - vel_prev_error) / (1.0/freq);
  vel_prev_error = vel_error;
  double vel_pid_output = Kp_vel * vel_error
                        + Ki_vel * vel_integral 
                        + Kd_vel * vel_derivative;

// /////////////////////////////////////////////////////////////////////////////// Target Pitch
  double target_pitch = vel_pid_output;
  target_pitch = constrain(target_pitch, -3.0, 3.0); 

// /////////////////////////////////////////////////////////////////////////////// Angle PID
  state.theta = alpha_theta * state.theta + (1-alpha_theta) * imuData.pitch;
  state.theta_dot = alpha_theta_dot * state.theta_dot + (1-alpha_theta_dot) * imuData.angularVelY;
  double error = target_pitch - state.theta;
  theta_integral += error * ( 1.0 / freq); // dt=0.01s approx
  double derivative = -state.theta_dot;
  if(theta_integral >= 0.5 ) theta_integral = 0.5;
  if(theta_integral <= -0.5) theta_integral =-0.5;
  double theta_output = Kp_theta * error + Kd_theta * derivative + Ki_theta * theta_integral;
  Serial.print(error); Serial.print(",");
  Serial.print(Kp_theta * error); Serial.print(",");       // P term
  Serial.print(Ki_theta * theta_integral); Serial.print(","); // I term
  Serial.print(Kd_theta * derivative); Serial.print(",");  // D term
  Serial.println(theta_output);

// /////////////////////////////////////////////////////////////////////////////// Motor Commands
  double steering_output = ang_vel_cmd; // tune scaling
  double left_omega = theta_output + steering_output;
  double right_omega = theta_output - steering_output;
  left_omega = constrain(left_omega, -5.56, 5.56);
  right_omega = constrain(right_omega, -5.56, 5.56);
  // --- Send to motors ---
  motor_driver_set_omega(left_omega, right_omega);

  freq = (1000/(millis()-loop_start_time));
  // Serial.println(String(freq));
}
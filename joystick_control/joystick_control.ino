#include "motor_driver.h"
#include "dualshock.h"
#include "servo_ik.h"
#include "imu_sensor.h"

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // in milliseconds
double body_height_command=300;

JOY_CMD joy_cmd = {-1, body_height_cmd, 0.0, 0.0};

////////////////////KALMAN FILTER
double x_est = 0.0;      // Estimated velocity
double P = 1.0;
double Q = 0.05;
double R = 0.05;

double omega_est = 0.0;
double P_omega = 1.0;
double Q_omega = 0.05;
double R_omega = 0.05;
/////////////////////////////////
struct STATE{
  double x_dot;
  double theta;
  double theta_dot;
  double alpha;
  double alpha_dot;
};

STATE state;

////////////////////////////////// L Q R ///////////////////////////////
// const int A_ROWS = 5;
// const int A_COLS = 5;
// const int B_ROWS = 5;
// const int B_COLS = 2;
// // A matrix (5x5)
// float A_matrix[A_ROWS][A_COLS] = {
//   {11.99131562,  3.04897,     0.0, 0.0, 0.0},
//   {0.0,          0.0,         1.0, 0.0, 0.0},
//   {27.58690981, 30.1293672,   0.0, 0.0, 0.0},
//   {0.0,          0.0,         0.0, 0.0, 1.0},
//   {0.0,          0.0,         0.0, 0.0, 0.0}
// };

// // B matrix (5x2)
// float B_matrix[B_ROWS][B_COLS] = {
//   {-1.19913156, -1.19913156},
//   {0.0,          0.0},
//   {-2.75869098, -2.75869098},
//   {0.0,          0.0},
//   {7.39649932,  -7.39649932}
// };
// // Define matrix dimensions
// const int Q_ROWS = 5;
// const int Q_COLS = 5;
// const int R_ROWS = 2;
// const int R_COLS = 2;

// // Q matrix (5x5) - process noise covariance
// float Q_matrix[Q_ROWS][Q_COLS] = {
//   {0.001, 0.0,   0.0,    0.0,    0.0},
//   {0.0,   10.0,  0.0,    0.0,    0.0},
//   {0.0,   0.0,   10.0,   0.0,    0.0},
//   {0.0,   0.0,   0.0,    0.1,  0.0},
//   {0.0,   0.0,   0.0,    0.0,    0.1}
// };

// // R matrix (2x2) - measurement noise covariance
// float R_matrix[R_ROWS][R_COLS] = {
//   {10.0, 0.0},
//   {0.0,  10.0}
// };

// float k_matrix[2][5] = {
//   { 4.99975001e-04, -4.90493009e+01, -1.05304700e-01,  7.07106781e-02,  1.19176370e-01 },
//   { 4.99975001e-04, -4.90493009e+01, -1.05304700e-01, -7.07106781e-02, -1.19176370e-01 }
// };

// float Kp = 0.02;
// float Ki = 0.001;
// float Kd = 0.02;
// float Kp_a = 1.0;
// float Ki_a = 0.001;
// float Kd_a = 0.02;

// float dt = 0.08; 

// float target_vel = 0.0;
// float target_ang_vel = 0.0;
// float integral = 0.0;
// float integral_ang = 0.0;
// float prev_error = 0.0;
// float prev_ang_err = 0.0;

// float u_t[2];

// void callback(float vel, float theta, float thetadot, float alpha, float alphadot, float t_vel, float t_ang_vel) {
//   theta = radians(theta);
//   target_vel = t_vel;
//   target_ang_vel = t_ang_vel;

//   float error_lin = -(target_vel - vel);
//   float error_ang = -(target_ang_vel - alphadot);

//   integral += error_lin * dt;
//   integral_ang += error_ang * dt;

//   float derivative = (error_lin - prev_error) / dt;
//   float derivative_ang = (error_ang - prev_ang_err) / dt;

//   float pid_tilt_com_correction = Kp * error_lin + Kd * derivative + Ki * integral;
//   float pid_yaw_com_correction = Kp_a * error_ang + Kd_a * derivative_ang + Ki_a * integral_ang;
//   // float pid_tilt_com_correction = error_lin;
//   // float pid_yaw_com_correction = error_ang*0.1;
//   // Serial.print(pid_tilt_com_correction);
//   // Serial.print('\t');
//   // Serial.println(pid_yaw_com_correction);
//   theta = theta - pid_tilt_com_correction;
//   // Serial.println(theta);
//   thetadot = thetadot;
//   alpha = 0.0 - pid_yaw_com_correction;
//   alphadot = alphadot;

//   float np_x[5] = {vel, theta, thetadot, alpha, alphadot};
 
//   // String output = "np_x: ";
//   // for (int i = 0; i < 5; i++) {
//   //   output += String(np_x[i], 4); // 4 decimal places
//   //   if (i < 4) output += ", ";
//   // }
//   // Serial.println(output);

//   for (int i = 0; i < 2; i++) {
//     u_t[i] = 0.0;
//     for (int j = 0; j < 5; j++) {
//       u_t[i] -= k_matrix[i][j] * np_x[j];
//     }
//   }
//   prev_error = error_lin;
//   prev_ang_err = error_ang;
// }
//////////////////////////////////////////////////////LQR END/////////////////////

void kalmanUpdate(double accel, double dt, double v_enc) {
  x_est = x_est + accel * dt;
  P = P + Q;
  double K = P / (P + R);
  x_est = x_est + K * (v_enc - x_est);
  P = (1 - K) * P;
}
void kalmanUpdateAngular(double gyro_omega, double dt, double omega_enc) {
  omega_est = gyro_omega;
  P_omega = P_omega + Q_omega;
  double K = P_omega / (P_omega + R_omega);
  omega_est = omega_est + K * (omega_enc - omega_est);
  P_omega = (1 - K) * P_omega;
}

void setup() {
  Serial.begin(115200);
  setupJOY();
  motor_driver_setup();  
  setupServoIK();
  delay(1000);
  setupIMU();
}

void loop() {
  double loop_start_time = millis();
  // JOYSTICK
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;
    joy_cmd = UpdateJoyloop();
    // Serial.println(String(joy_cmd.on_off_state) + " " + 
    //                String(joy_cmd.body_height_cmd) + " " + 
    //                String(joy_cmd.left_joy_cmd) + " " + 
    //                String(joy_cmd.right_joy_cmd));
  }
  if(joy_cmd.on_off_state == -1){
    motor_driver_loop(0.0,0.0);
    ServoIK(0, joy_cmd.body_height_cmd);
    delay(10);
    return;
  }
  // KALMAN
  IMUData imuData = updateIMU();
  DriveVel drive_vel = query_vel();

  double lin_vel_cmd = -joy_cmd.left_joy_cmd;
  double ang_vel_cmd = joy_cmd.right_joy_cmd;

  if(lin_vel_cmd >= -5 && lin_vel_cmd <= 5){
    lin_vel_cmd = 0;
  }
  if(ang_vel_cmd >= -5 && ang_vel_cmd <= 5){
    ang_vel_cmd = 0;
  }
  lin_vel_cmd /= 512; // 0-> +-1
  ang_vel_cmd /= 256; // 0-> +-2
  // Serial.println(String(lin_vel_cmd) + " " + (String(ang_vel_cmd)));


  static unsigned long lastKFTime = millis();
  unsigned long now = millis();
  double dt_lin = (now - lastKFTime) / 1000.0;
  lastKFTime = now;
  static unsigned long lastTimeOmega = millis();
  unsigned long currentTimeOmega = millis();
  double dt_omega = (currentTimeOmega - lastTimeOmega) / 1000.0;
  lastTimeOmega = currentTimeOmega;

  double accel = imuData.linearAccX;     // m/s^2
  double v_enc = drive_vel.lin_vel;
  double omega_enc = drive_vel.ang_vel;      // from drivetrain
  double omega_gyro = -imuData.angularVelZ; 

  // kalmanUpdate(accel, dt_lin, v_enc);
  // kalmanUpdateAngular(omega_gyro, dt_omega, omega_enc);

  // state.x_dot = x_est;
  state.theta = imuData.pitch;
  state.theta_dot = imuData.angularVelY;
  // state.alpha = imuData.yaw;
  // state.alpha_dot = omega_est;

  // callback(state.x_dot, state.theta,state.theta_dot, state.alpha, state.alpha_dot, lin_vel_cmd, ang_vel_cmd);

// double scaled_left = u_t[0] * 20;
// double scaled_right = u_t[1] * 30;

// double offset_left = (scaled_left >= 0) ? 50 : -50;
// double offset_right = (scaled_right >= 0) ? 50 : -50;

// double final_left = constrain(scaled_left + offset_left, -255, 255);
// double final_right = constrain(scaled_right + offset_right, -255, 255);
double final_left = (lin_vel_cmd + (L*ang_vel_cmd/2))*255;
double final_right = (lin_vel_cmd - (L*ang_vel_cmd/2))*255;

// Serial.print(final_left, 4);
// Serial.print(" ");
// Serial.println(final_right, 4);

ServoIK(0, joy_cmd.body_height_cmd);
// Serial.println(joy_cmd.body_height_cmd);
Serial.print("theta ");
Serial.print(state.theta);
Serial.print("theta_dot ");
Serial.println(state.theta_dot);

// MOTORS DRIVER
motor_driver_loop(final_left, final_right);


  
  delay(10);
  double freq = (1000/(millis()-loop_start_time));
  // Serial.println("                                                     "+String(freq));
}
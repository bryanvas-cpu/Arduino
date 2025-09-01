#include "motor_driver_cytron_senpai.h"
#include "dualshock.h"
#include "servo_ik.h"
#include "imu_sensor.h"

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // in microseconds
double body_height_command=330;

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

// --- PID gains for balancing ---
float Kp_bal = 15;   // proportional gain (tune)12
// float Kd_bal = -150.0;    // derivative gain (tune)
float Kd_bal = 1.35;    // derivative gain (tune)0.85
float Ki_bal = 2;    // optional integral2

float balance_integral = 0.0;

float Kp_vel = 0.2;   // tune0.2
float Ki_vel = 0.08;   // tune0.08
float Kd_vel = 0.000001;   // tune0.00000001
double vel_integral = 0.0;
double vel_prev_error = 0.0;

double freq = 84;

// --- Exponential filters ---
double v_enc_filtered = 0.0;
double theta_filtered = 0.0;

float alpha_vel = 0.8;    // smoothing for velocity
float alpha_theta = 0.6;  // smoothing for pitch angle


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
  // setupJOY();
  motor_driver_setup();  
  // setupServoIK();
  delay(500);
  // setupIMU();
}

void loop() {
  unsigned long loop_start_time = micros();
  delay(1);
  static double left_setpoint = 0;
  static double right_setpoint = 0;

  // --- Check if new serial input is available ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // read a line
    input.trim(); // remove spaces or newline

    // Expect input format: "L_RPM R_RPM"
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      double left_rpm_cmd = input.substring(0, spaceIndex).toFloat();
      double right_rpm_cmd = input.substring(spaceIndex + 1).toFloat();
      Serial.print(left_rpm_cmd);
      Serial.print(" ");
      Serial.print(right_rpm_cmd);
      Serial.print(" ");

      // Convert RPM → rad/s for PID loop
      left_setpoint = left_rpm_cmd * (2.0 * M_PI / 60.0);
      right_setpoint = right_rpm_cmd * (2.0 * M_PI / 60.0);
    }
  }

  // --- Update measured wheel velocities and robot velocities ---
  DriveVel drive_vel = query_vel();  

  // --- Run PID control ---
  motor_driver_loop(left_setpoint, right_setpoint);

  // --- Convert measured rad/s → RPM ---
  double left_rpm_meas = omega_l * 60.0 / (2.0 * M_PI);
  double right_rpm_meas = omega_r * 60.0 / (2.0 * M_PI);

  // --- Print for Serial Plotter (tab-separated) ---
  Serial.print("   ");
  Serial.print(left_rpm_meas);
  Serial.print("\t");
  Serial.print(right_rpm_meas);
  Serial.print("\t");
  Serial.print(drive_vel.lin_vel);   // m/s
  Serial.print("\t");
  Serial.print(drive_vel.ang_vel); // rad/s
  // delay(1000);
  // --- Optional: control loop frequency estimation ---
  double freq = 1000000.0 / (micros() - loop_start_time + 1);
  Serial.print("               freq: ");
  Serial.println(freq);
}



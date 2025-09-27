#include "HardwareSerial.h"
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// Motor driver pins

#define r_pwm 2
#define r_dir 4
#define l_pwm 18
#define l_dir 5

#define r_enc_a 33
#define r_enc_b 32
#define l_enc_a 12
#define l_enc_b 13

const int pwmResolution = 8;
const int pwmFreq = 5000;
const int pwmMax = 255;

volatile int right_encoder_counter = 0;
volatile int left_encoder_counter = 0;
volatile double right_encoder_smoothed = 0.0;
volatile double left_encoder_smoothed = 0.0;

double omega_r = 0.0, omega_l = 0.0;
const double counts_per_rev = 134.0;

long prev_right_ticks = 0;
long prev_left_ticks = 0;

double command = 0;
String inputString = "";
bool stringComplete = false;

float r = 0.145;
float L = 0.41;

double kp = 5, ki = 0.0, kd = 0.0;  // Tune these values
double integral_l = 0, integral_r = 0;
double prev_error_l = 0, prev_error_r = 0;

double loop_frequency = 84; // hz
double loop_time = 1.0/loop_frequency; // sec

struct DriveVel{
    float lin_vel;
    float ang_vel;
};

const double alpha_motor = 0.1;

void IRAM_ATTR right_encoder_callback() {
  int dir = digitalRead(r_enc_b) == HIGH ? 1 : -1;
  right_encoder_counter += dir;
}

void IRAM_ATTR left_encoder_callback() {
  int dir = digitalRead(l_enc_b) == HIGH ? 1 : -1;
  left_encoder_counter += dir;
}

void right_motor_cmd(double command) {
  int pwm = constrain((int)abs(command), 0, pwmMax);
  digitalWrite(r_dir, command >= 0 ? HIGH : LOW); // Direction
  ledcWrite(0, pwm); // Speed
  Serial.print(" r_pwm: ");
  // Serial.print(r_dir);
  Serial.print(command);
}

void left_motor_cmd(double command) {
  int pwm = constrain((int)abs(command), 0, pwmMax);
  digitalWrite(l_dir, command >= 0 ? HIGH : LOW); // Direction
  ledcWrite(1, pwm); // Speed
  Serial.print(" l_pwm: ");
  // Serial.print(l_dir);
  Serial.print(command);
}

void motor_driver_setup() {
  

  pinMode(r_pwm, OUTPUT);
  pinMode(r_dir, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(l_dir, OUTPUT);

  pinMode(r_enc_b, INPUT); pinMode(l_enc_b, INPUT);

  attachInterrupt(digitalPinToInterrupt(r_enc_a), right_encoder_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(l_enc_a), left_encoder_callback, RISING);

  ledcSetup(0, pwmFreq, pwmResolution);
  ledcAttachPin(r_pwm, 0);

  ledcSetup(1, pwmFreq, pwmResolution);
  ledcAttachPin(l_pwm, 1);
}

double computePID(double setpoint, double measured, double &integral, double &prev_error) {
    double error = setpoint - measured;
    integral += error * loop_time;
    integral = constrain(integral, -50.0, 50.0);
    double derivative = (error - prev_error) * loop_frequency;
    double output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    Serial.print(" error ");
    Serial.print(kp * error);
    Serial.print(" derivative ");
    Serial.print(kd * derivative);
    Serial.print(" integral ");
    Serial.print(ki * integral);
    return output;
}

// ---- Modified Motor Driver Loop ----
// cmd_left and cmd_right are *desired wheel angular velocities* in rad/s
void motor_driver_loop(double cmd_left, double cmd_right) {
    // PID control for left motor
    double control_l = computePID(cmd_left, omega_l, integral_l, prev_error_l);
    left_motor_cmd(control_l);

    // PID control for right motor
    double control_r = computePID(cmd_right, omega_r, integral_r, prev_error_r);
    right_motor_cmd(control_r);
}

DriveVel query_vel(){
    long right_ticks = right_encoder_counter;
    long left_ticks  = left_encoder_counter;

    // --- compute delta ticks ---
    long delta_r = right_ticks - prev_right_ticks;
    long delta_l = left_ticks - prev_left_ticks;

    // --- compute angular velocity [rad/s] ---
    double omega_r_raw = (2.0 * M_PI * delta_r) / (counts_per_rev * loop_time);
    double omega_l_raw = (2.0 * M_PI * delta_l) / (counts_per_rev * loop_time);

    omega_r = alpha_motor * omega_r + (1.0 - alpha_motor) * omega_r_raw;
    omega_l = alpha_motor * omega_l + (1.0 - alpha_motor) * omega_l_raw;

    // --- store state ---
    prev_right_ticks = right_ticks;
    prev_left_ticks  = left_ticks;

    // --- compute linear & angular velocity ---
    float lin_vel = (r / 2.0) * (omega_l + omega_r);
    float ang_vel = (r / L) * (omega_r - omega_l);

    return {lin_vel, ang_vel};
}


#endif


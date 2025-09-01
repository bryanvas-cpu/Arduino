#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU9250_asukiaaa.h>
// #include "cmath"

#define USMIN 500   // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400  // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50
#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds. We will reset the data if no signal

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
//////////////////////////////ROLL-CONTROL////////////////////////
double body_roll=0;

double left_leg_length=200;
double right_leg_length=200;
double min_leg_length = 200;
double max_leg_length = 300;

double body_height = 300;
double base_width = 0.3523;

double kp = 5;
double kd = 0.01;

double l1 = 112.5, l2=225.0, l3=75, l4=150, l5=75, d=75;
double joint_angles[6];

double dummy_roll_cmd=0;
double roll_cmd;
int flag=1;

double current_roll;
double current_pitch;
//////////////////////////////////////MPU9250////////////////////////
MPU9250_asukiaaa mySensor;

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

float gx, gy, gz;
float ax, ay, az;
//////////////////////////////////CODE////////////////////////////////
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

void get_roll_pitch(){
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
  current_roll  = atan2(ay, -az) * 180.0 / PI;                        // X-axis tilt
  current_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;   // Y-axis tilt
}

double get_recommended_roll;


int inverse_kinematics(double xe, double ye, int left_leg_param) {

  if(left_leg_param){
    Serial.println("Left_leg--------------------------");
  }
  else{
    Serial.println("--------------------------Right_leg");
  }
  xe = -xe;
  double L = sqrtf(xe*xe + ye*ye);
  double J3 = acos(   ((l1 * l1) + (l2 * l2) - (L * L))   /   (2 * l1 * l2)   ) * (180 / PI);
  double B = acos(   ((L * L) + (l1 * l1) - (l2 * l2))   /   (2 * L * l1)   ) * (180 / PI);
  double A = atan(xe / ye) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
  double J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'
  joint_angles[0] = 90+J2;
  joint_angles[1] = J3 + J2 -90;
  double xa = l1*cos(joint_angles[0] * 3.1415/180.0) + l5*cos(joint_angles[1] * 3.1415/180.0 );
  double ya = l1*sin(joint_angles[0] * 3.1415/180.0) + l5*sin(joint_angles[1]* 3.1415/180.0 );
  // Serial.print(" xa,ya "+ String(xa)+", " +String(ya) + " || ");


  xe = xa-d;
  ye = ya;

  L = sqrtf(xe*xe + ye*ye);
  J3 = acos(   ((l3 * l2) + (l4 * l4) - (L * L))   /   (2 * l3 * l4)   ) * (180 / PI);
  B = acos(   ((L * L) + (l3 * l3) - (l4 * l4))   /   (2 * L * l3)   ) * (180 / PI);
  A = atan(xe / ye) * (180 / PI);  // BECAUSE Z REST IS NEGATIVE, THIS RETURNS A NEGATIVE VALUE
  J2 = (B + A);  // BECAUSE 'A' IS NEGATIVE AT REST WE NEED TO INVERT '-' TO '+'

  joint_angles[2] = 90-J2;
  double k = asin(l3 * sin(J3) /L);
  joint_angles[3] = A + k +90;

  if(left_leg_param = 1){
    joint_angles[5] = joint_angles[0] - 90; // rear joint servo command
    joint_angles[4] = joint_angles[2] - 45; // front joint servo comand
  }
  else{
    joint_angles[5] = 180 - joint_angles[0]; // rear joint servo command
    joint_angles[4] = 135 - joint_angles[2]; // front joint servo comand
  }
  

  // NaN check for joint angles
  for (int i = 0; i < 4; ++i) {
    if (isnan(joint_angles[i])) {
      return 0;  // Invalid solution
    }
  }
  for (int i = 4; i < 6; ++i) {
    if (joint_angles[i] < 0 || joint_angles[i] > 90) {
      return 0;  // Invalid solution
    }
  }
  return 1;  // Valid solution
}

void setup(){
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  Wire.begin(); // SDA = GPIO21, SCL = GPIO22 on ESP32
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag(); // even if not working, harmless

  delay(100);

  calibrateGyro();
  delay(10);
}

void loop() {

  get_roll_pitch(); // current_roll, current_pitch have been acquired



  double recommended_roll = 0;

  recommended_roll = radians(recommended_roll);   // Convert to radians
  roll_cmd = recommended_roll; // get roll_cmd from pid
  Serial.println("roll_cmd: " + String(roll_cmd));
  if(recommended_roll < 0){
    double left_leg_length_redn = base_width*tan(-roll_cmd);
    left_leg_length = fmin(fmax(min_leg_length,(body_height-left_leg_length_redn*1000)),body_height);
    int result = inverse_kinematics(37.5, left_leg_length, 1);
    // Serial.println("Result: "+ String(result));
    if(result){
      int us_1 = map(joint_angles[4]-5.7, 0,90,USMIN,USMAX);
      int us_2 = map(joint_angles[5]-8.6, 0,90,USMIN,USMAX);
      pwm.writeMicroseconds(0, us_1);
      pwm.writeMicroseconds(1, us_2);
      Serial.println("Joint_angles_left: " + String(joint_angles[4]) + ", "+ joint_angles[5]);
    }

    right_leg_length = body_height;
    result = inverse_kinematics(37.5, right_leg_length, 0);
    // Serial.println("Result: "+ String(result));
    if(result){
      int us_1 = map(joint_angles[4], 0,90,USMIN,USMAX);
      int us_2 = map(joint_angles[5], 0,90,USMIN,USMAX);
      pwm.writeMicroseconds(2, us_1);
      pwm.writeMicroseconds(3, us_2);
      Serial.println("Joint_angles_right: " + String(joint_angles[4]) + ", "+ joint_angles[5]);
    }
  }
  else{
    
    left_leg_length = body_height;
    int result = inverse_kinematics(37.5, left_leg_length, 1);
    // Serial.println("Result: "+ String(result));
    if(result){
      int us_1 = map(joint_angles[4]-5.7, 0,90,USMIN,USMAX);
      int us_2 = map(joint_angles[5]-8.6, 0,90,USMIN,USMAX);
      pwm.writeMicroseconds(0, us_1);
      pwm.writeMicroseconds(1, us_2);
      Serial.println("Joint_angles_left: " + String(joint_angles[4]) + ", "+ joint_angles[5]);
    }

    double right_leg_length_redn = base_width*tan(roll_cmd);
    right_leg_length = fmin(fmax(min_leg_length,(body_height-right_leg_length_redn*1000)),body_height);
    result = inverse_kinematics(37.5, right_leg_length, 0);
    // Serial.println("Result: "+ String(result));
    if(result){
      int us_1 = map(joint_angles[4], 0,90,USMIN,USMAX);
      int us_2 = map(joint_angles[5], 0,90,USMIN,USMAX);
      pwm.writeMicroseconds(2, us_1);
      pwm.writeMicroseconds(3, us_2);
      Serial.println("Joint_angles_right: " + String(joint_angles[4]) + ", "+ joint_angles[5]);
    }
  }

  
  delay(1);
}
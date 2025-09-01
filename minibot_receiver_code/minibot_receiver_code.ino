#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Servo.h>

#define SERVO_FREQ 50
#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds. We will reset the data if no signal

const uint64_t pipeIn = 0xF9E8F0F0E2LL;
unsigned long lastRecvTime = 0, newTime = 0;

RF24 radio(7, 8);
Servo servo1, servo2;

float dir,speed,x_coordinate,y_coordinate;
int rightWheelSpeed,leftWheelSpeed, rightWheelSpeed_p, leftWheelSpeed_p;

struct PacketData {
  byte max_step_height;
  byte max_step_length;
  byte default_roll;
  byte default_pitch;
  byte x_coordinate;
  byte y_coordinate;
  byte default_body_height_up;
  byte default_body_height_down;
  byte on_off;
  byte gait;
};
PacketData receiverData;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set_variable_values() {
  x_coordinate = (float)(receiverData.x_coordinate*1.00);
  y_coordinate = (float)(receiverData.y_coordinate*1.00);
  if(x_coordinate > 120 && x_coordinate < 130){
    x_coordinate = 127;
  }
  if(y_coordinate > 120 && y_coordinate < 130){
    y_coordinate = 127;
  }


  // Map joystick input to direction (turn) and speed
  dir = (x_coordinate - 127) * 90.0 / 127;  // Centered around 0, range -90 to 90
  speed = (y_coordinate - 127) * 90.0 / 127; // Centered around 0, range -90 to 90

  // Calculate wheel speeds, centered at 90
  leftWheelSpeed = 90 + speed + dir;  // Turn left decreases left speed
  rightWheelSpeed = 180-(90 + speed - dir); // Turn right decreases right speed

  leftWheelSpeed = leftWheelSpeed * 0.1 + leftWheelSpeed_p *0.9;
  rightWheelSpeed = rightWheelSpeed * 0.1 + rightWheelSpeed_p * 0.9;

  leftWheelSpeed_p = leftWheelSpeed;
  rightWheelSpeed_p = rightWheelSpeed;
  // Constrain wheel speeds to the valid range (0 to 180)
  if (leftWheelSpeed > 180)
      leftWheelSpeed = 180;
  if (leftWheelSpeed < 0)
      leftWheelSpeed = 0;

  if (rightWheelSpeed > 180)
      rightWheelSpeed = 180;
  if (rightWheelSpeed < 0)
      rightWheelSpeed = 0;

}

void set_variable_default_values(){
  dir = 90;
  speed = 90;
}

void execute_main_code() {

  // Serial.print(" x:");
  // Serial.print(x_coordinate);
  // Serial.print(" y:");
  // Serial.print(y_coordinate);      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Serial.print(" speed1:");
  // Serial.print(dir);
  // Serial.print(" speed2:");
  // Serial.print(speed);
  // Serial.print(" lws:");
  // Serial.print(leftWheelSpeed);
  // Serial.print(" rws:");
  // Serial.println(rightWheelSpeed);

  servo1.write(leftWheelSpeed);
  servo2.write(rightWheelSpeed);
}


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();  //start the radio receiver
  set_variable_default_values();
  delay(10);
  servo1.attach(6);
  servo2.attach(9);
}

void loop() {
  // Check if RF is connected and packet is available
  if (radio.isChipConnected() && radio.available()) {
    radio.read(&receiverData, sizeof(PacketData));
    lastRecvTime = millis();
    set_variable_values();
  } else {
    //Check Signal lost.
    unsigned long now = millis();
    if (now - lastRecvTime > SIGNAL_TIMEOUT) {
      set_variable_default_values();
    }
  }


  execute_main_code();

  ///////////////////////////////////////////////////////////////////////////////////
}
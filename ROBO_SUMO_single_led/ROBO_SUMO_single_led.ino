#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <math.h>
// #include <Servo.h>

#define LED_COUNT 16
#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds. We will reset the data if no signal
// int Llpwm = 10, Lrpwm = 6, Rlpwm = 5, Rrpwm = 3,Wpwm=9;
int Llpwm = 10, Lrpwm = 6, Rlpwm = 5, Rrpwm = 9;
int flag = 1,newButtonVal,oldButtonVal=1,newButtonVal2,oldButtonVal2,flag2=1,mode=1;
int i=0;
float theta,x,y;
int leftWheelSpeed, rightWheelSpeed;
int speed,dir;
const uint64_t pipeIn = 0xF9E8F0F0E1LL;
unsigned long lastRecvTime = 0,newTime=0,oldTime=0,ledOldTime,ledNewTime;

unsigned long time_loop_start, time_loop;

RF24 radio(7, 8); 
// Servo weapon_motor;

struct PacketData 
{
  byte drive_speed;
  byte drive_dir;
  byte weapon_speed;
  byte button_value;  
  byte button2_value;
  byte mode;
};
PacketData receiverData;

void do_led(int color, int led_pin=1){

  if(color == 1){
    // r - d3
    digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    digitalWrite(4,LOW);
  }
  else if(color == 2){
    // g - d4
    digitalWrite(2,LOW);
    digitalWrite(3,LOW);
    digitalWrite(4,HIGH);
  }
  else if(color ==3){
    // b - d2
    digitalWrite(2,HIGH);
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
  }
  else{
    // y - d3, d4
    digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
  }
  // digitalWrite(2,LOW);
  // digitalWrite(3,LOW);
  // digitalWrite(4,LOW);
}

int convert(byte x){
  int distance;
  distance = (int)(0.0000111022*x*x*x-0.0051327263*x*x+0.8043316558*x+4.8826759773);
  if(distance<0)
    distance = 0;
  else if(distance>60)
    distance = 60;
  return distance-30;

}

int convert2(byte x){
  int distance;
  distance = (int)(0.0000000019*x*x*x*x*x-0.0000011172*x*x*x*x+0.0002334634*x*x*x-0.0201869309*x*x+0.7546246965*x-0.0004368998);
  if(distance<0)
    distance = 0;
  else if(distance>60)
    distance = 60;
  return distance-30;

}

//Assign default input received values
void setActualValues(){
  if(receiverData.mode==1){
    speed = 0;
    dir = 0;
    if(ledNewTime - ledOldTime >30){
      do_led(2,flag2); // green
      ledOldTime=ledNewTime;
    }

  }
  else if(receiverData.mode==2){
    speed = convert(receiverData.drive_speed);
    dir = convert2(receiverData.weapon_speed);
    if(ledNewTime - ledOldTime >30){
      do_led(4,flag2);
      ledOldTime=ledNewTime;
    }
  }
  else if(receiverData.mode==3){
    speed = convert(receiverData.drive_speed);
    dir = convert2(receiverData.weapon_speed);
    if(ledNewTime - ledOldTime >30){
      do_led(1,flag2); // red
      ledOldTime=ledNewTime;
    }
  }
  else{
    if(ledNewTime - ledOldTime >30){
      do_led(3,flag2); // blue
      ledOldTime=ledNewTime;
    }
  }
}

void setInputDefaultValues(){
  // The middle position for joystick. (254/2=127)
  speed = 0;
  dir  = 0;
}

void mapAndWriteValues(){

}

void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,pipeIn);
  radio.startListening(); //start the radio receiver      
  setInputDefaultValues();
  mapAndWriteValues();
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(Llpwm,OUTPUT);
  pinMode(Rlpwm,OUTPUT);
  pinMode(Lrpwm,OUTPUT);
  pinMode(Rrpwm,OUTPUT);
}

void loop()
{
  time_loop_start = millis();
  // Check if RF is connected and packet is available 
  if(radio.isChipConnected() && radio.available())
  {
    radio.read(&receiverData, sizeof(PacketData)); 
    setActualValues();
    lastRecvTime = millis(); 
  }
  else
  {
    //Check Signal lost.
    unsigned long now = millis();
    if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
    {
      setInputDefaultValues();
      do_led(3);
    }
  }

  ledNewTime = millis();
///////////////////////////////////////////////////button1///////////////////////////
  newButtonVal=receiverData.button_value;
  if(newButtonVal == 0 && oldButtonVal == 1)
  {
    flag*=-1;
  }
  oldButtonVal=newButtonVal;


  // if(flag == 1)
  // {
  // setActualValues();
  //   digitalWrite(3,HIGH);
  // }
  // else if(flag == -1)
  // {
  //   setInputDefaultValues();
  //   digitalWrite(3,LOW);
  //
///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////button2///////////////////////////////////
  newButtonVal2=receiverData.button2_value;
  if(newButtonVal2 == 0 && oldButtonVal2 == 1)
  {
    flag2*=-1;
  }
  oldButtonVal2=newButtonVal2;


  if(flag2 == 1)
  {
    // digitalWrite(4,HIGH);
    digitalWrite(5,LOW);
  }
  else if(flag2 == -1)
  {
    speed*=-1;
    // digitalWrite(4,LOW);
    digitalWrite(5,HIGH);
  }
///////////////////////////////////////////////////////////////////////////////////
////////////////////////////////mode///////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
  newTime = millis();
  if(radio.available() > 0)
  {
    oldTime = newTime;
  }

  // if(newTime - oldTime > 1000)
  // {
  //   setInputDefaultValues();
  //   do_led(3);
  // }
  dir=fabsf(dir)*tan(3.1415*dir/120);
  speed=fabsf(speed)*tan(3.1415*speed/120);
  x=dir/30.0;
  y=speed/30.0;
  y=0.8*y;
  x=1.0*x;
  // leftWheelSpeed = ((y*(y/sqrt(x*x+y*y))  +  x*(x/sqrt(x*x+y*y)))  /  sqrt(x*x+y*y))  * 255.0;
  // rightWheelSpeed =((y*(y/sqrt(x*x+y*y))  -  x*(x/sqrt(x*x+y*y)))  /  sqrt(x*x+y*y))  * 255.0;

  leftWheelSpeed = (( y* fabsf(y/sqrt(x*x + y*y)))  +  ( x* fabsf(x/sqrt(x*x + y*y))))*220;
  rightWheelSpeed = (( y* fabsf(y/sqrt(x*x + y*y)))  -  ( x* fabsf(x/sqrt(x*x + y*y))))*220;
  if(leftWheelSpeed>255)
    leftWheelSpeed = 255;
  if(leftWheelSpeed < (-255))
    leftWheelSpeed = -255;
  if(rightWheelSpeed>255)
    rightWheelSpeed = 255;
  if(rightWheelSpeed < (-255))
    rightWheelSpeed = -255;
    
  if(leftWheelSpeed>=0){
    analogWrite(Llpwm,(int)leftWheelSpeed);
    analogWrite(Lrpwm,0);
  }
  else{
    analogWrite(Lrpwm,(int)(-1*leftWheelSpeed));
    analogWrite(Llpwm,0);
  }

  if(rightWheelSpeed>=0){
    analogWrite(Rlpwm,(int)rightWheelSpeed);
    analogWrite(Rrpwm,0);
  }
  else{
    analogWrite(Rrpwm,(int)(-1*rightWheelSpeed));
    analogWrite(Rlpwm,0);
  }

  Serial.print(speed);
  Serial.print(" ");
  Serial.print(dir);
  Serial.print(" ");
  Serial.print(flag);
  Serial.print(" ");
  Serial.print(flag2);
  Serial.print(" ");
  Serial.print(receiverData.button_value);
  Serial.print(" ");  
  Serial.print(receiverData.button2_value);    
  Serial.print(" ");
  Serial.print(leftWheelSpeed);
  Serial.print(" ");  
  Serial.print(rightWheelSpeed);    
  Serial.print(" ");
 
  Serial.println(receiverData.mode);
  
}
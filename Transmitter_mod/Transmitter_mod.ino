
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint64_t pipeOut = 0xF9E8F0F0E1LL;   //IMPORTANT: The same as in the receiver 0xF9E8F0F0E1LL
int onButton = 5;
int flipSwitch =  6;
int redLed = 2;
int yellowLed = 3;
int greenLed = 4;
int switch_1 = A7;
int switch_2 = A6;

RF24 radio(8, 9); // select CE,CSN pin

struct PacketData 
{
  byte drive_speed;
  byte drive_dir;
  byte weapon_speed;
  byte button_value;  
  byte button2_value;
  byte mode;
};
PacketData data;

void mapAndWriteValues()
{

}

void setup()
{
  Serial.begin(115200);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.stopListening(); //start the radio Transmitter 
  pinMode(switch_1,INPUT);
  pinMode(switch_2,INPUT);
  pinMode(onButton,INPUT_PULLUP);
  pinMode(flipSwitch,INPUT_PULLUP);   
  pinMode(redLed,OUTPUT);
  pinMode(greenLed,OUTPUT);
  pinMode(yellowLed,OUTPUT);
}

//This function is used to map 0-1023 joystick value to 0-254. hence 127 is the center value which we send.
//It also adjust the deadband in joystick.
//Jotstick values range from 0-1023. But its center value is not always 511. It is little different.
//So we need to add some deadband to center value. in our case 500-530. Any value in this deadband range is mapped to center 127.
int mapAndAdjustJoystickValues(int value)
{
  value = (map(value, 0.0, 1023.0, 0, 255));
  mapAndWriteValues();
  return value;
}

void loop()
{
  if(analogRead(switch_1)>100 && analogRead(switch_2)>100){
    digitalWrite(greenLed,HIGH);
    digitalWrite(redLed,LOW);
    digitalWrite(yellowLed,LOW);
    data.mode=1;
  }
  else if(analogRead(switch_1)<100 && analogRead(switch_2)<100){
    digitalWrite(greenLed,LOW);
    digitalWrite(redLed,HIGH);
    digitalWrite(yellowLed,LOW);
    data.mode = 3;
  }
  else if((analogRead(switch_1)<100 && analogRead(switch_2>100)) || (analogRead(switch_1)>100 && analogRead(switch_2<100))){
    digitalWrite(greenLed,LOW);
    digitalWrite(redLed,LOW);
    digitalWrite(yellowLed,HIGH);
    data.mode=2;
  }
  
  data.drive_speed    = mapAndAdjustJoystickValues(analogRead(A2))-256;
  data.drive_dir    = 255-mapAndAdjustJoystickValues(analogRead(A1));
  data.weapon_speed   = 255-mapAndAdjustJoystickValues(analogRead(A0));
  data.button_value    = digitalRead(onButton);
  data.button2_value    =digitalRead(flipSwitch);

  Serial.print(data.drive_speed);
  Serial.print("  ");
  Serial.print(data.drive_dir);
  Serial.print("  ");
  Serial.print(data.weapon_speed);
  Serial.print("  ");
  Serial.print(data.button_value);
  Serial.print("  ");
  Serial.print(data.button2_value);
  Serial.print("  ");
  Serial.print(data.mode);
  Serial.print("  ");
  Serial.print(analogRead(switch_1));
  Serial.print("  "); 
  Serial.println(analogRead(switch_2));
  radio.write(&data, sizeof(PacketData));
}
#include <SCServo.h>
SMS_STS st;
void setup(){
Serial1.begin(1000000); //Initialize the serial port, if you use ESP32 and other devices, you can also choose a custom serial port
// Serial1.begin(1000000, SERIAL_8N1, RX, TX); // custom serial port
st.pSerial = &Serial1;
while(!Serial1) {}
}
void loop(){}

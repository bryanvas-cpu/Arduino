#include <SCServo.h>

SMS_STS st;

// the UART used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

u8 S_ID = 11;

void setup()
{
  Serial.begin(115200); // Initialize Serial for user input
  Serial.println("Enter servo angle (0-360):");
  
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD); // Initialize servo UART
  st.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  if (Serial.available()) {
    float desiredAngle = Serial.parseFloat(); // Read the angle input from the Serial Monitor
    if(desiredAngle == 500.0)
    {
      st.CalibrationOfs(S_ID);
    }
    else{
      if (desiredAngle >= 0 && desiredAngle <= 360) {
        moveServoToAngle(S_ID, desiredAngle, 3400, 50); // Move servo to the specified angle
        Serial.print("Moving servo to angle: ");
        Serial.println(desiredAngle);
      } else {
        Serial.println("Invalid angle! Please enter a value between 0 and 360.");
      }
    }
    
    delay(200); // Prevent serial flooding
  }
}

void moveServoToAngle(u8 id, float angle, u16 speed, u8 acc)
{
  // Convert angle (0° to 360°) to position (0 to 4095)
  u16 position = (angle / 360.0) * 4095;
  st.WritePosEx(id, position, speed, acc); // Move servo to the calculated position
}

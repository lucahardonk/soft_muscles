#include "AS5600.h"
#include <RobojaxBTS7960.h>

AS5600 as5600;  // Use default Wire
#define AS_DIR 4

#define RPWM 5 // define pin 3 for RPWM pin (output)
#define R_EN 6 // define pin 2 for R_EN pin (input)
#define R_IS 7 // define pin 5 for R_IS pin (output)

#define LPWM 8 // define pin 6 for LPWM pin (output)
#define L_EN 9 // define pin 7 for L_EN pin (input)
#define L_IS 10 // define pin 8 for L_IS pin (output)

#define CW 1 //do not change
#define CCW 0 //do not change
#define debug false //change to 0 to hide serial monitor debugging infornmation or set to 1 to view

#define relay0 A0
#define relay1 A1
#define relay2 A2


RobojaxBTS7960 motor(R_EN,RPWM,R_IS, L_EN,LPWM,L_IS,debug);


unsigned long previousMillis = 0;

void setup() {
  motor.begin();

  pinMode(relay0, OUTPUT); 
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  //turn them all off with high -> inverse logic
  digitalWrite(relay0, HIGH);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);

  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();

  as5600.begin(AS_DIR);                         // Set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // Default, just be explicit.

  Serial.println(as5600.getAddress());

  int as_status = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(as_status);

  delay(100);

  testMotorScript();
  digitalWrite(relay0, HIGH);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  delay(500);
  digitalWrite(relay0, LOW);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  delay(500);
  digitalWrite(relay0, HIGH);
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, HIGH);
  delay(500);
  digitalWrite(relay0, HIGH);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, LOW);
  delay(500);
}

void loop() {

  /*
  // Just to show how reset can be used
  if (as5600.getRevolutions() >= 10)
  {
    as5600.resetPosition();
  }*/
}

void testMotorScript() {

  for (int a = 0; a < 2000; a++) {
    motor.rotate(a/20,CCW);
    delay(1);
    printPosition();
  }
  // Stop the motor
 
    motor.stop();
    printPosition();
    delay(1000);
    
  
  // Reverse rotation
  for (int a = 0; a < 2000; a++) {
    motor.rotate(a/20,CW);
    delay(1);
    printPosition();
  }
  // Stop the motor
 
    motor.stop();
    printPosition();
    delay(1000);

    Serial.println("===================================");
    
  
}


void printPosition(){

  Serial.print(as5600.getCumulativePosition());
  Serial.print("\t");
  Serial.println(as5600.getRevolutions());
}



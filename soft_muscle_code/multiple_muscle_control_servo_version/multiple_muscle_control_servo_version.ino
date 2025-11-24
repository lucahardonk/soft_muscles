#include "PneumaticSystem.h"
//---------------------------------------------------->changes to organize
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Default I2C address for the PCA9685 is 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Adjust these values to match your servo's pulse range
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERVOMUSCLE0 0
#define SERVOMUSCLE1 1

int pulselen = 0;
int angle = 0;
int last_angle  = -1; // Initialize with an invalid angle to ensure the first command is executed
//----------------------------------------------------------------------------------------------------------->
PressureSensor sensorePressione1(30,PRESSURE_SENSOR_1);
Motor motor(RPWM_PIN, R_EN_PIN, R_IS_PIN, LPWM_PIN, L_EN_PIN, L_IS_PIN);
Piston piston(0, -4500, A_AIR_PIN, A_OUT_PIN, B_AIR_PIN, B_OUT_PIN);
Tank tank(TANKVALVE);
Muscle polsoSX(MUSCLE1);
Muscle polsoDX(MUSCLE2);


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Starting setup...");

  Wire.begin();
  motor.begin();
  piston.begin();
  tank.begin();
  polsoSX.begin();
  polsoDX.begin();
  sensorePressione1.begin(); 
  //as5600.begin(AS_DIR);                    // Set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // Default, just be explicit.
  //as5600StatusCheck();
  as5600.resetPosition();
  as5600.resetCumulativePosition();
  currentPosition = as5600.getCumulativePosition();
  printTimer.attach_us(printISR, 15000);  // Set timer to call printISR every 15 milliseconds
  pinMode(PRESSURE_SENSOR_1, INPUT);

  //<----------------->
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // Set the oscillator frequency for the PCA9685
  pwm.setPWMFreq(SERVO_FREQ);  // Set the PWM frequency to 50 Hz (analog servos)

  delay(10);  // Wait for the PCA9685 to be ready
  //------------------------------------>
  
}

int percentagePosition = 0;
String command;
float lastMediaMobilePress1 = 0; 
void loop() {
  if (Serial.available() > 0) {                     // Check if there is any data available to read
    command = Serial.readStringUntil('\n');  // Read the input as a string until a newline character
    command.trim();                                 // Trim any leading and trailing whitespace
    command.toUpperCase();                          // Convert the input string to lowercase

    if (command == "OPEN_A_AIR") {
      piston.openValve(A_AIR_PIN);
      Serial.println("ACK");
    } else if (command == "CLOSE_A_AIR") {
      piston.closeValve(A_AIR_PIN);
      Serial.println("ACK");
    } else if (command == "OPEN_A_OUT") {
      piston.openValve(A_OUT_PIN);  // Open valve A_OUT
      Serial.println("ACK");
    } else if (command == "CLOSE_A_OUT") {
      piston.closeValve(A_OUT_PIN);  // Close valve A_OUT
      Serial.println("ACK");
    } else if (command == "OPEN_B_AIR") {
      piston.openValve(B_AIR_PIN);  // Open valve B_AIR
      Serial.println("ACK");
    } else if (command == "CLOSE_B_AIR") {
      piston.closeValve(B_AIR_PIN);  // Close valve B_AIR
      Serial.println("ACK");
    } else if (command == "OPEN_B_OUT") {
      piston.openValve(B_OUT_PIN);  // Open valve B_OUT
      Serial.println("ACK");
    } else if (command == "CLOSE_B_OUT") {
      piston.closeValve(B_OUT_PIN);  // Close valve B_OUT
      Serial.println("ACK");
    } else if (command.startsWith("MOVE_MOTOR:")) {
      int position = command.substring(11).toInt();
      percentagePosition = position;
      //Serial.println(position);
      Serial.println("ACK");
    } 
    //------------------------------------------------------------>
    else if (command.startsWith("SERVOMUSCLE0:")) {
      int angle = command.substring(13).toInt();
      Serial.println(angle);
      int pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(0, 0, pulselen);
      Serial.println("ACK");
    }else if (command.startsWith("SERVOMUSCLE1:")) {
      int angle = command.substring(13).toInt();
      Serial.println(angle);
      int pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(1, 0, pulselen);
      Serial.println("ACK");
    }
    //----------------------------------------------------------------------__>
    else if (command == "PRESSURE1") {
      Serial.println(String(lastMediaMobilePress1) + ",ACK");
    }
    else {
      Serial.println("error: command not found!");
    }
  }
  piston.movePiston(percentagePosition);
  
  lastMediaMobilePress1 = sensorePressione1.filtro_media_mobile();

}


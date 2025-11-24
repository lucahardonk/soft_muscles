#include "PneumaticSystem.h"

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
    } else if (command == "PRESSURE1") {
      Serial.println(String(lastMediaMobilePress1) + ",ACK");
    } else {
      Serial.println("error: command not found!");
    }
  }
  piston.movePiston(percentagePosition);
  
  lastMediaMobilePress1 = sensorePressione1.filtro_media_mobile();

}


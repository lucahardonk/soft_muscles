#include "AS5600.h"
#include <mbed.h>


/*
note importanti:
  questo sketch use un pid controller per spostare la siringa al valore assoluto desiderato, è un po brusco perchè deve contrastare l'accumolo di pressione.
  i valori registrati di per spostare del tutto la siringa sono da -5200 a siriga totalmente ritratta a 0 quando totalmente spinta.
  -5200 ------------------ 0
    |                      |
    |                      |
    |                      |
  siringa totalemte     siringa totalemte
  ritratta              contratta
    |                      |
  muscolo vuoto         muscolo contratto
  

  ATTENZIONE: essendo un test del pid e del funzionamento non sono stai implementati controlli fuorirange, quindi se non si vuoe disfare tutto, 
  ricontrollare che i vali immessi da raggiungere siano quelli giusti, pena distruzione degli ingnaggi o della siringa.

*/
mbed::Ticker printTimer;
bool printFlag = true;

AS5600 as5600;  // Use default Wire
#define AS_DIR 4

#define RPWM 5  // define pin 3 for RPWM pin (output)
#define R_EN 6  // define pin 2 for R_EN pin (input)
#define R_IS 7  // define pin 5 for R_IS pin (output)

#define LPWM 8   // define pin 6 for LPWM pin (output)
#define L_EN 9   // define pin 7 for L_EN pin (input)
#define L_IS 10  // define pin 8 for L_IS pin (output)


#define CW 0   // Clockwise
#define CCW 1  // Counterclockwise

#define relay0 A0
#define relay1 A1
#define relay2 A2

unsigned long previousMillis = 0;


// my log cotrol parameters
long setpoint = 0;
long currentPosition = 0;
int logConstant = 3;
int argumentMultiply = 500;
float threshold = 100;




void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup...");

  // motor setup
  pinMode(RPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_IS, INPUT);

  pinMode(LPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_IS, INPUT);

  // Enable the motor driver
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Initial motor state
  stopMotor();

  Wire.begin();

  pinMode(relay0, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  //turn them all off with high -> inverse logic
  digitalWrite(relay0, HIGH);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);

  as5600.begin(AS_DIR);                    // Set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // Default, just be explicit.
  Serial.println(as5600.getAddress());

  int as_status = as5600.isConnected();

  Serial.print("Connect: ");
  Serial.println(as_status);

  if (as_status) {
    Serial.println("AS5600 is connected.");
  } else {
    Serial.println("AS5600 is not connected.");
  }
  as5600StatusCheck();

  as5600.resetPosition();
  as5600.resetCumulativePosition();
  currentPosition = as5600.getCumulativePosition();

  printTimer.attach_us(printISR, 15000); // Set timer to call printISR every 1000 milliseconds (1 second)
}

void loop() {
  static unsigned long lastMoveTime = 0;
  static int moveIndex = 0;

  unsigned long currentMillis = millis();

  if (currentMillis - lastMoveTime >= 2000) {
    lastMoveTime = currentMillis;

    switch (moveIndex) {
      case 0:
        setpoint = 0;
        break;
      case 1:
        setpoint = -1000;
        break;
      case 2:
        setpoint = -4500;
        break;
      case 3:
        setpoint = -300;
        break;
    }

    moveIndex = (moveIndex + 1) % 4;
    Serial.println("==========================");
    Serial.println("index at: " + String(moveIndex));
  }
  long input = as5600.getCumulativePosition();

  int output = movePiston(setpoint, input, logConstant, argumentMultiply, threshold);
  // Check if the print flag is set by the interrupt
  if (printFlag) {
    printFlag = false;  // Reset the flag
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(input);
    Serial.print(",");
    Serial.println(output);
  }

}

void printISR() {
  printFlag = true;
}

void as5600StatusCheck() {
  long cumulativePosition = as5600.getCumulativePosition();
  int revolutions = as5600.getRevolutions();
  int position = as5600.readAngle();
  int rawAngle = as5600.rawAngle();
  int magnitude = as5600.readMagnitude();
  bool magnetDetected = as5600.detectMagnet();

  Serial.print("Cumulative Position: ");
  Serial.println(cumulativePosition);
  Serial.print("Revolutions: ");
  Serial.println(revolutions);
  Serial.print("Position: ");
  Serial.println(position);
  Serial.print("Raw Angle: ");
  Serial.println(rawAngle);
  Serial.print("Magnitude: ");
  Serial.println(magnitude);
  Serial.print("Magnet Detected: ");
  Serial.println(magnetDetected ? "Yes" : "No");
}

void runMotor(int direction, int speed) {
  if (direction == CW) {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  } else if (direction == CCW) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, speed);
  }
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}


int movePiston(long setpoint, long currentPosition, int logConstant, int argumentMultiply, float threshold){
  long error = setpoint - currentPosition; // Calculate error
  bool dir  = error > 0 ? CCW : CW; // Define motor direction based on the error
  error = abs(error); // Get absolute error
  int out = 255; // Default output value

  if(error > threshold){
    // Move with maximum speed if current position is less than setpoint * threshold
    runMotor(dir, out); 
  } else {
    // Decelerate using a logarithmic curve as it approaches the setpoint
    out = constrain(logConstant * log(error * argumentMultiply), 0, 255);
    runMotor(dir, out); 
  }
  return out; // Return the output value as an int
}
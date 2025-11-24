#include "PneumaticSystem.h"

// Global Variables Initialization
long setpoint = 0;
long currentPosition = 0;
int logConstant = 3;
int argumentMultiply = 500;
float threshold = 100;
bool printFlag = true;
mbed::Ticker printTimer;
AS5600 as5600;

// PressureSensor Class Implementation
PressureSensor::PressureSensor(int sizeMedia, int sensor_pin) : fsize(sizeMedia), i(0), pin(sensor_pin) {
    fil = new float[fsize]();
}

PressureSensor::~PressureSensor() {
    delete[] fil;
}

void PressureSensor::begin() {
    pinMode(pin, INPUT);
}

float PressureSensor::filtro_media_mobile() {
    float avg = 0.0;
    float newValue = analogRead(pin);
    fil[i] = newValue;
    i = (i + 1) % fsize;
    for (int j = 0; j < fsize; j++) {
        avg += fil[j];
    }
    avg /= fsize;
    return avg;
}

float PressureSensor::readSensor() {
    return analogRead(pin);
}

// Motor Class Implementation
Motor::Motor(uint8_t RPWM, uint8_t R_EN, uint8_t R_IS, uint8_t LPWM, uint8_t L_EN, uint8_t L_IS): RPWM(RPWM), R_EN(R_EN), R_IS(R_IS), LPWM(LPWM), L_EN(L_EN), L_IS(L_IS) {}

void Motor::begin() {
    pinMode(RPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(R_IS, INPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(L_IS, INPUT);
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);
    stopMotor();
}

void Motor::runMotor(int direction, int speed) {
    if (direction == CW) {
        analogWrite(RPWM, speed);
        analogWrite(LPWM, 0);
    } else if (direction == CCW) {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, speed);
    }
}

void Motor::stopMotor() {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
}

int Motor::movePiston(long setpoint, long currentPosition, int logConstant, int argumentMultiply, float threshold) {
    long error = setpoint - currentPosition;
    bool dir = error > 0 ? CCW : CW;
    error = abs(error);
    int defaultSpeed = 250;
    if (error > threshold) {
        runMotor(dir, defaultSpeed);
    } else {
        defaultSpeed = constrain(logConstant * log(error * argumentMultiply), 0, 255);
        runMotor(dir, defaultSpeed);
    }
    return defaultSpeed;
}

// Piston Class Implementation
Piston::Piston(long zero, long max, uint8_t A_AIR, uint8_t A_OUT, uint8_t B_AIR, uint8_t B_OUT)
    : zeroPos(zero), maxPos(max), A_AIR(A_AIR), A_OUT(A_OUT), B_AIR(B_AIR), B_OUT(B_OUT) {}

void Piston::begin() {
    pinMode(A_AIR, OUTPUT);
    pinMode(A_OUT, OUTPUT);
    pinMode(B_AIR, OUTPUT);
    pinMode(B_OUT, OUTPUT);
    openValve(A_AIR);
    openValve(A_OUT);
    openValve(B_AIR);
    openValve(B_OUT);
}

void Piston::movePiston(long percentage) {
    long conversion = map(percentage, 0, 100, zeroPos, maxPos);
    motor.movePiston(conversion, as5600.getCumulativePosition(), logConstant, argumentMultiply, threshold);
}

void Piston::openValve(int valve) {
    digitalWrite(valve, HIGH);
}

void Piston::closeValve(int valve) {
    digitalWrite(valve, LOW);
}

long Piston::getPostion() {
    return as5600.getCumulativePosition();
}

// Tank Class Implementation
Tank::Tank(int pin) : pin(pin) {}

void Tank::begin() {
    pinMode(pin, OUTPUT);
    closeValve(pin);
}

void Tank::openValve(int valve) {
    digitalWrite(valve, HIGH);
}

void Tank::closeValve(int valve) {
    digitalWrite(valve, LOW);
}

// Muscle Class Implementation
Muscle::Muscle(int pin) : pin(pin) {}

void Muscle::begin() {
    pinMode(pin, OUTPUT);
    openValve(pin);
}

void Muscle::openValve(int valve) {
    digitalWrite(valve, HIGH);
}

void Muscle::closeValve(int valve) {
    digitalWrite(valve, LOW);
}

void printISR() {
    printFlag = true;
}

void as5600StatusCheck() {
    Serial.println(as5600.getAddress());

    int as_status = as5600.isConnected();
    Serial.print("Connect: ");
    Serial.println(as_status);

    if (as_status) {
        Serial.println("AS5600 is connected.");
    } else {
        Serial.println("AS5600 is not connected.");
    }

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

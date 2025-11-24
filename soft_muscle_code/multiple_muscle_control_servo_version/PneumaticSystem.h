#ifndef PNEUMATICSYSTEM_H
#define PNEUMATICSYSTEM_H

#include "AS5600.h"
#include <mbed.h>

// Pin Definitions
#define MUSCLE1 2
#define MUSCLE2 3
#define MUSCLE3 11
#define AS_DIR 4
#define RPWM_PIN 5
#define R_EN_PIN 6
#define R_IS_PIN 7
#define LPWM_PIN 8
#define L_EN_PIN 9
#define L_IS_PIN 10
#define A_AIR_PIN A0
#define A_OUT_PIN A1
#define B_AIR_PIN A2
#define B_OUT_PIN A3
#define TANKVALVE 12
#define PRESSURE_SENSOR_1 A6

// Direction Definitions
#define CW 0
#define CCW 1

// Global Variables
extern long setpoint;
extern long currentPosition;
extern int logConstant;
extern int argumentMultiply;
extern float threshold;
extern bool printFlag;
extern mbed::Ticker printTimer;
extern AS5600 as5600;

// PressureSensor Class
class PressureSensor {
private:
    int fsize;
    float* fil;
    int i;
    int pin;
public:
    PressureSensor(int sizeMedia, int sensor_pin);
    ~PressureSensor();
    void begin();
    float filtro_media_mobile();
    float readSensor();
};

// Motor Class
class Motor {
private:
    uint8_t RPWM;
    uint8_t R_EN;
    uint8_t R_IS;
    uint8_t LPWM;
    uint8_t L_EN;
    uint8_t L_IS;
public:
    Motor(uint8_t RPWM, uint8_t R_EN, uint8_t R_IS, uint8_t LPWM, uint8_t L_EN, uint8_t L_IS);
    void begin();
    void runMotor(int direction, int speed);
    void stopMotor();
    int movePiston(long setpoint, long currentPosition, int logConstant, int argumentMultiply, float threshold);
};

// Piston Class
class Piston {
private:
    long zeroPos;
    long maxPos;
    uint8_t A_AIR;
    uint8_t A_OUT;
    uint8_t B_AIR;
    uint8_t B_OUT;
public:
    Piston(long zero, long max, uint8_t A_AIR, uint8_t A_OUT, uint8_t B_AIR, uint8_t B_OUT);
    void begin();
    void movePiston(long percentage);
    void openValve(int valve);
    void closeValve(int valve);
    long getPostion();
};

// Tank Class
class Tank {
private:
    int pin;
public:
    Tank(int pin);
    void begin();
    void openValve(int valve);
    void closeValve(int valve);
};

// Muscle Class
class Muscle {
private:
    int pin;
public:
    Muscle(int pin);
    void begin();
    void openValve(int valve);
    void closeValve(int valve);
};


//objects pre declaration

extern PressureSensor sensorePressione1;
extern Motor motor;
extern Piston piston;
extern Tank tank;
extern Muscle polsoSX;
extern Muscle polsoDX;


// Function Declarations

void printISR();
void as5600StatusCheck();

#endif  // PNEUMATICSYSTEM_H

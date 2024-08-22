#ifndef _DC_MOTOR_CONTROL_H_
#define _DC_MOTOR_CONTROL_H_

#include <Arduino.h>

// Pin definitions for motor control
#define pwmFrequency 800 // Frequency in Hertz
#define pwmResolution 8  // 8-bit resolution (values from 0 to 255)

// Definitions for motor control
#define STRAIGHT 1
#define SLOW 2
#define LEFT 3
#define RIGHT 4
#define TURN_LEFT 5
#define TURN_RIGHT 6
#define STANDBY 7
#define STOP 8

class DCMotorControl {
  public:
    DCMotorControl(uint8_t pinPWNMotorA, uint8_t pinPWNMotorB);

    void CarMovementControl(uint8_t direction, uint8_t speed, int8_t alpha);
    void SettingMotor(uint8_t speedMotorA, uint8_t speedMotorB);

  private:
    uint8_t pinPWNMotorA;
    uint8_t pinPWNMotorB;
};

#endif

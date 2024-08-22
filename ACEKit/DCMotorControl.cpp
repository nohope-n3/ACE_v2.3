#include "DCMotorControl.h"
#include <Arduino.h>

DCMotorControl::DCMotorControl(uint8_t pinPWNMotorA, uint8_t pinPWNMotorB) {
    this->pinPWNMotorA = pinPWNMotorA;
    this->pinPWNMotorB = pinPWNMotorB;

    ledcAttachChannel(pinPWNMotorA, pwmFrequency, pwmResolution, 0);
    ledcAttachChannel(pinPWNMotorB, pwmFrequency, pwmResolution, 1);
}

void DCMotorControl::SettingMotor(uint8_t speedMotorA, uint8_t speedMotorB) {
    // Seting for motor A
    ledcWrite(pinPWNMotorA, speedMotorA);
    // Seting for motor B
    ledcWrite(pinPWNMotorB, speedMotorB);
}

void DCMotorControl::CarMovementControl(uint8_t direction, uint8_t speed, int8_t alpha) {
    switch (direction) {
    case STRAIGHT:
        SettingMotor(speed, speed - alpha);
        break;
    case SLOW:
        SettingMotor(speed - alpha, speed - alpha);
        break;
    case LEFT:
        SettingMotor(speed - alpha, speed);
        break;
    case RIGHT:
        SettingMotor(speed, speed - alpha);
        break;

    case TURN_LEFT:
        // Forward
        SettingMotor(0, 0);
        delay(100);
        SettingMotor(speed, speed);
        delay(500);
        // Turn
        SettingMotor(0, speed);
        delay(alpha * 100);
        // Forward
        SettingMotor(speed, speed);
        delay(200);
        break;
    case TURN_RIGHT:
        // Forward
        SettingMotor(0, 0);
        delay(100);
        SettingMotor(speed, speed);
        delay(500);
        // Turn
        SettingMotor(speed, 0);
        delay(alpha * 100);
        // Forward
        SettingMotor(speed, speed);
        delay(200);
        break;

    case STANDBY:
        SettingMotor(speed, speed);
        delay(alpha * 100);
        break;

    default:
        // Stop car
        SettingMotor(0, 0);
        break;
    }
}

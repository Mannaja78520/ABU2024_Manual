#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <DefinePin.h>

Motor motor1(20000, 10, 0, 1, motor1PWM, motor1PinA, motor1PinB);
Motor motor2(20000, 10, 0, 1, motor2PWM, motor2PinA, motor2PinB);
Motor motor3(20000, 10, 0, 1, motor3PWM, motor3PinA, motor3PinB);

// Motor motor1(0, 1, motor1PWM, motor1PinA, motor1PinB);
// Motor motor2(0, 1, motor2PWM, motor2PinA, motor2PinB);
// Motor motor3(0, 1, motor3PWM, motor3PinA, motor3PinB);

class Movement
{
public:
  void MovePower(int motor1Power, int motor2Power, int motor3Power)
  {
    motor1.spin(motor1Power);
    motor2.spin(motor2Power);
    motor3.spin(motor3Power);
  }
};

#endif
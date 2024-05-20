#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <Controller.h>

#include <Movement.h>
#include <Harvest.h>
#include <Ball.h>
#include <Communicate.h>
#include <Utilize.h>

DataFromZigbeeJoystickXbox gamepad;
// DataFromZigbeeJoystickPlayStation gamepad;

class Robot
{
  Movement m;
  Harvest h;
  Ball b;

public:
  void init()
  {
    Serial.begin(115200);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    h.init();
    b.init();

    delay(200);
    Serial.println("Ready!");
  }

  void MovePower(int motor1Speed, int motor2Speed, int motor3Speed)
  {
    m.MovePower(motor1Speed, motor2Speed, motor3Speed);
  }

  void BallSpin(int SpinSpeed)
  {
    b.BallSpin(SpinSpeed);
  }

  void loop()
  {
  }
};

#endif
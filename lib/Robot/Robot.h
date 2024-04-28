#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>

#include <Movement.h>
#include <Harvest.h>
#include <Ball.h>
#include <RobotAnalysis.h>
#include <Communicate.h>
#include <Utilize.h>

DataFromZigbeeJoystickXbox gamepad;
TransferData t;
// DataFromZigbeeJoystickPlayStation gamepad;

class Robot{
  Movement m;
  Harvest h;
  Ball b;
  RobotAnalysis ra;

  public:
    void init(){
      Serial.begin(115200);
      ra.init();
      
      h.init();
      b.init();
      
      gamepad.init();
      delay(200);
      Serial.println ("Ready!");
    }

    void MovePower(int motor1Speed, int motor2Speed, int motor3Speed){
      m.MovePower(motor1Speed, motor2Speed, motor3Speed);
    }
    
    void BallSpin(int SpinSpeed){
      b.BallSpin(SpinSpeed);
    }

    void loop(){
      ra.loop();
      gamepad.readData();
    }
};

#endif
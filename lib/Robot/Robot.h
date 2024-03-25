#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>

class Robot{
  public:
    void init(){
      Grip0.attach(Servo0);
      // Grip1.attach(Servo1);
      // Grip2.attach(Servo2);
      // Grip3.attach(Servo3);

      Grip0.write(0);
      // Grip1.write(0);
      // Grip2.write(0);
      // Grip3.write(0);

      pinMode(GripUp, OUTPUT);
      // pinMode(Solenoid1, OUTPUT);
      // pinMode(Solenoid2, OUTPUT);
      // pinMode(Solenoid3, OUTPUT);
    }
};

#endif
#ifndef HARVEST_H
#define HARVEST_H

#include <DefinePin.h> 

Servo Grip1;
Servo Grip2;
Servo Grip3;
Servo Grip4;

class Harvest{
    public:
        void init(){
            pinMode(GripUP, OUTPUT);
            pinMode(GripSlide, OUTPUT);
            
            digitalWrite(GripUP, HIGH);
            digitalWrite(GripSlide, HIGH);

            Grip1.attach(Servo1);
            Grip2.attach(Servo4);
            Grip3.attach(Servo3);
            Grip4.attach(Servo2);
            
            Grip1.write(3);
            Grip2.write(3);
            Grip3.write(3);   
            Grip4.write(3);
        }   
};

#endif
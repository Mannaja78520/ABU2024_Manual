#ifndef BALL_H
#define BALL_H

#include <DefinePin.h>
#include <Utilize.h>

Motor Spin(0, 0, motor4PWM, motor4PinA, motor4PinB);
// Motor Spin2(0, motor5PWM, motor5PinA, motor5PinB);

// Motor Spin1(20000, 8, 1, motor4PWM, motor4PinA, motor4PinB);
// Motor Spin2(20000, 8, 0, motor5PWM, motor5PinA, motor5PinB);

Servo BallUP_DOWN;
Servo BallLeftGrip;
Servo BallRightGrip;

class Ball{
    int nowSpin = 0;
    public:
        void init(){
            pinMode(KickBall, OUTPUT);
            digitalWrite(KickBall, HIGH);

            BallUP_DOWN.attach(Servo5);
            BallLeftGrip.attach(Servo6);
            BallRightGrip.attach(Servo7);
            
            BallUP_DOWN.write(0);
            BallLeftGrip.write(0);
            BallRightGrip.write(180);   
        }
        void BallSpin(int SpinSpeed){
            Spin.spin(SpinSpeed);
        }
};

#endif
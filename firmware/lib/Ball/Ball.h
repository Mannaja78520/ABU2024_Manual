#ifndef BALL_H
#define BALL_H

#include <DefinePin.h>
#include <Utilize.h>

// Motor Spin(0, 0, motor4PWM, motor4PinA, motor4PinB);

Motor Spin(20000, 8, 0, 1, motor4PWM, motor4PinA, motor4PinB);

Servo BallUP_DOWN;
Servo BallLeftGrip;
Servo BallRightGrip;

class Ball
{
    int nowSpin = 0;

public:
    void init()
    {
        pinMode(KickBall, OUTPUT);
        digitalWrite(KickBall, HIGH);

        BallUP_DOWN.setPeriodHertz(50);
        BallLeftGrip.setPeriodHertz(50);
        BallRightGrip.setPeriodHertz(50);

        BallUP_DOWN.attach(Servo5);
        BallLeftGrip.attach(Servo6);
        BallRightGrip.attach(Servo7);

        BallUP_DOWN.write(180);
        BallLeftGrip.write(95);
        BallRightGrip.write(91);
    }
    void BallSpin(int SpinSpeed)
    {
        Spin.spin(SpinSpeed);
    }
};

#endif
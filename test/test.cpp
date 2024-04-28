#include <Robot.h>
// #include <Arduino.h>
// #include <Servo.h>

int grip1Val = 0, grip2Val = 0, grip3Val = 0, grip4Val = 0, 
UpDownVal = 180, LeftVal = 0, RightVal = 180;

Robot r;

void setup(){
    r.init();
}

void loop(){
    r.loop();
    // r.BallSpin(255);

    // Grip1 is servo3
    // Grip2 is servo2
    // Grip3 is servo1
    // Grip4 is servo4

    // grip1Val = gamepad.Dpad_right ? grip1Val + 1 : grip1Val;
    // grip1Val = gamepad.Dpad_left ? grip1Val -1 :grip1Val;
    // grip1Val = grip1Val < 0 ? 0 : grip1Val > 180 ? 180 : grip1Val;
    // Grip1.write(grip1Val);
    // Serial.print("Grip1 : ");
    // Serial.println(grip1Val);
    
    // grip2Val = gamepad.Dpad_right ? grip2Val + 1 : grip2Val;
    // grip2Val = gamepad.Dpad_left ? grip2Val -1 :grip2Val;
    // grip2Val = grip2Val < 0 ? 0 : grip2Val > 180 ? 180 : grip2Val;
    // Grip2.write(grip2Val);
    // Serial.print("Grip2 : ");
    // Serial.println(grip2Val);

    // grip3Val = gamepad.Dpad_right ? grip3Val + 1 : grip3Val;
    // grip3Val = gamepad.Dpad_left ? grip3Val -1 :grip3Val;
    // grip3Val = grip3Val < 0 ? 0 : grip3Val > 180 ? 180 : grip3Val;
    // Grip3.write(grip3Val);
    // Serial.print("Grip3 : ");
    // Serial.println(grip3Val);

    // grip4Val = gamepad.Dpad_right ? grip4Val + 1 : grip4Val;
    // grip4Val = gamepad.Dpad_left ? grip4Val -1 :grip4Val;
    // grip4Val = grip4Val < 0 ? 0 : grip4Val > 180 ? 180 : grip4Val;
    // Grip4.write(grip4Val);
    // Serial.print("Grip4 : ");
    // Serial.println(grip4Val);

    // UpDownVal = gamepad.Dpad_right ? UpDownVal + 1 : UpDownVal;
    // UpDownVal = gamepad.Dpad_left ? UpDownVal -1 :UpDownVal;
    // UpDownVal = UpDownVal < 0 ? 0 : UpDownVal > 180 ? 180 : UpDownVal;
    // BallUP_DOWN.write(UpDownVal);
    // Serial.print("UpDownVal : ");
    // Serial.println(UpDownVal);

    // LeftVal = gamepad.Dpad_right ? LeftVal + 1 : LeftVal;
    // LeftVal = gamepad.Dpad_left ? LeftVal -1 :LeftVal;
    // LeftVal = LeftVal < 0 ? 0 : LeftVal > 180 ? 180 : LeftVal;
    // BallLeftGrip.write(LeftVal);
    // Serial.print("LeftVal : ");
    // Serial.println(LeftVal);

    // RightVal = gamepad.Dpad_right ? RightVal + 1 : RightVal;
    // RightVal = gamepad.Dpad_left ? RightVal -1 :RightVal;
    // RightVal = RightVal < 0 ? 0 : RightVal > 180 ? 180 : RightVal;
    // BallRightGrip.write(RightVal);
    // Serial.print("RightVal : ");
    // Serial.println(RightVal);

    // Serial.println(gamepad.lx);
    
    // Serial1.println("xxx");
    delay(75);
}
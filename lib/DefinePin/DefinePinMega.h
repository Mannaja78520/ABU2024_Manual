#ifndef DefinePinMega_H
#define DefinePinMega_H

#include <Arduino.h>
#include <Servo.h>
#include <motor.h>

// IMU Pin
#define SDA_PIN    20
#define SCL_PIN    21

// Motor_PWM_Pin
// Movement
#define motor1PWM   2 // ล้อหน้า
#define motor2PWM   3 // ล้อซ้าย
#define motor3PWM   4 // ล้อขวา
// Ball
#define motor4PWM   5 // มอเตอร์หมุนตัวซ้าย
// #define motor5PWM   6 // มอเตอร์หมุนตัวขวา

// Motor Pin
// Movement
#define motor1PinA  22 // ล้อหน้า
#define motor1PinB  23
#define motor2PinA  24 // ล้อซ้าย
#define motor2PinB  25
#define motor3PinA  26 // ล้อขวา
#define motor3PinB  27
// Ball
#define motor4PinA  28 // มอเตอร์หมุนตัวซ้าย
#define motor4PinB  29
// #define motor5PinA  30 // มอเตอร์หมุนตัวขวา
// #define motor5PinB  31

// Servo Pin
// Servo หนีบต้นกล้า (Harvest)
#define Servo1       8  
#define Servo2       9  
#define Servo3      10  
#define Servo4      11 
// Servo หนีบลูกบอล (Ball)
#define Servo5      7 // Servo ยก
#define Servo6      12 // แขนซ้าย
#define Servo7      13 // แขนขวา

// Solenoid Pin
#define GripSlide   48 // in1
#define GripUP      49// in2
#define KickBall    47 // in3

#endif
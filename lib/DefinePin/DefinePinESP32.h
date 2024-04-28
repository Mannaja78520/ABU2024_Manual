#ifndef DefinePinESP32_H
#define DefinePinESP32_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <motor_esp.h>

// IMU Pin
#define SDA_PIN    21
#define SCL_PIN    22

// Motor_PWM_Pin
// Movement
#define motor1PWM   25 // ล้อหน้า
#define motor2PWM   26 // ล้อซ้าย
#define motor3PWM   27 // ล้อชวา
// Ball
#define motor4PWM   28 // มอเตอร์หมุนตัวซ้าย
#define motor5PWM   29 // มอเตอร์หมุนตัวขวา

// Motor Pin
// Movement
#define motor1PinA  30 // ล้อหน้า
#define motor1PinB  31
#define motor2PinA  32 // ล้อซ้าย
#define motor2PinB  33
#define motor3PinA  34 // ล้อขวา
#define motor3PinB  35
// Ball
#define motor4PinA  16 // มอเตอร์หมุนตัวซ้าย
#define motor4PinB  15
#define motor5PinA  14 // มอเตอร์หมุนตัวขวา
#define motor5PinB  13

// Servo Pin
// Servo หนีบต้นกล้า (Harvest)
#define Servo1      12  // สายฟ้า
#define Servo2      11  // สายม่วง
#define Servo3      17  // สายเหลือง
#define Servo4      18 // สายเขียว
// Servo หนีบลูกบอล (Ball)
#define Servo5       8 // Servo ยก
#define Servo6       7 // แขนซ้าย
#define Servo7       6 // แขนขวา

// Solenoid Pin
#define GripSlide   29 // in1
#define GripUP      29 // in2
#define KickBall    29 // in3

#endif
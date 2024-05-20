#ifndef DRIVE_OUTPUT_H
#define DRIVE_OUTPUT_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV true
#define MOTOR2_INV false
#define MOTOR3_INV true
#define MOTOR4_INV false

#define MOTOR1_BREAK true
#define MOTOR2_BREAK true
#define MOTOR3_BREAK true
#define MOTOR4_BREAK true

#define MOTOR1_PWM 23
#define MOTOR1_IN_A 22
#define MOTOR1_IN_B 21

#define MOTOR2_PWM 19
#define MOTOR2_IN_A 18
#define MOTOR2_IN_B 5

#define MOTOR3_PWM 34
#define MOTOR3_IN_A 35
#define MOTOR3_IN_B 32

#define MOTOR4_PWM 33
#define MOTOR4_IN_A 25
#define MOTOR4_IN_B 26

#endif
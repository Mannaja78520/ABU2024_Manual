import sys
sys.path.append('../robot_ws/src/robot_core/src')
from utilize import *
import keyboard
from adafruit_servokit import ServoKit

# define servo
kit = ServoKit(channels=8)
Grip1 = kit.servo[0]
Grip2 = kit.servo[1]
Grip3 = kit.servo[2]
Grip4 = kit.servo[3]
BallUP_DOWN = kit.servo[4]
BallLeftGrip = kit.servo[5]
BallRightGrip = kit.servo[6]

Grip1.angle = 0
Grip2.angle = 0
Grip3.angle = 0
Grip4.angle = 0
BallUP_DOWN.angle   = 180
BallLeftGrip.angle  = 180
BallRightGrip.angle = 0

Grip1Angle = 0
Grip2Angle = 0
Grip3Angle = 0
Grip4Angle = 0
BallUP_DOWNAngle = 180
BallLeftGripAngle  = 180
BallRightGripAngle = 0

while True:
    key = keyboard.read_key()
    
    Grip1Angle = clip(Grip1Angle + 1 if key == 'w' else (Grip1Angle - 1 if key == 's' else Grip1Angle), 0, 180)
    Grip1.angle = Grip1Angle
    print(Grip1Angle)
    
    Grip2Angle = clip(Grip2Angle + 1 if key == 'e' else (Grip2Angle - 1 if key == 'd' else Grip2Angle), 0, 180)
    Grip2.angle = Grip2Angle
    print(Grip2Angle)
    
    Grip3Angle = clip(Grip3Angle + 1 if key == 'r' else (Grip3Angle - 1 if key == 'f' else Grip3Angle), 0, 180)
    Grip3.angle = Grip3Angle
    print(Grip3Angle)
    
    Grip4Angle = clip(Grip4Angle + 1 if key == 't' else (Grip4Angle - 1 if key == 'g' else Grip4Angle), 0, 180)
    Grip4.angle = Grip4Angle
    print(Grip4Angle)
    
    BallUP_DOWNAngle = clip(BallUP_DOWNAngle + 1 if key == 'y' else (BallUP_DOWNAngle - 1 if key == 'h' else BallUP_DOWNAngle), 0, 180)
    BallUP_DOWN.angle = BallUP_DOWNAngle
    print(BallUP_DOWNAngle)
    
    BallLeftGripAngle = clip(BallLeftGripAngle + 1 if key == 'u' else (BallLeftGripAngle - 1 if key == 'j' else BallLeftGripAngle), 0, 180)
    BallLeftGrip.angle = BallLeftGripAngle
    print(BallLeftGripAngle)
    
    BallRightGripAngle = clip(BallRightGripAngle + 1 if key == 'i' else (BallRightGripAngle - 1 if key == 'k' else BallRightGripAngle), 0, 180)
    BallRightGrip.angle = BallRightGripAngle
    print(BallRightGripAngle)
    
    if key == 'q':
        print('Quitting the program')
        break
        
    
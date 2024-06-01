from config.config import *
from adafruit_servokit import ServoKit

# define servo
kit = ServoKit(channels=16)
Grip1 = kit.servo[servo1]
Grip2 = kit.servo[servo2]
Grip3 = kit.servo[servo3]
Grip4 = kit.servo[servo4]
BallUP_DOWN = kit.servo[servo5]
BallLeftGrip = kit.servo[servo6]
BallRightGrip = kit.servo[servo7]

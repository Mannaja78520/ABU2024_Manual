import curses
import sys
sys.path.append('../robot_ws/src/robot_core/src')
from utilize import clip
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

def main(stdscr):

    Grip1Angle = 0
    Grip2Angle = 0
    Grip3Angle = 0
    Grip4Angle = 0
    BallUP_DOWNAngle = 180
    BallLeftGripAngle  = 180
    BallRightGripAngle = 0

    curses.cbreak()  # Disable line buffering
    stdscr.clear()
    stdscr.addstr(0, 0, "Press a key (q to quit):")
    stdscr.refresh()
    while True:
        key = stdscr.getch()  # Non-blocking keyboard input
        
        Grip1Angle = clip(Grip1Angle + 1 if key == ord('w') else (Grip1Angle - 1 if key == ord('s') else Grip1Angle), 0, 180)
        Grip1.angle = Grip1Angle
        # print(Grip1Angle)
            
        Grip2Angle = clip(Grip2Angle + 1 if key == ord('e') else (Grip2Angle - 1 if key == ord('d') else Grip2Angle), 0, 180)
        Grip2.angle = Grip2Angle
        # print(Grip2Angle)
            
        Grip3Angle = clip(Grip3Angle + 1 if key == ord('r') else (Grip3Angle - 1 if key == ord('f') else Grip3Angle), 0, 180)
        Grip3.angle = Grip3Angle
        # print(Grip3Angle)
        
        Grip4Angle = clip(Grip4Angle + 1 if key == ord('t') else (Grip4Angle - 1 if key == ord('g') else Grip4Angle), 0, 180)
        Grip4.angle = Grip4Angle
        # print(Grip4Angle)
        
        BallUP_DOWNAngle = clip(BallUP_DOWNAngle + 1 if key == ord('y') else (BallUP_DOWNAngle - 1 if key == ord('h') else BallUP_DOWNAngle), 0, 180)
        BallUP_DOWN.angle = BallUP_DOWNAngle
        # print(BallUP_DOWNAngle)
        
        BallLeftGripAngle = clip(BallLeftGripAngle + 1 if key == ord('u') else (BallLeftGripAngle - 1 if key == ord('j') else BallLeftGripAngle), 0, 180)
        BallLeftGrip.angle = BallLeftGripAngle
        # print(BallLeftGripAngle)
        
        BallRightGripAngle = clip(BallRightGripAngle + 1 if key == ord('i') else (BallRightGripAngle - 1 if key == ord('k') else BallRightGripAngle), 0, 180)
        BallRightGrip.angle = BallRightGripAngle
        # print(BallRightGripAngle)
        if key == ord('q'):
            print('Quitting the program')
            break
        
        elif key != -1:  # Check if a key is pressed
            stdscr.clear()
            stdscr.addstr(0, 0, "You pressed: {}".format(chr(key)))
            stdscr.addstr(1, 0, "Grip1 Angle: {}".format(Grip1Angle))
            stdscr.addstr(2, 0, "Grip2 Angle: {}".format(Grip2Angle))
            stdscr.addstr(3, 0, "Grip3 Angle: {}".format(Grip3Angle))
            stdscr.addstr(4, 0, "Grip4 Angle: {}".format(Grip4Angle))
            stdscr.addstr(5, 0, "BallUp_Down Angle: {}".format(BallUP_DOWNAngle))
            stdscr.addstr(6, 0, "Ball_Left   Angle: {}".format(BallLeftGripAngle))
            stdscr.addstr(7, 0, "Ball_Right  Angle: {}".format(BallRightGripAngle))
            stdscr.refresh()

curses.wrapper(main)
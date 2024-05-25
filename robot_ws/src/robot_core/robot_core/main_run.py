import rclpy
import math
import numpy as np
import time
import sys
sys.path.append('../robot_ws/src/robot_core/src')
from controller import Controller
from utilize import *
from gamepad_zigbee import gamepad_Zigbee

import lgpio
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from adafruit_servokit import ServoKit

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos

gamepad = gamepad_Zigbee('/dev/ttyUSB1', 230400)
control = Controller(1.3, 0.001)
mpu = MPU9250( 
            address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
            address_mpu_slave=None, 
            bus=1, 
            gfs=GFS_1000, 
            afs=AFS_8G, 
            )

# open gpio chip
grip_slide = 23
grip_up    = 24

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, grip_slide)
lgpio.gpio_claim_output(h, grip_up)
lgpio.gpio_write(h, grip_slide, 1)
lgpio.gpio_write(h, grip_up, 1)

maxSpeed = 1023.0
NormalSpeed = 1.0
SlowSpeed = 0.4
turnSpeed = 0.5
SpinBallSpeed = 1023.0

# define servo
kit = ServoKit(channels=8)
Grip1 = kit.servo[0]
Grip2 = kit.servo[1]
Grip3 = kit.servo[2]
Grip4 = kit.servo[3]
BallUP_DOWN = kit.servo[4]
BallLeftGrip = kit.servo[5]
BallRightGrip = kit.servo[6]

# setup servo
def setupServo():
    Grip1.angle = 0
    Grip2.angle = 0
    Grip3.angle = 0
    Grip4.angle = 0
    BallUP_DOWN.angle   = 180
    BallLeftGrip.angle  = 180
    BallRightGrip.angle = 0

class mainRun(Node):
    
    def Reset(self, IsResetGyro = False):
        # TransForm variables
        self.Y_Pressed = False
        self.ISGripUP = False
        self.B_Pressed = False
        self.ISGripSlide = False
        
        # Time
        self.CurrentTime = time.time()
        
        # Macro variables
        self.yaw = 0
        self.LastTime = 0
        self.setpoint = 0
        self.lastRXTime = 0
        self.UseIMU = False
        self.IMUHeading = True
        self.M_Pressed = False
        self.LSB_Pressed = False

        # variables
        self.K_Pressed = self.IS_13_Keep = self.IS_24_Keep = False
        self.A_Pressed = self.GotBall = self.ChargeBall = self.ISBallSpin = False
        self.X_Pressed = False
        self.ArmUp = True
        
        self.MacroTime = 0
        
        # Control Reset
        control.ResetVariable()
        
        if IsResetGyro == True:
            # Config imu
            mpu.configure()
            mpu.calibrate()
            mpu.configure()
            mpu.gbias
        
    def __init__(self):
        super().__init__("Robot_mainRun_Control_Node")

        # Emergency variables
        self.EmergencyStop = True
        
        self.received_data = ""
        self.Reset(True)
        setupServo()
        
        self.debug = self.create_subscription(
            Twist, "debug/motor", self.debug_callback, qos_profile=qos.qos_profile_system_default,
        )
        
        self.sent_drive = self.create_publisher(
            Twist, "moveMotor", qos_profile=qos.qos_profile_system_default
        )
        # self.sent_gripper = self.create_publisher(
        #     Twist, "gripper", qos_profile=qos.qos_profile_system_default
        # )
        self.sent_drive_timer = self.create_timer(0.03, self.sent_to_microros)
        # self.sent_gripper_timer = self.create_timer(0.05, self.sent_gripper_callback)

    def debug_callback(self, msgin):
        return
        
    def Emergency_StartStop(self):
        if(gamepad.upload):
            self.Reset()
            self.EmergencyStop = True
            return
        
        if (gamepad.logo and self.EmergencyStop):
            self.Reset(True)
            setupServo()
            self.EmergencyStop = False
            return
    
    def imu(self):
        self.CurrentTime = time.time()
        Dt = self.CurrentTime - self.LastTime
        self.LastTime = self.CurrentTime
        gyro_data = mpu.readGyroscopeMaster()
        
        # gyro_data[2] is gz
        calibrate_gz = 0 if abs(gyro_data[2]) < 0.5 else gyro_data[2]
        
        calibrate_gz = math.radians(calibrate_gz) * -1
        
        self.yaw += WrapRads(calibrate_gz * Dt)
        
        if (gamepad.screen):
            self.yaw = 0
            self.setpoint = 0
            return
        
        m = gamepad.menu
        if (not m) :
            self.M_Pressed = False
            return
        
        if (self.M_Pressed) :
            return
        self.M_Pressed = True
        if(not self.UseIMU) :
            self.UseIMU = True
            control.ResetVariable()
            return
        self.UseIMU = False
        
    def imuHeading(self): 
        lsb = gamepad.left_stick_button
        if (not lsb) :
            self.LSB_Pressed = False
            return
        
        if (self.LSB_Pressed) :
            return
        self.LSB_Pressed = True
        if(not self.UseIMU) :
            self.IMUHeading = True
            control.ResetVariable()
            return
        self.IMUHeading = False
            
    def MoveRobot(self):
        lx =  gamepad.lx * NormalSpeed
        ly =  gamepad.ly * NormalSpeed * -1
        rx =  gamepad.rx * turnSpeed

        ly = SlowSpeed if gamepad.dpad_up    else(-SlowSpeed if gamepad.dpad_down else ly)
        lx = SlowSpeed if gamepad.dpad_right else (-SlowSpeed if gamepad.dpad_left else lx)

        self.setpoint = self.yaw

        D = max(abs(lx)+abs(ly)+abs(rx), 1.0)

        motor1Speed = float("{:.1f}".format((ly + lx + rx) / D * maxSpeed))
        motor2Speed = float("{:.1f}".format((ly - lx - rx) / D * maxSpeed))
        motor3Speed = float("{:.1f}".format((ly - lx + rx) / D * maxSpeed))
        motor4Speed = float("{:.1f}".format((ly + lx - rx) / D * maxSpeed))
        return motor1Speed, motor2Speed, motor3Speed, motor4Speed
    
    def MoveRobot_IMU(self):
        lx =  gamepad.lx * NormalSpeed
        ly =  gamepad.ly * NormalSpeed * -1
        rx =  gamepad.rx * turnSpeed

        ly = SlowSpeed if gamepad.dpad_up    else (-SlowSpeed if gamepad.dpad_down else ly)
        lx = SlowSpeed if gamepad.dpad_right else (-SlowSpeed if gamepad.dpad_left else lx)
        
        if self.IMUHeading :
            x2  =  (math.cos(self.yaw) * lx) - (math.sin(self.yaw) * ly)
            y2  =  (math.sin(self.yaw) * lx) + (math.cos(self.yaw) * ly)
        if not self.IMUHeading :
            x2  =  lx
            y2  =  ly
        
        R = control.Calculate(WrapRads(self.setpoint - self.yaw))
        # R = rx
        self.lastRXTime = self.CurrentTime if rx != 0 else self.lastRXTime
        if (rx != 0 or  self.CurrentTime - self.lastRXTime < 0.3) :
            R = rx
            self.setpoint = self.yaw

        D = max(abs(x2)+abs(y2)+abs(R), 1.0)

        motor1Speed = float("{:.1f}".format((y2 + x2 + R) / D * maxSpeed))
        motor2Speed = float("{:.1f}".format((y2 - x2 - R) / D * maxSpeed))
        motor3Speed = float("{:.1f}".format((y2 - x2 + R) / D * maxSpeed))
        motor4Speed = float("{:.1f}".format((y2 + x2 - R) / D * maxSpeed))
        return motor1Speed, motor2Speed, motor3Speed, motor4Speed
    
    def Slide_Transform(self):
        b = gamepad.B
        if (not b) :
            self.B_Pressed = False
            return
        if (self.B_Pressed) : 
            return
        self.B_Pressed = True
        if(not self.ISGripSlide) :
            lgpio.gpio_write(h, grip_slide, 0)
            self.ISGripSlide = True
            return
        lgpio.gpio_write(h, grip_slide, 1)
        self.ISGripSlide = False
        
    def UpDown_Transform(self):
        y = gamepad.Y
        if (not y) :
            self.Y_Pressed = False
            return
        if (self.Y_Pressed) :
            return
        self.Y_Pressed = True
        if (not self.ISGripUP) :
            lgpio.gpio_write(h, grip_up, 0)
            self.ISGripUP = True
            return
        lgpio.gpio_write(h, grip_up, 1)
        self.ISGripUP = False

    def keepHarvest(self):
        # x = 0.0
        dropDelay = 0.4
        lb = gamepad.left_bumper
        rb = gamepad.right_bumper
        lt = gamepad.left_trigger  > 0.3
        rt = gamepad.right_trigger > 0.3
        if not(lb or rb or rt or lt) :
            self.K_Pressed = False
            return
        if self.K_Pressed:
            return 
        if lb and rb:
            # x = 1.0
            Grip1.angle = 112
            Grip2.angle = 112
            Grip3.angle = 112
            Grip4.angle = 112
            self.IS_13_Keep = self.IS_24_Keep = True
            time.sleep(0.35)
            return 
        if self.IS_13_Keep :
            if lb or lt :
                self.IS_13_Keep = False
                if lt :
                    # x = 2.0
                    Grip1.angle = 86
                    Grip3.angle = 86
                    time.sleep(dropDelay)
                    return 
                # x = 3.0
                Grip1.angle = 0
                Grip3.angle = 0
                return 
        if self.IS_24_Keep :
            if rb or rt :
                self.IS_24_Keep = False
                if rt:
                    # x = 4.0
                    Grip2.angle = 86
                    Grip4.angle = 86
                    time.sleep(dropDelay)
                    return 
                # x = 5.0
                Grip2.angle = 0
                Grip4.angle = 0
                return 
            
    def keepBall(self):
        # y = 0.0
        a = gamepad.A
        MTime = self.CurrentTime - self.MacroTime
        if (MTime > 1 and self.GotBall and not self.ChargeBall) :
            # y = 3.0
            BallUP_DOWN.angle = 160
            self.ArmUp = True
            if(MTime > 2.5) :
                # y = 4.0
                BallUP_DOWN.angle = 100
                if(MTime > 3) :
                    # y = 5.0
                    self.ISBallSpin = True
                    self.ChargeBall = True
            return
                    
        if (not a) :
            self.A_Pressed = False
            return
        if (self.A_Pressed) :
            return
        self.A_Pressed = True
        if(not self.GotBall and not self.ArmUp) :
            BallLeftGrip.angle  = 180
            BallRightGrip.angle =  0
            self.MacroTime = self.CurrentTime
            self.GotBall = True
            return 
        BallUP_DOWN.angle = 5
        BallLeftGrip.angle = 130
        BallRightGrip.angle = 55
        self.ArmUp = False
        self.GotBall = False
        self.ChargeBall = False
        self.ISBallSpin = False
        return 
    
    def AdjustArm(self):
        # z = 0.0
        x = gamepad.X
        if (not x) :
            self.X_Pressed = False
            return 
        if (self.X_Pressed) :
            return 
        self.X_Pressed = True
        if self.ChargeBall and self.ArmUp and x: # KickBall
            # z = 3.0
            BallUP_DOWN.angle = 160
            time.sleep(0.3)
            BallUP_DOWN.angle = 5
            time.sleep(0.3)
            BallLeftGrip.angle = 130
            BallRightGrip.angle = 55
            self.ArmUp = False
            self.GotBall = False
            self.ChargeBall = False
            self.ISBallSpin = False
            return  
        if (not self.ArmUp and not self.ChargeBall):
            # z = 1.0
            BallLeftGrip.angle = 180
            BallRightGrip.angle = 0
            time.sleep(0.2)
            BallUP_DOWN.angle = 175
            self.ArmUp = True
            return 
        
        if(self.ArmUp and not self.ChargeBall):
            # z = 2.0
            BallUP_DOWN.angle = 5
            time.sleep(0.2)
            BallLeftGrip.angle = 130
            BallRightGrip.angle = 55
            self.ArmUp = self.GotBall = self.ChargeBall = False
            return 
        
    def sent_to_microros(self):  # publisher drive topic
        movement_msg = Twist()
        gamepad.receive_data()
        
        self.Emergency_StartStop()
        if self.EmergencyStop :
            print (gamepad.received_data)
        if not self.EmergencyStop:
            self.imu()
            self.Slide_Transform()
            self.UpDown_Transform()
            self.keepHarvest()
            self.AdjustArm()
            
            if not(self.ISGripSlide or self.ISGripUP):
                self.keepBall()         
            
            if self.UseIMU:
                movement_msg.linear.x, movement_msg.linear.y, movement_msg.linear.z, movement_msg.angular.x = self.MoveRobot()
            if not self.UseIMU:
                movement_msg.linear.x, movement_msg.linear.y, movement_msg.linear.z, movement_msg.angular.x = self.MoveRobot_IMU()
            
            print(self.ISBallSpin)
            movement_msg.angular.y = float(self.ISBallSpin)
            movement_msg.angular.z = SpinBallSpeed
        
        self.sent_drive.publish(movement_msg)

def main():
    rclpy.init()

    sub = mainRun()   
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

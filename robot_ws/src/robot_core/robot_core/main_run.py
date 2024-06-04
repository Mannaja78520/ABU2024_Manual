import rclpy
import math
import time
from std_msgs.msg import String
import numpy as np

from config.config import *
from src.controller import Controller
from src.utilize import *
from src.gamepad_zigbee import gamepad_Zigbee
from src.imu import IMU

from adafruit_servokit import ServoKit
import lgpio

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos

gamepad = gamepad_Zigbee('/dev/ttyUSB1', 230400)
imu_control = Controller(2.32, 0.1)
brake_control_x2 = Controller(kp = 100, ki = 1)
brake_control_y2 = Controller(kp = 150, ki = 9, kd = 3)
imu = IMU()

# define servo
kit = ServoKit(channels=16)
Grip1 = kit.servo[servo1]
Grip2 = kit.servo[servo2]
Grip3 = kit.servo[servo3]
Grip4 = kit.servo[servo4]
BallUP_DOWN = kit.servo[servo5]
BallLeftGrip = kit.servo[servo6]
BallRightGrip = kit.servo[servo7]

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, grip_slide)
lgpio.gpio_claim_output(h, grip_up)
lgpio.gpio_write(h, grip_slide, 1)
lgpio.gpio_write(h, grip_up, 1)
lgpio.gpio_claim_input(h, emergency_pin, lgpio.SET_BIAS_PULL_DOWN)

# setup servo
def setupServo():
    Grip1.angle = 0
    Grip2.angle = 0
    Grip3.angle = 0
    Grip4.angle = 0
    BallUP_DOWN.angle   = 180
    BallLeftGrip.angle  = 180
    BallRightGrip.angle = 5

class mainRun(Node):
    
    def Reset(self, IsResetGyro = False):
        # TransForm variables
        self.Y_Pressed = False
        self.ISGripUP = False
        self.B_Pressed = False
        self.ISGripSlide = False
        
        # Time
        self.CurrentTime = time.time()
        
        # IMU variables
        self.LastTime = self.CurrentTime
        self.UseIMU = True
        self.IMUHeading = True
        # Gyro
        self.yaw = To_Radians(-90)
        self.setpoint = To_Radians(-90)
        
        # Macro variables
        self.lastRXTime = 0
        self.M_Pressed = False
        self.LSB_Pressed = False

        # variables
        self.K_Pressed = self.IS_13_Keep = self.IS_24_Keep = False
        self.A_Pressed = self.GotBall = self.ChargeBall = self.ISBallSpin = False
        self.X_Pressed = False
        self.ArmUp = True
        
        self.MacroTime = 0
        
        # Control variables
        self.loopCheckBrake = 0
        self.lastx2 = self.lasty2 = np.arange(100)
        # Control Reset
        imu_control.ResetVariable()
        brake_control_x2.ResetVariable()
        brake_control_y2.ResetVariable()
        
        if IsResetGyro == True:
            # Config imu
            imu.configure()
        
    def __init__(self):
        super().__init__("Robot_mainRun_Control_Node")

        # Emergency variables
        self.EmergencyStop = True
        
        self.Reset(True)
        setupServo()
        
        self.sent_drive = self.create_publisher(
            Twist, "moveMotor", qos_profile=qos.qos_profile_system_default
        )
        self.sent_imu = self.create_publisher(
            Twist, "imu", qos_profile=qos.qos_profile_system_default
        )
        self.sent_gamepad = self.create_publisher(String, 'gamepad', 60)
        # self.debug = self.create_subscription(
        #     Twist, "debug/motor", self.debug_callback, qos_profile=qos.qos_profile_system_default,
        # )
        
        self.sent_drive_timer = self.create_timer(0.05, self.sent_to_microros)
        # self.sent_gripper_timer = self.create_timer(0.05, self.sent_gripper_callback)

    # def debug_callback(self, msgin):
    #     return
        
    def Emergency_StartStop(self):
        IS_EmergencyActive = lgpio.gpio_read(h, emergency_pin)
        if(gamepad.upload or gamepad.received_data == '' or not IS_EmergencyActive):
            setupServo()
            self.Reset()
            self.EmergencyStop = True
            return
        
        if (gamepad.logo and self.EmergencyStop and IS_EmergencyActive):
            self.Reset(True)
            setupServo()
            self.EmergencyStop = False
            return
    
    def imu_Read_Action(self):
        self.CurrentTime = time.time()
        Dt = self.CurrentTime - self.LastTime
        self.LastTime = self.CurrentTime
        
        self.accel_x_m2 = 0 if abs(imu.ax) < 0.05 else imu.ax 
        
        filter_gz = 0 if abs(imu.gz) < 0.475 else To_Radians(imu.gz)
        self.yaw += WrapRads(filter_gz * Dt)
        # print(self.yaw)
        
        if (gamepad.screen):
            self.yaw = To_Radians(-90)
            self.setpoint = To_Radians(-90)
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
            imu_control.ResetVariable()
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
        if(not self.IMUHeading) :
            self.IMUHeading = True
            imu_control.ResetVariable()
            return
        self.IMUHeading = False
            
    def MoveRobot(self):
        # Gain movement
        lx =  gamepad.lx * NormalSpeed
        ly =  gamepad.ly * NormalSpeed * -1
        rx =  gamepad.rx * turnSpeed
        Bx = By = 0

        # Slow movement
        ly = SlowSpeed if gamepad.dpad_up    else (-SlowSpeed if gamepad.dpad_down else ly)
        lx = SlowSpeed if gamepad.dpad_right else (-SlowSpeed if gamepad.dpad_left else lx)
        
        if self.UseIMU :
            if self.IMUHeading :
                x2  =  (math.cos(self.yaw) * lx) - (math.sin(self.yaw) * ly)
                y2  =  (math.sin(self.yaw) * lx) + (math.cos(self.yaw) * ly)
            else :
                x2  =  lx
                y2  =  ly
            
            R = imu_control.Calculate(WrapRads(self.setpoint - self.yaw))   
            if (lx == 0.0 and ly == 0.0 and rx == 0.0) and abs(R) < 0.035:
                R = 0.0
            
            self.lastRXTime = self.CurrentTime if rx != 0 else self.lastRXTime
            if (rx != 0 or  self.CurrentTime - self.lastRXTime < 0.45) :
                R = rx
                self.setpoint = self.yaw
        else :
            self.setpoint = self.yaw
            x2 = lx
            y2 = ly
            R  = rx
            
        # self.loopCheckBrake = 0 if self.loopCheckBrake == 50 else self.loopCheckBrake
        # self.lastx2[self.loopCheckBrake], self.lasty2[self.loopCheckBrake] = x2, y2
        # lastx2 = np.sum(self.lastx2)
        # lasty2 = np.sum(self.lasty2)
        
        # if (lastx2 > 0 or lasty2 > 0) and (lx == 0 and ly == 0):
        #     Bx = brake_control_x2.Calculate(lastx2)
        #     By = brake_control_y2.Calculate(lasty2)

        D = max(abs(x2)+abs(y2)+abs(R), 1.0)
        motor4Speed = float("{:.1f}".format((y2 + x2 - R - Bx - By) / D * maxSpeed))
        motor1Speed = float("{:.1f}".format((y2 + x2 + R - Bx - By) / D * maxSpeed))
        motor2Speed = float("{:.1f}".format((y2 - x2 - R + Bx - By) / D * maxSpeed))
        motor3Speed = float("{:.1f}".format((y2 - x2 + R + Bx - By) / D * maxSpeed))
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
        dropDelay = 0.3
        lb = gamepad.left_bumper
        rb = gamepad.right_bumper
        lt = gamepad.left_trigger  > 0.3
        rt = gamepad.right_trigger > 0.3
        if lb and rb:
            # x = 1.0
            Grip1.angle = 180
            Grip2.angle = 180
            Grip3.angle = 180
            Grip4.angle = 180
            self.IS_13_Keep = self.IS_24_Keep = True
            self.K_Pressed = True
            return 
        if not(lb or rb or rt or lt) :
            self.K_Pressed = False
            return
        if self.K_Pressed:
            return 
        self.K_Pressed = True
        if self.IS_13_Keep :
            if lb or lt :
                self.IS_13_Keep = False
                if lt :
                    # x = 2.0
                    Grip1.angle = 158
                    Grip3.angle = 158
                    time.sleep(dropDelay)
                # x = 3.0
                Grip1.angle = 0
                Grip3.angle = 0
                return 
        if self.IS_24_Keep :
            if rb or rt :
                self.IS_24_Keep = False
                if rt:
                    # x = 4.0
                    Grip2.angle = 158
                    Grip4.angle = 158
                    time.sleep(dropDelay) 
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
            if(MTime > 2) :
                # y = 4.0
                BallUP_DOWN.angle = 135
                if(MTime > 2.3) :
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
        BallUP_DOWN.angle = 15
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
            BallUP_DOWN.angle = 180
            time.sleep(0.35)
            BallUP_DOWN.angle = 15
            time.sleep(0.1)
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
            BallRightGrip.angle = 5
            time.sleep(0.1)
            BallUP_DOWN.angle = 175
            self.ArmUp = True
            return 
        
        if(self.ArmUp and not self.ChargeBall):
            # z = 2.0
            BallUP_DOWN.angle = 15
            time.sleep(0.1)
            BallLeftGrip.angle = 130
            BallRightGrip.angle = 55
            self.ArmUp = self.GotBall = self.ChargeBall = False
            return 
        
    def sent_to_microros(self):  # publisher drive topic
        movement_msg = Twist()
        imu_msg = Twist()
        gamepad_msg = String()
        
        gamepad.receive_data()
        gamepad_msg.data = gamepad.received_data
        imu.read()
        imu_msg.linear.x, imu_msg.linear.y, imu_msg.linear.z = map(float, imu.accel_data)
        imu_msg.angular.x, imu_msg.angular.y, imu_msg.angular.z = float(imu.gx), float(imu.gy), float(imu.yaw)
        
        self.Emergency_StartStop()
        # if self.EmergencyStop :
        #     print (imu.gz)
        if not self.EmergencyStop:
            self.imu_Read_Action()
            self.imuHeading()
            self.Slide_Transform()
            self.UpDown_Transform()
            self.keepHarvest()
            self.AdjustArm()
            
            if not(self.ISGripSlide or self.ISGripUP):
                self.keepBall()         
            
            movement_msg.linear.x, movement_msg.linear.y, movement_msg.linear.z, movement_msg.angular.x = self.MoveRobot()
            movement_msg.angular.y = float(self.ISBallSpin)
            movement_msg.angular.z = SpinBallSpeed
        
        # print(lgpio.gpio_read(h, emergency_pin))
        
        self.sent_drive.publish(movement_msg)
        self.sent_imu.publish(imu_msg)
        self.sent_gamepad.publish(gamepad_msg)

def main():
    rclpy.init()

    sub = mainRun()   
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

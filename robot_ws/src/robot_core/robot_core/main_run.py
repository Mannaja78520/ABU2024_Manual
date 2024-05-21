import rclpy
import math
import numpy as np

import pygame
import serial
import time

import lgpio

from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy import qos

# class Calculate():
    

class Zigbee(Node):
    def __init__(self):
        super().__init__("Robot_Zigbee_Control_Node")

        self.maxSpeed = 1023.0
        self.NormalSpeed = 1.0
        self.turnSpeed = 0.5

        # open gpio chip
        self.grip_slide = 23
        self.grip_up    = 24
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, self.grip_slide)
        lgpio.gpio_claim_output(self.h, self.grip_up)
        lgpio.gpio_write(self.h, self.grip_slide, 1)
        lgpio.gpio_write(self.h, self.grip_up, 1)
        
        self.Y_Pressed = False
        self.ISGripUP = False
        self.B_Pressed = False
        self.ISGripSlide = False
        
        self.mpu = MPU9250( 
            address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
            address_mpu_slave=None, 
            bus=1, 
            gfs=GFS_1000, 
            afs=AFS_8G, 
            )
        
        self.mpu.configure()
        self.mpu.calibrate()
        self.mpu.configure()
        self.mpu.gbias
        
        self.CurrentTime = time.time()
        self.yaw = 0
        self.LastTime = 0
        self.setpoint = 0
        self.UseIMU = False
        self.M_Pressed = False

        self.lx = self.ly = self.rx = self.ry = 0
        self.right_trigger = self.left_trigger = 0

        self.A = self.B = self.X = self.Y = self.left_bumper = 0
        self.right_bumper = self.screen = self.menu = self.logo = 0
        self.left_stick_button = self.right_stick_button = self.upload = 0

        self.dpad_right = self.dpad_left = self.dpad_up = self.dpad_down = False
        
        # Last data variables
        self.last_lx = self.last_ly = self.last_rx = self.last_ry = 0
        self.last_rt = self.last_lt = 0

        self.last_A = self.last_B = self.last_X = self.last_Y = self.last_lb = 0
        self.last_rb = self.last_s = self.last_m = self.last_f = self.last_lsb = 0
        self.last_rsb = self.last_u = 0

        self.last_dr = self.last_dl = self.last_du = self.last_dd = False
        
        self.K_Pressed = self.IS_13_Keep = self.IS_24_Keep = False
        self.A_Pressed = self.GotBall = self.ChargeBall = False
        self.X_Pressed = False
        self.ArmUp = True
        
        self.last_data_time = self.last_time = time.time()
        self.MacroTime = 0
        
        self.SpinBallSpeed = 1023.0
        
        # self.debug = self.create_subscription(
        #     Twist, "debug/motor", self.debug_callback, qos_profile=qos.qos_profile_system_default,
        # )
        # self.debug
        
        self.sent_drive = self.create_publisher(
            Twist, "moveMotor", qos_profile=qos.qos_profile_system_default
        )
        self.sent_gripper = self.create_publisher(
            Twist, "gripper", qos_profile=qos.qos_profile_system_default
        )
        self.sent_drive_timer = self.create_timer(0.03, self.sent_data)
        # self.sent_gripper_timer = self.create_timer(0.05, self.sent_gripper_callback)
        
        self.ser = self.initialize_serial('/dev/ttyUSB1', 230400)
        
    # def debug_callback(self, msg_in):
    #     self.slope = 0
        
    def reset_variable(self):
        self.lx = 0
        self.ly = 0
        self.rx = 0
        self.ry = 0
        self.right_trigger = 0
        self.left_trigger = 0

        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.left_bumper = 0
        self.right_bumper = 0
        self.screen = 0
        self.menu = 0
        self.logo = 0
        self.left_stick_button = 0
        self.right_stick_button = 0
        self.upload = 0

        self.dpad_right = False
        self.dpad_left = False
        self.dpad_up = False
        self.dpad_down = False

    def initialize_serial(self, port, baud_rate):
        try:
            ser = serial.Serial(port, baud_rate)
            time.sleep(2)
            return ser
        except serial.SerialException:
            return None
    
    def use_last_data(self):
        self.lx = self.last_lx
        self.ly = self.last_ly
        self.rx = self.last_rx
        self.ry = self.last_ry
        self.right_trigger = self.last_rt
        self.left_trigger = self.last_lt

        self.A = self.last_A
        self.B = self.last_B
        self.X = self.last_X
        self.Y = self.last_Y
        self.left_bumper = self.last_lb
        self.right_bumper = self.last_rb
        self.screen = self.last_s
        self.menu = self.last_m
        self.logo = self.last_f
        self.left_stick_button = self.last_lsb
        self.right_stick_button = self.last_rsb
        self.upload = self.last_u

        self.dpad_right = self.last_dr
        self.dpad_left = self.last_dl
        self.dpad_up = self.last_du
        self.dpad_down = self.last_dd
        
    def update_last_data(self):
        self.last_lx = self.lx
        self.last_ly = self.ly
        self.last_rx = self.rx
        self.last_ry = self.ry
        self.last_rt = self.right_trigger
        self.last_lt = self.left_trigger
        
        self.last_A = self.A
        self.last_B = self.B
        self.last_X = self.X
        self.last_Y = self.Y
        self.last_lb = self.left_bumper
        self.last_rb = self.right_bumper
        self.last_s = self.screen
        self.last_m = self.menu
        self.last_f = self.logo
        self.last_lsb = self.left_stick_button
        self.last_rsb = self.right_stick_button
        self.last_u = self.upload

        self.last_dr = self.dpad_right
        self.last_dl = self.dpad_left
        self.last_du = self.dpad_up
        self.last_dd = self.dpad_down
        

    def receive_data(self, ser):
        expected_data_length = 40
        have_data_from_controller = ser.in_waiting
        if have_data_from_controller > 0:
            # reset_variables()
            self.last_data_time = self.CurrentTime
            received_data = ser.readline().strip().decode()
            if expected_data_length <= len(received_data) <= 80:
                self.last_time = self.CurrentTime
                tokens = received_data.split(",")
                index = 0
                sum_of_data = 0
                for token in tokens[:21]:
                    data = int(token) if token else 0
                    sum_of_data += abs(data)
                    if index == 0:
                        checksum = data
                        sum_of_data = 0
                    elif index == 1:
                        self.lx = data / 100.0
                    elif index == 2:
                        self.ly = data / 100.0
                    elif index == 3:
                        self.left_trigger = data / 100.0
                    elif index == 4:
                        self.rx = data / 100.0
                    elif index == 5:
                        self.ry = data / 100.0
                    elif index == 6:
                        self.right_trigger = data / 100.0
                    elif index == 7:
                        self.A = data
                    elif index == 8:
                        self.B = data
                    elif index == 9:
                        self.X = data
                    elif index == 10:
                        self.Y = data
                    elif index == 11:
                        self.left_bumper = data
                    elif index == 12:
                        self.right_bumper = data
                    elif index == 13:
                        self.screen = data
                    elif index == 14:
                        self.menu = data
                    elif index == 15:
                        self.logo = data
                    elif index == 16:
                        self.left_stick_button = data
                    elif index == 17:
                        self.right_stick_button = data
                    elif index == 18:
                        self.upload = data
                    elif index == 19:
                        right_left_dpad = data
                    elif index == 20:
                        up_down_dpad = data
                    index += 1
                # Discard any remaining data in the buffer
                ser.reset_input_buffer()

                received_checksum = abs(sum_of_data - checksum)
                if received_checksum == 0:
                    # Debugging: Print received data
                    # print("Data Match")
                    self.dpad_right = right_left_dpad == 1
                    self.dpad_left = right_left_dpad == -1
                    self.dpad_up = up_down_dpad == 1
                    self.dpad_down = up_down_dpad == -1
                    # compare_data(self.CurrentTime)
                    self.update_last_data()
                    return
                print("Checksum mismatchnot ")
                ser.readline().decode('utf-8').rstrip()
                self.use_last_data()
                return
            print("Incomplete data received ")
            ser.readline().decode('utf-8').rstrip()
            self.use_last_data()
            return
        if self.CurrentTime - self.last_data_time < 300:
            have_data_from_controller = True
            self.use_last_data()
            return
        have_data_from_controller = False
        
    
    # def start:
    
    def imu(self):
        self.CurrentTime = time.time()
        Dt = self.CurrentTime - self.LastTime
        self.LastTime = self.CurrentTime
        gyro_data = self.mpu.readGyroscopeMaster()
        
        calibrate_gyrodata = 0 if abs(gyro_data[2]) < 0.5 else gyro_data[2]
        
        calibrate_gyrodata = math.radians(calibrate_gyrodata)
        
        self.yaw += calibrate_gyrodata * Dt
        
        self.yaw -= math.pi*2 if self.yaw >  math.pi else self.yaw
        self.yaw += math.pi*2 if self.yaw < -math.pi else self.yaw
        
        if (self.screen):
            self.yaw = 0

        m = self.menu
        if (not m) :
            self.M_Pressed = False
            return
        
        if (self.M_Pressed) :
            return
        self.M_Pressed = True
        if(not self.UseIMU) :
            self.UseIMU = True
            # c.reset()
            return
        self.UseIMU = False
            
    def MoveRobot(self):
        lx =  self.lx * self.NormalSpeed
        ly =  self.ly * self.NormalSpeed * -1
        rx =  self.rx * self.turnSpeed

        ly =  0.4 if self.dpad_up else ly
        ly = -0.4 if self.dpad_down else ly
        lx =  0.4 if self.dpad_right else lx
        lx = -0.4 if self.dpad_left else lx

        self.setpoint = self.yaw if rx != 0 else self.setpoint

        D = max(abs(lx)+abs(ly)+abs(rx), 1.0)

        motor1Speed = float("{:.1f}".format((ly + lx + rx) / D * self.maxSpeed))
        motor2Speed = float("{:.1f}".format((ly - lx - rx) / D * self.maxSpeed))
        motor3Speed = float("{:.1f}".format((ly - lx + rx) / D * self.maxSpeed))
        motor4Speed = float("{:.1f}".format((ly + lx - rx) / D * self.maxSpeed))
        return motor1Speed, motor2Speed, motor3Speed, motor4Speed
    
    def MoveRobot_IMU(self):
        lx =  self.lx * self.NormalSpeed
        ly =  self.ly * self.NormalSpeed * -1
        rx =  self.rx * self.turnSpeed

        ly =  0.4 if self.dpad_up else ly
        ly = -0.4 if self.dpad_down else ly
        lx =  0.4 if self.dpad_right else lx
        lx = -0.4 if self.dpad_left else lx
        
        x2  =  (math.cos(-self.yaw) * lx) - (math.sin(-self.yaw) * ly)
        y2  =  (math.sin(-self.yaw) * lx) + (math.cos(-self.yaw) * ly)
        
        # R = c.Calculate(self.yaw - setpoint)
        R = rx
        lastRXTime = self.CurrentTime if rx != 0 else lastRXTime
        if (rx != 0 or  self.CurrentTime - lastRXTime < 300) :
            R = rx
            self.setpoint = self.yaw

        D = max(abs(lx)+abs(ly)+abs(R), 1.0)

        motor1Speed = float("{:.1f}".format((ly + lx + R) / D * self.maxSpeed))
        motor2Speed = float("{:.1f}".format((ly - lx - R) / D * self.maxSpeed))
        motor3Speed = float("{:.1f}".format((ly - lx + R) / D * self.maxSpeed))
        motor4Speed = float("{:.1f}".format((ly + lx - R) / D * self.maxSpeed))
        return motor1Speed, motor2Speed, motor3Speed, motor4Speed
    
    def Slide_Transform(self):
        b = self.B
        if (not b) :
            self.B_Pressed = False
            return
        if (self.B_Pressed) : 
            return
        self.B_Pressed = True
        if(not self.ISGripSlide) :
            lgpio.gpio_write(self.h, self.grip_slide, 0)
            self.ISGripSlide = True
            return
        lgpio.gpio_write(self.h, self.grip_slide, 1)
        self.ISGripSlide = False
        
    def UpDown_Transform(self):
        y = self.Y
        if (not y) :
            self.Y_Pressed = False
            return
        if (self.Y_Pressed) :
            return
        self.Y_Pressed = True
        if (not self.ISGripUP) :
            lgpio.gpio_write(self.h, self.grip_up, 0)
            self.ISGripUP = True
            return
        lgpio.gpio_write(self.h, self.grip_up, 1)
        self.ISGripUP = False

    def keepHarvest(self):
        x = 0.0
        lb = self.left_bumper
        rb = self.right_bumper
        lt = self.left_trigger  > 0.3
        rt = self.right_trigger > 0.3
        if not(lb or rb or rt or lt) :
            self.K_Pressed = False
            return float(x)
        if self.K_Pressed:
            return float(x)
        if lb and rb:
            x = 1.0
            self.IS_13_Keep = self.IS_24_Keep = True
            # time.sleep(0.35)
            return float(x)
        if self.IS_13_Keep :
            if lb or lt :
                self.IS_13_Keep = False
                if lt :
                    x = 2.0
                    return float(x)
                x = 3.0
                return float(x)
        if self.IS_24_Keep :
            if rb or rt :
                self.IS_24_Keep = False
                if rt:
                    x = 4.0
                    return float(x)
                x = 5.0
                return float(x)
            
    def keepBall(self):
        y = 0.0
        MTime = self.CurrentTime - self.MacroTime
        if (MTime > 1 and self.GotBall and not self.ChargeBall) :
            y = 3.0
            # BallUP_DOWN.write(155)
            self.ArmUp = True
            if(MTime > 2.5) :
                y = 4.0
                # BallUP_DOWN.write(125)
                if(MTime > 3) :
                    y = 5.0
                    # r.BallSpin(BallSpinPower)
                    self.ISBallSpin = True
                    self.ChargeBall = True
            return y
                    
        if (not self.A) :
            self.A_Pressed = False
            return y
        if (self.A_Pressed) :
            return y
        self.A_Pressed = True
        if(not self.GotBall and not self.ArmUp) :
            y = 1.0
            self.MacroTime = self.CurrentTime
            self.GotBall = True
            return y
        y = 2.0
        
        self.ArmUp = False
        self.GotBall = False
        self.ChargeBall = False
        return y
    
    def AdjustArm(self):
        z = 0.0
        x = self.X
        if (not x) :
            self.X_Pressed = False
            return float(z)
        if (self.X_Pressed) :
            return float(z)
        self.X_Pressed = True
        if self.ChargeBall and self.ArmUp and x: # KickBall
            z = 3.0
            self.ChargeBall = self.ArmUp = self.GotBall = False
            return z
        if (not self.ArmUp and not self.ChargeBall):
            z = 1.0
            self.ArmUp = True
            return float(z)
        
        if(self.ArmUp and not self.ChargeBall):
            z = 2.0
            self.ArmUp = self.GotBall = self.ChargeBall = False
            return float(z)
        
    def sent_data(self):  # publisher drive topic
        movement_msg = Twist()
        
        # self.imu()
        self.reset_variable()
        self.receive_data(self.ser)
        
        self.Slide_Transform()
        self.UpDown_Transform()
        
        movement_msg.linear.x, movement_msg.linear.y, movement_msg.linear.z, movement_msg.angular.x = self.MoveRobot()
        self.sent_drive.publish(movement_msg)
        
        gripper_msg = Twist()
        x = self.keepHarvest()
        y = self.keepBall()
        z = self.AdjustArm()
        gripper_msg.linear.x = float(x) if x else 0.0
        gripper_msg.linear.y = float(y) if y else 0.0
        gripper_msg.linear.z = float(z) if z else 0.0
        gripper_msg.angular.z = self.SpinBallSpeed
        self.sent_gripper.publish(gripper_msg)

def main():
    rclpy.init()

    sub = Zigbee()    
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

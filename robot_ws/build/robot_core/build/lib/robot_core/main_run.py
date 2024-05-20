import rclpy
import math
import numpy as np

import pygame
import serial
import time

from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy import qos


class Zigbee(Node):
    def __init__(self):
        super().__init__("xbox_control_node")
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
        
        self.sent_drive = self.create_publisher(
            Twist, "moveMotor", qos_profile=qos.qos_profile_system_default
        )
        self.sent_gripper = self.create_publisher(
            Twist, "gripper", qos_profile=qos.qos_profile_system_default
        )
        self.sent_drive_timer = self.create_timer(0.03, self.sent_data)
        # self.sent_gripper_timer = self.create_timer(0.05, self.sent_gripper_callback)
        
        self.ser = self.initialize_serial('/dev/ttyUSB0', 230400)
        
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
        current_time = time.time()
        expected_data_length = 40
        have_data_from_controller = ser.in_waiting
        if have_data_from_controller > 0:
            # reset_variables()
            self.last_data_time = current_time
            received_data = ser.readline().strip().decode()
            if expected_data_length <= len(received_data) <= 80:
                self.last_time = current_time
                tokens = received_data.split(",")
                index = 0
                sum_of_data = 0
                for token in tokens[:21]:
                    data = int(token)
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
                    # compare_data(current_time)
                    self.update_last_data()
                    return
                print("Checksum mismatchnot ")
                ser.readline().decode('utf-8').rstrip()
                self.use_last_data()
                return
            print("Incomplete data receivednot ")
            ser.readline().decode('utf-8').rstrip()
            self.use_last_data()
            return
        if current_time - self.last_data_time < 300:
            have_data_from_controller = True
            self.use_last_data()
            return
        have_data_from_controller = False
        
    
    # def start:
    def MoveRobot(self):
        maxSpeed = 1023.0
        NormalSpeed = 1.0
        turnSpeed = 0.5
        lx =  self.lx * NormalSpeed
        ly =  self.ly * NormalSpeed
        rx =  self.rx * turnSpeed

        # lx = gamepad.Dpad_left  ? -0.75 : 
        #     gamepad.Dpad_right ?  0.75 : lx
        # ly = gamepad.Dpad_down  ? -0.75 :
        #     gamepad.Dpad_up    ?  0.75 : ly

        # setpoint = yaw

        D = max(abs(lx)+abs(ly)+abs(rx), 1.0)

        motor1Speed = float("{:.1f}".format((ly + lx + rx) / D * maxSpeed))
        motor2Speed = float("{:.1f}".format((ly - lx - rx) / D * maxSpeed))
        motor3Speed = float("{:.1f}".format((ly - lx + rx) / D * maxSpeed))
        motor4Speed = float("{:.1f}".format((ly + lx - rx) / D * maxSpeed))
        return motor1Speed, motor2Speed, motor3Speed, motor4Speed

    def keepHarvest(self):
        x = 0.0
        lb = self.left_bumper
        rb = self.right_bumper
        lt = self.left_trigger  > 0.3
        rt = self.right_trigger > 0.3
        if not(lb or rb or rt or lt) :
            self.K_Pressed = False
            return x
        if self.K_Pressed:
            return x
        if lb and rb:
            x = 1.0
            IS_13_Keep = IS_24_Keep = True
            return x
        if IS_13_Keep :
            if lb or lt :
                if lt :
                    x = 2.0
                    return x
                x = 3.0
                IS_13_Keep = False
                return x
        if IS_24_Keep :
            if rb or rt :
                if rt:
                    x = 4.0
                    return x
                x = 5.0
                IS_24_Keep = False
                return x
            
    def keepBall(self):
        y = 0.0
        CurrentTime = time.time()
        MTime = CurrentTime - self.MacroTime
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
            self.MacroTime = CurrentTime
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
        if self.ChargeBall and self.ArmUp and x: # KickBall
            z = 3.0
            return z
        if (not x) :
            self.X_Pressed = False
            return float(z)
        if (self.X_Pressed) :
            return float(z)
        self.X_Pressed = True
        if (not self.ArmUp and not self.ChargeBall):
            z = 1.0
            self.ArmUp = True
            return float(z)
        
        if(self.ArmUp and not self.ChargeBall):
            z = 2.0
            self.ArmUp = False
            return float(z)
        
    def sent_data(self):  # publisher drive topic
        movement_msg = Twist()
        
        self.reset_variable()
        self.receive_data(self.ser)
        
        movement_msg.linear.x, movement_msg.linear.y, movement_msg.linear.z, movement_msg.angular.x = self.MoveRobot()
        self.sent_drive.publish(movement_msg)
        
        gripper_msg = Twist()
        gripper_msg.linear.x = self.keepHarvest()
        gripper_msg.linear.y = self.keepBall()
        gripper_msg.linear.z = self.AdjustArm()
        gripper_msg.angular.z = self.SpinBallSpeed
        self.sent_gripper.publish(gripper_msg)
        

def main():
    rclpy.init()

    sub = Zigbee()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

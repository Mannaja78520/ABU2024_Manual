import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
import pynput.keyboard

from config.config import *
from src.utilize import *

# setup servo
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
        # Control Reset
        
        
    def __init__(self):
        super().__init__("Robot_mainRun_Control_Node")
        self.keyboard_listener = pynput.keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()
        # Emergency variables
        self.EmergencyStop = True
        
        self.Reset(True)
        self.reset_robot_state()
        
        self.sent_drive = self.create_publisher(
            Twist, "moveMotor", qos_profile=qos.qos_profile_system_default
        )
        
        # self.debug = self.create_subscription(
        #     Twist, "debug/motor", self.debug_callback, qos_profile=qos.qos_profile_system_default,
        # )
        
        self.sent_drive_timer = self.create_timer(0.05, self.sent_to_microros)

    # def debug_callback(self, msgin):
    #     return
    
    def reset_robot_state(self):
        # Reset movement and control variables
        self.lx = self.ly = self.rx = 0.0  # Linear and angular velocities
        self.current_time = time.time()  # Current time for calculations
        self.last_rx_time = 0  # Last time a rotation key was pressed
        
    def on_press(self, key):
        # Update movement variables based on pressed keys
        if key == pynput.keyboard.Key.esc:
            self.emergency_stop = True
            return False  # Stop listener on pressing Esc

        # Movement control with WASD keys
        self.lx = 1 if key == pynput.keyboard.KeyCode.from_char('a') else (-1 if key == pynput.keyboard.KeyCode.from_char('d') else self.lx)
        self.ly = 1 if key == pynput.keyboard.KeyCode.from_char('w') else (-1 if key == pynput.keyboard.KeyCode.from_char('s') else self.ly)

        # Rotation control with F and J keys
        self.rx = 1 if key == pynput.keyboard.KeyCode.from_char('f') else (-1 if key == pynput.keyboard.KeyCode.from_char('j') else self.rx)
        self.last_rx_time = self.current_time if self.rx != 0 else self.last_rx_time
        
        if key == pynput.keyboard.Key.space:
            self.reset_robot_state()
        
    def MoveRobot(self):
        
        D = max(abs(self.lx) + abs(self.ly) + abs(self.rx), 1)

        motor1Speed = (self.ly + self.lx - self.rx) / D * maxSpeed
        motor2Speed = (self.ly + self.lx + self.rx) / D * maxSpeed
        motor3Speed = (self.ly - self.lx - self.rx) / D * maxSpeed
        motor4Speed = (self.ly - self.lx + self.rx) / D * maxSpeed
        return motor1Speed, motor2Speed, motor3Speed, motor4Speed
    
    def sent_to_microros(self):  # publisher drive topic
        movement_msg = Twist()

        # if self.EmergencyStop :
        #     print (imu.gz)
        movement_msg.linear.x, movement_msg.linear.y, movement_msg.linear.z, movement_msg.angular.x = self.MoveRobot()
        movement_msg.angular.y = float(self.ISBallSpin)
        movement_msg.angular.z = SpinBallSpeed
        
        # print(lgpio.gpio_read(h, emergency_pin))
        
        self.sent_drive.publish(movement_msg)

def main():
    rclpy.init()

    sub = mainRun()   
    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

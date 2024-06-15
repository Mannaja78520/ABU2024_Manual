import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pynput.keyboard

class KeyboardListener(Node):

    def __init__(self):
        super().__init__('keyboard_listener')
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.keyboard_listener = pynput.keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()
        self.running = True  # Flag to track running state
        self.linear_x = 0.0  # Default linear velocity
        self.angular_z = 0.0  # Default angular velocity

    def on_press(self, key):
        if key == pynput.keyboard.Key.esc:
            self.running = False
            self.keyboard_listener.stop()
            rclpy.shutdown()
            return

        # Movement controls with arrow keys (modify speeds as needed)
        if key == pynput.keyboard.Key.up:
            self.linear_x = 1.0  # Forward
        elif key == pynput.keyboard.Key.down:
            self.linear_x = -1.0  # Backward
        elif key == pynput.keyboard.Key.left:
            self.angular_z = 1.0  # Turn left
        elif key == pynput.keyboard.Key.right:
            self.angular_z = -1.0  # Turn right

        # Publish Twist message with updated velocities
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_x
        twist_msg.angular.z = self.angular_z
        self.twist_pub.publish(twist_msg)

    def run(self):
        while self.running:
            rclpy.spin_once(self)  # Check for messages and callbacks
            # Optional short delay for better responsiveness (remove in final version)
            # time.sleep(0.1)

def main():
    rclpy.init()
    node = KeyboardListener()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

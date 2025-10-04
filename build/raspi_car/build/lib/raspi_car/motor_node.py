#!/usr/bin/env python3
import rclpy                           # ROS2 Python client library
from rclpy.node import Node            # Base class for creating ROS2 nodes
from geometry_msgs.msg import Twist    # Message type for velocity commands
from gpiozero import Motor             # Library to control motors on Raspberry Pi
from gpiozero import PWMOutputDevice
enA = PWMOutputDevice(24)
enB = PWMOutputDevice(25)
class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")
        self.motor_right = Motor(forward=22, backward=23)
        self.motor_left = Motor(forward=17, backward=27)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.get_logger().info("Motor node started. Listening to /cmd_vel")
    def listener_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        speed = min(abs(linear_x), 1.0)
        if linear_x > 0:   # Forward
            self.motor_right.forward()
            self.motor_left.forward()
            enA.value = speed
            enB.value = speed
        elif linear_x < 0: # Backward
            self.motor_right.backward()
            self.motor_left.backward()
            enA.value = speed
            enB.value = speed
        elif angular_z > 0: # Turn left
            self.motor_right.forward()
            self.motor_left.backward()
            enA.value = min(0.4*abs(angular_z), 0.5)
            enB.value = min(0.4*abs(angular_z), 0.5)
        elif angular_z < 0: # Turn right
            self.motor_right.backward()
            self.motor_left.forward()
            enA.value = min(0.4*abs(angular_z), 0.5)
            enB.value = min(0.4*abs(angular_z), 0.5)
        else: # Stop
            self.motor_right.stop()
            self.motor_left.stop()
            enA.value = 0
            enB.value = 0
def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
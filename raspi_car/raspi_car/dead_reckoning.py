#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from gpiozero import Button
import math
import tf_transformations

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        # --- Encoder setup ---
        self.left_tick = Button(26)
        self.left_dir = Button(19)
        self.right_tick = Button(13)
        self.right_dir = Button(6)

        # --- Wheel/robot parameters ---
        self.TICKS_PER_REV = 160
        self.WHEEL_DIAMETER = 0.060   # meters
        self.WHEEL_BASE = 0.135       # meters (distance between wheels)
        self.DIST_PER_TICK = math.pi * self.WHEEL_DIAMETER / self.TICKS_PER_REV

        # --- Odometry variables ---
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw_deg = 0.0            # heading in degrees
        self.initial_yaw = None       # used to normalize heading

        # --- ROS2 Subscription for IMU ---
        self.create_subscription(Imu, '/bno055/imu', self.imu_callback, 10)

        # --- Encoder interrupts ---
        self.left_tick.when_pressed = self.left_tick_handler
        self.right_tick.when_pressed = self.right_tick_handler

        # --- Timer for console updates (10 Hz) ---
        self.timer = self.create_timer(0.1, self.update_position)

        self.get_logger().info("🚗 Odometry node started (Encoder + IMU)")

    def imu_callback(self, msg: Imu):
        """Converts quaternion → yaw in degrees (0–360°)"""
        q = msg.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)

        yaw_deg = math.degrees(yaw)
        yaw_deg = 180 - yaw_deg
        if yaw_deg < 0:
            yaw_deg += 360.0

        # Compute relative heading from start (0–360°)
        self.yaw_deg = (yaw_deg)% 360.0

    def left_tick_handler(self):
        """Count ticks for left wheel"""
        if self.left_dir.is_pressed:
            self.left_distance += self.DIST_PER_TICK
        else:
            self.left_distance -= self.DIST_PER_TICK

    def right_tick_handler(self):
        """Count ticks for right wheel"""
        if self.right_dir.is_pressed:
            self.right_distance += self.DIST_PER_TICK
        else:
            self.right_distance -= self.DIST_PER_TICK

    def update_position(self):
        """Integrate wheel motion + heading into position"""
        dL = self.left_distance
        dR = self.right_distance
        self.left_distance = 0.0
        self.right_distance = 0.0

        ds = (dL + dR) / 2.0  # average forward distance
        theta = math.radians(self.yaw_deg)  # convert heading to radians

        # Update position in world coordinates
        self.y += ds * math.cos(theta)
        self.x += ds * math.sin(theta)

        self.get_logger().info(
            f"X={self.x:.3f} m, Y={self.y:.3f} m, Heading={self.yaw_deg:.1f}°"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

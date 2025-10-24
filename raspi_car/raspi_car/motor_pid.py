#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from gpiozero import Motor, PWMOutputDevice
import time

# Motor enable pins
enA = PWMOutputDevice(24)
enB = PWMOutputDevice(25)


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        # Motors
        self.motor_right = Motor(forward=22, backward=23)
        self.motor_left = Motor(forward=17, backward=27)

        # PID parameters
        self.kp = -0.8
        self.ki = -1.5
        self.kd = 0
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

        # State variables
        self.current_ang_vel_z = 0.0
        self.target_ang_vel_z = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0

        # ROS2 subscriptions
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Imu, "/bno055/imu", self.imu_callback, 10)

        # Create a timer for continuous control updates (e.g., 20 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)

        self.get_logger().info("Motor node started with continuous IMU PID correction loop.")

    def imu_callback(self, msg: Imu):
        """Get angular velocity from IMU"""
        self.current_ang_vel_z = msg.angular_velocity.z

    def cmd_vel_callback(self, msg: Twist):
        """Update target velocity"""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def pid_correction(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 1.0
        self.last_time = current_time

        error = self.target_ang_vel_z - self.current_ang_vel_z

        # Conditional integral update (anti-windup)

        if (self.integral < 0.3 and self.integral > -0.3) or \
                (self.integral >= 0.3 and error > 0) or \
                (self.integral <= -0.3 and error < 0):
            new_integral = self.integral + error * dt
            self.integral = new_integral

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        correction = 1.0 + (self.kp * error + self.ki * self.integral + self.kd * derivative)
        correction = max(0.2, min(1.7, correction))
        return correction

    def control_loop(self):
        """Runs continuously to apply motor commands"""
        speed = min(abs(self.linear_x), 1.0)

        if self.linear_x != 0:
            correction = self.pid_correction()


            if self.linear_x > 0:  # Forward
                self.motor_right.forward()
                self.motor_left.forward()
                right_speed = max(0.0, min(1.0, speed * correction))
                left_speed = max(0.0, min(1.0, speed / correction))
            else:  # Backward
                self.motor_right.backward()
                self.motor_left.backward()
                right_speed = max(0.0, min(1.0, speed * correction))
                left_speed = max(0.0, min(1.0, speed / correction))

            enA.value = right_speed
            enB.value = left_speed

            self.get_logger().info(
                f"[Drive] IMU z={self.current_ang_vel_z:.3f}, Corr={correction:.3f}, L={right_speed:.2f}, R={left_speed:.2f}"
            )

        elif self.angular_z > 0:  # Manual turn left
            self.motor_right.forward()
            self.motor_left.backward()
            enA.value = min(0.4 * abs(self.angular_z), 0.5)
            enB.value = min(0.4 * abs(self.angular_z), 0.5)

        elif self.angular_z < 0:  # Manual turn right
            self.motor_right.backward()
            self.motor_left.forward()
            enA.value = min(0.4 * abs(self.angular_z), 0.5)
            enB.value = min(0.4 * abs(self.angular_z), 0.5)

        else:  # Stop
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


if __name__ == "__main__":
    main()

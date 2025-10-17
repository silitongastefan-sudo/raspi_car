import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import lgpio


class BNO055Logger(Node):
    def __init__(self):
        super().__init__('meow')
        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        ang_vel_z = msg.angular_velocity.z
        orient_w = msg.orientation.w
        self.get_logger().info(f'Angular Velocity Z: {ang_vel_z:.4f},Orientation W: {orient_w:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Logger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

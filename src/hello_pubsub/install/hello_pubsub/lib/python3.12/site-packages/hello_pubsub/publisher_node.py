import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class HelloPublisher(Node):
    def __init__(self):
        super().__init__("hello_publisher")
        self.publisher_=self.create_publisher(String, 'hello_topic',10)
        self.timer = self.create_timer(1.0,self.timer_callback)
        self.count = 0
    def timer_callback(self):
        msg = String()
        msg.data = "Hello ROS2! Count:" + str(self.count)
        self.get_logger.info(f"publishing:'{msg.data}'")
        self.count+=1
def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
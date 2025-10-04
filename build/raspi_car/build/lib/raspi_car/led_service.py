import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
class LedServiceServer(Node):
    def __init__(self):
        super().__init__('led_service_server')
        self.led_on= False
        self.srv = self.create_service(SetBool, 'toggle_led', self.toggle_led_callback)
        self.publisher_ = self.create_publisher(Bool, 'led_state',10)
        self.get_logger().info('led service ready, call "toggle_led" with True/False')
        self.get_logger().info('ros2 service call /toggle_led std_srvs/srv/SetBool "{data: true}"')
        self.timer = self.create_timer(1.0, self.publish_message)
    def toggle_led_callback(self, request, response):
        self.led_on = request.data
        response.success = True
        response.message = "Led is now " + str(self.led_on)
        msg = Bool()
        msg.data = self.led_on
        self.publisher_.publish(msg)
        return response
    def publish_message(self):
        msg = Bool()
        msg.data = self.led_on
        self.publisher_.publish(msg)
        self.get_logger().info('topic message: ' + str(msg))
def main(args=None):
    rclpy.init(args=args)
    node = LedServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
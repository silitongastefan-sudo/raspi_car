import rclpy
from rclpy.node import Node
from std_srvs.srv import Setbool

class LedServiceServer(Node):
    def __init__(self):
        super().__init__('led_service_server')
        self.led_on= False
        self.srv = self.create_service(Setbool, 'toggle_led', self.toggle_led_callback)
        self.get_logger().info('led service ready, call "toggle_led" with True/False')
    def toggle_led_callback(self, request, response):
        self.led_on = request.data
        response.success = True
        response.message = "Led is now " + str(self.led_on)
        self.get_logger().info(response.message)
        return response
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
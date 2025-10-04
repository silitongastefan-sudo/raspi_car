import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from gpiozero import LED
from time import sleep

led = LED(15)
class led_control(Node):
    def __init__(self):
        super().__init__('led_control')
        self.subscription = self.create_subscription (Bool, 'led_state', self.led,10)
        self.get_logger().info("listening to led_state")
    def led(self, msg):
        if msg.data == True:
            self.get_logger().info("led_state" + str(msg.data))
            self.led_strobe()
        else:
            led.off()
            self.get_logger().info("led_state is off")
    def led_strobe(self):
        led.off()
        sleep(1)
        led.on()
        sleep(0.1)
        led.off()
        sleep(0.1)
        led.on()
        sleep(0.1)
        led.off()


def main(args=None):
    rclpy.init(args=args)
    node = led_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        led.off()


if __name__ == '__main__':
    main()

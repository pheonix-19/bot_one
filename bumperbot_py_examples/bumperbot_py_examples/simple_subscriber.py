import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SimpleSubscriber(Node):
    
        def __init__(self):
            super().__init__('simple_subscriber')
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning
    
        def listener_callback(self, msg):
            self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
    node = SimpleSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
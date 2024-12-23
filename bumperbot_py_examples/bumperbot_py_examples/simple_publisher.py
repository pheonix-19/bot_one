import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simplle_publisher")
        
        # Create the publisher
        self.pub = self.create_publisher(String, "chatter", 10)
        
        # Initialize the counter and frequency
        self.counter_ = 0
        self.frequency_ = 1.0
        
        # Log the publishing frequency
        self.get_logger().info(f"Publishing at {self.frequency_} Hz")
        
        # Create the timer and specify the callback
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        # Create a new message
        msg = String()
        msg.data = f"Hello Ayush, counter: {self.counter_}"
        
        # Publish the message
        self.pub.publish(msg)
        
        # Increment the counter
        self.counter_ += 1

def main():
    # Initialize the ROS 2 client library
    rclpy.init()
    
    # Create the SimplePublisher node
    simple_publisher = SimplePublisher()
    
    # Spin to keep the program running and processing callbacks
    rclpy.spin(simple_publisher)
    
    # Clean up the node and shutdown ROS 2
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

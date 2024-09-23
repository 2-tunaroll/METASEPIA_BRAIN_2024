import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from custom_msgs.msg import CMD

class controller_node(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(CMD, 'CMD', 10)
        
        
    def joy_callback(self, msg):
        out = CMD()
        out.surge   = msg.axes[1]    
        out.sway    = msg.axes[0]   
        out.pitch   = msg.axes[4]   
        out.yaw     = msg.axes[3]    
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = controller_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
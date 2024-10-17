import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from custom_msgs.msg import CMD

MAX_AMPLITUDE = 70.0
MIN_AMPLITUDE = 0.0

class controller_node(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(CMD, 'CMD', 10)
        self.amplitude = 40.0
        
         
    def joy_callback(self, msg):

        # increase amplitude
        if (msg.buttons[2]):
            self.amplitude += 5.0

        # decrease amplitude
        if (msg.buttons[0]):
            self.amplitude -= 5.0

        if (self.amplitude > MAX_AMPLITUDE): 
            self.amplitude = MAX_AMPLITUDE
        
        if (self.amplitude < MIN_AMPLITUDE):
            self.amplitude = MIN_AMPLITUDE

        out = CMD()
        
        out.surge       = msg.axes[1]    
        out.sway        = -msg.axes[0]   
        out.pitch       = msg.axes[4]   
        out.yaw         = -msg.axes[3]

        if not (msg.axes[2] < -0.9 and msg.axes[5] < -0.9):
            out.surge = 0.0
            out.sway = 0.0
            out.pitch = 0.0
            out.yaw = 0.0

        out.amplitude   = self.amplitude
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = controller_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
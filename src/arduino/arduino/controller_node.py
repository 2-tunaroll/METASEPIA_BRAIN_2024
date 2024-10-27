import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from custom_msgs.msg import CMD

MAX_AMPLITUDE = 70.0
MIN_AMPLITUDE = 0.0

MAX_SPEED = 1.0
MIN_SPEED = 0.0

MAX_MODE = 3

AUTOREPEAT_DELAY = 0.2

def clamp(value, MAX, MIN):
    if (value > MAX):
        value = MAX
    if (value < MIN):
        value = MIN
    return value

class controller_node(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(CMD, 'CMD', 10)
        self.amplitude = 40.0
        self.speed_multiplier = 1.0
        self.mode = 0

        self.last_button_pressed_time = {}
         
    def joy_callback(self, msg):
        current_time = time.monotonic()

        for button_index, action in [(1, self.increase_speed), (3, self.decrease_speed),
                                     (2, self.increase_amplitude), (0, self.decrease_amplitude),
                                     (9, self.change_mode)]:
            if msg.buttons[button_index]:
                last_press = self.last_button_pressed_time.get(button_index, 0)
                if current_time - last_press >= AUTOREPEAT_DELAY:
                    action()
                    self.last_button_pressed_time[button_index] = current_time
        
        out = CMD()
        out.mode    = self.mode
        out.surge   =  msg.axes[1]      * self.speed_multiplier
        out.sway    = -msg.axes[0]      * self.speed_multiplier
        out.pitch   =  msg.axes[4]      * self.speed_multiplier
        out.yaw     = -msg.axes[3]      * self.speed_multiplier

        if not (msg.axes[2] < -0.9 and msg.axes[5] < -0.9):
            out.surge   = 0.0
            out.sway    = 0.0
            out.pitch   = 0.0
            out.yaw     = 0.0

        out.amplitude           = self.amplitude
        out.speed_multiplier    = self.speed_multiplier
        
        self.publisher.publish(out)

    def increase_speed(self):
        self.speed_multiplier = clamp(self.speed_multiplier + 0.1, MAX_SPEED, MIN_SPEED)

    def decrease_speed(self):
        self.speed_multiplier = clamp(self.speed_multiplier - 0.1, MAX_SPEED, MIN_SPEED)

    def increase_amplitude(self):
        self.amplitude = clamp(self.amplitude + 5.0, MAX_AMPLITUDE, MIN_AMPLITUDE)
    
    def decrease_amplitude(self):
        self.amplitude = clamp(self.amplitude - 5.0, MAX_AMPLITUDE, MIN_AMPLITUDE)
    
    def change_mode(self):
        self.mode = (self.mode + 1) % MAX_MODE

def main(args=None):
    rclpy.init(args=args)
    node = controller_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
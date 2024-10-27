import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from custom_msgs.msg import CMD, BatteryState

import serial
import numpy as np
import bisect

HEADER_VAL = 42

def float_to_uint8(value, min_value=-1.0, max_value=1.0):
    # Clamp the value to the min and max range
    clamped_value = max(min_value, min(max_value, value))
    # Scale to the range of int8
    scaled_value = int(np.round((clamped_value - min_value) * 255 / (max_value - min_value)))
    # Convert to np.int8 and return
    return np.uint8(scaled_value)

class arduino_node(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.subscription = self.create_subscription(CMD, 'CMD', self.cmd_callback, 10)
        self.publisher = self.create_publisher(BatteryState, 'BatteryState', 10)
        self.serial_port = serial.Serial(
                                port = '/dev/ttyACM0',
                                baudrate = 115200,
                                timeout = 1
                            )
        self.mode = 0
        self.pct = [  0,   5,  10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  60,
        65,  70,  75,  80,  85,  90,  95, 100]
        self.vtg = [ 9.82, 10.83, 11.06, 11.12, 11.18, 11.24, 11.3 , 11.36, 11.39,
       11.45, 11.51, 11.56, 11.62, 11.74, 11.86, 11.95, 12.07, 12.25,
       12.33, 12.45, 12.6 ]
    
    def voltage_to_battery_charge(self, voltage):
        if voltage < 9.8:
            return -1.0
        
        if voltage >= 12.6:
            return 100.0
        
        i = bisect.bisect_right(self.vtg, voltage) - 1
        
        return  self.pct[i] \
            + (voltage - self.vtg[i]) \
            * (self.pct[i+1] - self.pct[i])/(self.vtg[i+1] - self.vtg[i])
        
    
    def cmd_callback(self, msg):
        message = (
            HEADER_VAL,
            msg.mode,                   # type = instruction
            float_to_uint8(msg.surge),    
            float_to_uint8(msg.sway),    
            float_to_uint8(msg.pitch),    
            float_to_uint8(msg.yaw),
            np.uint8(msg.amplitude)
        )

        self.serial_port.write(message)

        # Recieve response and update voltage topic
        if self.serial_port.in_waiting >= 3:
            status = self.serial_port.read()
            status_val = int.from_bytes(status, byteorder='big')

            if status_val == 0b1:
                v_high_byte = ord(self.serial_port.read())
                v_low_byte  = ord(self.serial_port.read())

                voltage = (v_high_byte << 8 | v_low_byte) * (12.6 / 1023) # recombine voltage and rescale for 12.6 max voltage
            
                msg = BatteryState()
                msg.voltage = voltage
                msg.charge = self.voltage_to_battery_charge(voltage)
                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = arduino_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import serial
import numpy as np

MESSAGE_TYPE = 1

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
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(String, 'test', 10)
        self.serial_port = serial.Serial(
                                port = '/dev/ttyACM0',
                                baudrate = 115200,
                                timeout = 1
                            )
        self.pct = [  0,   5,  10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  60,
        65,  70,  75,  80,  85,  90,  95, 100]
        self.vtg = [ 9.82, 10.83, 11.06, 11.12, 11.18, 11.24, 11.3 , 11.36, 11.39,
       11.45, 11.51, 11.56, 11.62, 11.74, 11.86, 11.95, 12.07, 12.25,
       12.33, 12.45, 12.6 ]
    
    def voltage_to_battery_charge(self, voltage):
        i = 0
        
        for v in self.vtg:
            if voltage > v or i == 20:
                break
            i = i + 1
        
        pct =  self.vtg[i] \
            + (self.vtg[i+1] - voltage) \
            * (self.pct[i+1] - self.pct[i])/(self.vtg[i+1] - self.vtg[i])
        
        return i
    
    def joy_callback(self, msg):
        # Send instruction over serial
        message = (
            MESSAGE_TYPE,                   # type = instruction
            float_to_uint8(msg.axes[1]),    # surge
            float_to_uint8(msg.axes[0]),    # sway
            float_to_uint8(msg.axes[4]),    # pitch
            float_to_uint8(msg.axes[3])     # yaw
        )
        self.serial_port.write(message)

        # Recieve response and update voltage topic
        if self.serial_port.in_waiting >= 6:
            status = self.serial_port.read()
            status_val = int.from_bytes(status, byteorder='big')

            if status_val == 0b1:
                data = self.serial_port.read(5)  

                # Extract each byte and interpret as uint8
                voltage = data[0] * 12.6 / 256
                surge = data[1]
                sway = data[2]
                pitch = data[3]
                yaw = data[4]

                charge = self.voltage_to_battery_charge(voltage)
            
                msg = String()
                msg.data = f"Voltage: {voltage}, Charge: {charge}, Surge: {surge}, Sway: {sway}, Pitch: {pitch}, Yaw: {yaw}"
                self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = arduino_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
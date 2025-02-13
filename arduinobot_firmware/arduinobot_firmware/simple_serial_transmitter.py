#!/usr/bin/python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.port_ = self.get_parameter("port").value
        self.baud_rate_ = self.get_parameter("baud_rate").value

        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baud_rate_, timeout=0.1)


    def msgCallback(self, msg):
        self.get_logger().info(f"New message received, publishing on port: {self.arduino_.name}")
        self.arduino_.write(msg.data.encode("utf-8"))



def main():
    rclpy.init()
    node = SimpleSerialTransmitter()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
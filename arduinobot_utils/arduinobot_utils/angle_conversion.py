#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arduinobot_interfaces.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class AngleConversion(Node):
    def __init__(self):
        super().__init__("angle_conversion")

        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulertoquaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaterniontoeulerCallback)
        self.get_logger().info("Angle conversion services are ready")
    
    def eulertoquaternionCallback(self, req, res):
        self.get_logger().info("Requested to convert euler angles roll: %f, pitch: %f, yaw: %f, into a quaternion." % (req.roll, req.pitch, req.yaw))
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        self.get_logger().info("Corresponding quaternion x: %f, y: %f, z: %f, w: %f" % (res.x, res.y, res.z, res.w))
        return res

    def quaterniontoeulerCallback(self, req, res):
        self.get_logger().info("Requested to convert quaternion x: %f, y: %f, z: %f, w: %f, into euler angles" % (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info("Corresponding euler angles roll: %f, pitch: %f, yaw: %f." % (res.roll, res.pitch, res.yaw)) 
        return res
    
def main():
    rclpy.init()
    node = AngleConversion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
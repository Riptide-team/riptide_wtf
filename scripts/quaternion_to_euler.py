#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import numpy as np
from scipy.spatial.transform import Rotation


class QuaternionToEuler(Node):

    def __init__(self):
        super().__init__('riptide_wtf')
        self.imu_subscription = self.create_subscription(Imu, '/riptide_1/imu_broadcaster/imu_status', self.imu_callback, 10)

    def imu_callback(self, msg):
        angles = Rotation.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]).as_euler('zyx', degrees=True)

        self.get_logger().info(f"Angles {angles}")


def main(args=None):
    rclpy.init(args=args)
    quaternion_to_euler = QuaternionToEuler()
    rclpy.spin(quaternion_to_euler)

if __name__ == '__main__':
    main()

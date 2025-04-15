#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import math
import yaml
import os

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Wczytanie parametrów z pliku YAML
        self.load_robot_params()

        # Sub i pub
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)


    def load_robot_params(self):
        try:
            pkg_share = get_package_share_directory('my_robot_rviz')  # to działa również po instalacji
            yaml_path = os.path.join(pkg_share, 'config', 'robot_params.yaml')
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                links = data['links']
                self.L1 = links['link2_length']
                self.L2 = links['link3_length']
                self.h = links['base_height'] + links['joint_offset1'] + links['joint_offset2']
        except Exception as e:
            self.get_logger().error(f"Błąd wczytywania YAML: {e}")
            raise

    def point_callback(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        try:
            joint_positions = self.calculate_ik(x, y, z)
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = [
                "base_to_link1",
                "link1_to_link2",
                "link2_to_link3",
                "link3_to_link4",
                "link4_to_link5"
            ]
            joint_msg.position = joint_positions
            self.joint_pub.publish(joint_msg)
            self.get_logger().info(f"Published joint state for target: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        except Exception as e:
            self.get_logger().warn(f"IK error: {e}")

    def calculate_ik(self, x, y, z):
        # Joint 1: obrót wokół osi Z
        theta1 = math.atan2(y, x)

        # Pozycja w płaszczyźnie XY
        r = math.sqrt(x**2 + y**2)
        dz = z - self.h
        D = math.sqrt(r**2 + dz**2)

        # Sprawdzenie zasięgu
        if D > (self.L1 + self.L2):
            raise ValueError("Poza zasięgiem ramienia")

        # Geometryczna metoda
        alpha = math.acos((self.L1**2 + D**2 - self.L2**2) / (2 * self.L1 * D))
        beta = math.acos((self.L1**2 + self.L2**2 - D**2) / (2 * self.L1 * self.L2))

        theta2 = math.atan2(dz, r) - alpha
        theta3 = math.pi - beta

        # Prosto: ustalmy pozostałe jako zero
        theta4 = 0.0
        theta5 = 0.0

        return [theta1, theta2, theta3, theta4, theta5]


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

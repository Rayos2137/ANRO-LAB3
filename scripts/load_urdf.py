#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
from robot_state_publisher.robot_state_publisher import RobotStatePublisher
from urdf_parser_py.urdf import URDF

class URDFLoader(Node):
    def __init__(self):
        super().__init__('urdf_loader')

        # Wczytanie pliku URDF
        urdf_file = os.path.join(
            get_package_share_directory('my_robot_py'), 'urdf', 'my_robot.urdf'
        )
        with open(urdf_file, 'r') as f:
            robot_desc = f.read()

        # Parsowanie URDF
        self.robot_model = URDF.from_xml_string(robot_desc)
        self.get_logger().info(f'Robot name: {self.robot_model.name}')

        # Publikowanie modelu
        self.robot_publisher = RobotStatePublisher(self)
        self.robot_publisher.set_robot_description(robot_desc)

def main(args=None):
    rclpy.init(args=args)
    node = URDFLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

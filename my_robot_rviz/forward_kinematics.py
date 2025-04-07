#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.publisher = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'tool_frame', now)
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            
            pose_msg.pose.position.x = trans.transform.translation.x
            pose_msg.pose.position.y = trans.transform.translation.y
            pose_msg.pose.position.z = trans.transform.translation.z
            pose_msg.pose.orientation = trans.transform.rotation
            
            self.publisher.publish(pose_msg)
            
            self.get_logger().info(f"Pozycja efektora: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
        
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Czekam na TF: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

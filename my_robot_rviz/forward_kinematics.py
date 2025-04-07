#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_transformations  # Biblioteka do konwersji kątów Eulera na kwaterniony

# Długości segmentów ramienia
L1 = 0.05  # Wysokość podstawy
L2 = 0.135 # Długość pierwszego ramienia
L3 = 0.147  # Długość drugiego ramienia
L4 = 0.06 # Długość trzeciego ramienia (przed chwytakiem)

def forward_kinematics(joint_angles):
    """
    Oblicza pozycję efektora końcowego w przestrzeni 3D oraz orientację.
    """
    theta1, theta2, theta3, theta4, theta5 = joint_angles

    # Konwersja kątów na radiany
    t1 = np.radians(theta1)  # Joint 1 - obrót wokół osi Z
    t2 = np.radians(theta2)  # Joint 2 - obrót wokół osi X
    t3 = np.radians(theta3)  # Joint 3 - obrót wokół osi X
    t4 = np.radians(theta4)  # Joint 4 - obrót wokół osi X
    t5 = np.radians(theta5)  # Joint 5 - obrót wokół osi Z (nie wpływa na pozycję, tylko orientację)

    # Obliczenie pozycji efektora
    x = ((L2 * np.cos(t2) + L3 * np.cos(t2 + t3) + L4 * np.cos(t2 + t3 + t4)) * np.cos(t1))
    y = (L2 * np.cos(t2) + L3 * np.cos(t2 + t3) + L4 * np.cos(t2 + t3 + t4)) * np.sin(t1)
    z = (L1 + L2 * np.sin(t2) + L3 * np.sin(t2 + t3) + L4 * np.sin(t2 + t3 + t4))

    # Obliczanie orientacji w postaci kwaternionu
    roll = t2 + t3 + t4  # Przykładowe obroty (proste założenie)
    pitch = t5  # Prosty przykład użycia theta5
    yaw = 0  # Zakładam, że yaw jest zerowy, ale można go dostosować, w zależności od stawów

    # Konwersja kątów Eulera na kwaterniony
    quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

    return x, y, z, quat

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        self.get_logger().info('ForwardKinematicsNode initialized.')  # Dodane logowanie

        # Subskrypcja pozycji stawów
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,  # Upewnij się, że funkcja jest przypisana tutaj
            10)
        
        # Publikacja pozycji efektora
        self.publisher = self.create_publisher(PoseStamped, '/end_effector_pose', 10)

    def joint_callback(self, msg):
        """
        Pobiera aktualne kąty w stawach i publikuje pozycję efektora.
        """
        if len(msg.position) < 5:
            self.get_logger().warn("Niekompletne dane z /joint_states, czekam...")
            return

        joint_angles = msg.position[:5]  # Zbieranie tylko 5 pierwszych kątów

        # Obliczanie pozycji efektora i orientacji
        x, y, z, quat = forward_kinematics(joint_angles)

        # Tworzenie wiadomości o pozycji efektora
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        # Ustawianie orientacji w postaci kwaternionu
        pose_msg.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        # Publikowanie pozycji efektora
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Pozycja efektora: \nX: {x}\nY: {y}\nZ: {z}\nOrientacja: {quat}")
        

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

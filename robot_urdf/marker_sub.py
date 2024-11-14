"""
Main assignment 1 node
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray
from ros2_aruco_interfaces.msg import ArucoMarkers


class MarkerClass_Subscriber (Node):
    def __init__(self):
        super().__init__('marker_sub')
        self.aruco_marker = None
        self.subscription_marker = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10)
        self.subscription_marker  # prevent unused variable warning
        """
        self.marker_pose = self.create_subscription(
            PoseArray,
            'aruco_poses',
            self.pose_marker_callback,
            10)
        self.marker_pose
        """

    def aruco_marker_callback(self, msg_marker):
        # self.get_logger().info(f'ID: {msg_marker.marker_ids[-1]}')
        self.aruco_marker = msg_marker
    """
    def pose_marker_callback(self, msg_pose):
        self.pose_marker = msg_pose
    """

def main ():
    rclpy.init()
    node = MarkerClass_Subscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
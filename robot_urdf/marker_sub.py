"""
Main assignment 1 node
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers


class MarkerClass_Subscriber (Node):
    def __init__(self):
        super().__init__('marker_sub')
        self.marker_id = 0
        self.marker_pose = Pose()
        self.subscription_marker = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10)
        self.subscription_marker  # prevent unused variable warning

    def aruco_marker_callback(self, msg_marker):
        self.marker_id = msg_marker.marker_ids[-1]
        self.marker_pose = msg_marker.poses[-1]
        self.get_logger().info(f'Received message with marker {self.marker_id} and pose {self.marker_pose}')


def main ():
    rclpy.init()
    node = MarkerClass_Subscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
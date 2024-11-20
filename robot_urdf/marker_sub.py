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
        self.subscription_marker = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10)
        self.subscription_marker  # prevent unused variable warning
        self.marker_id = 0
        self.marker_pose = Pose()
        self.min_marker = 3
        self.detected_markers = ArucoMarkers() #list of all Aruco markers detected 
        self.end_recognition = False           #flag to stop the robot when it returns to the starting marker
        

    def aruco_marker_callback(self, msg_marker):
        self.marker_id = msg_marker.marker_ids[-1]
        self.marker_pose = msg_marker.poses[-1]
        self.robot_control ()

    def robot_control(self):
        if not self.marker_id:
            self.get_logger().info('No marker detected yet')
            
        else:
            if self.marker_id not in self.detected_markers.marker_ids:
                self.get_logger().info(f'marker ID: {self.marker_id}')
                self.detected_markers.marker_ids.append(self.marker_id)
                self.detected_markers.poses.append(self.marker_pose)
                    
            else:
                if len(self.detected_markers.marker_ids) >= self.min_marker:
                    if self.detected_markers.marker_ids[0] == self.marker_id:
                        self.end_recognition = True
        

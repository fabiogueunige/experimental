import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
import numpy as np
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge


class MarkerClass_Subscriber (Node):
    def __init__(self):
        super().__init__('marker_sub')
        self.subscription_marker = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10)
        self.subscription_marker  # prevent unused variable warning
        
        self.subscription_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)
        self.subscription_image  # prevent unused variable warning
        self.current_img = None
        
        """
        self.subscription_corners = self.create_subscription(
            Pose,
            'aruco_corners',
            self.corners_callback,
            10)
        self.subscription_corners
        self.center = Pose()
        """

        self.marker_id = 0
        self.marker_pose = Pose()
        self.min_marker = 3
        self.detected_markers = [] #list of all Aruco markers detected 
        self.end_recognition = False #flag to stop the robot when it returns to the starting marker

    """    
    def corners_callback(self, msg_corners):   
        self.center = msg_corners
        """

    def aruco_marker_callback(self, msg_marker):
        self.marker_id = msg_marker.marker_ids[-1]
        self.marker_pose = msg_marker.poses[-1]
        self.robot_control ()

    def image_callback(self, msg_image):
        self.current_img = msg_image

    def robot_control(self):
        if not self.marker_id:
            self.get_logger().info('No marker detected yet')
            
        else:
            if self.marker_id not in [marker['id'] for marker in self.detected_markers]:
                self.detected_markers.append({
                    'id': self.marker_id,
                    'pose': self.marker_pose,
                    'image': self.current_img,
                    # 'centers': self.center
                })
                    
            else:
                if len(self.detected_markers) >= self.min_marker:
                    if self.detected_markers[0]['id'] == self.marker_id:
                        self.end_recognition = True

    def reorder(self):
        # reorder the markers from the smaller id to the biggest one
        self.detected_markers.sort(key=lambda x: x['id'])
        

        

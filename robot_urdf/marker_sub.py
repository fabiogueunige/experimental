import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from ros2_aruco_interfaces.msg import ArucoMarkers
import numpy as np
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2


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

        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.marker_id = 0
        self.marker_pose = Pose()
        self.min_marker = 3
        self.detected_markers = [] #list of all Aruco markers detected 
        self.end_recognition = False #flag to stop the robot when it returns to the starting marker

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
                cv_image = self.bridge.imgmsg_to_cv2(self.current_img,
                                            desired_encoding='mono8')
                corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                            self.aruco_dictionary,
                                                            parameters=self.aruco_parameters)
                pose_center = Pose()
                pose_center.position.x = (corners[0][0][0][0] + corners[0][0][2][0] + corners[0][0][3][0] + corners[0][0][1][0]) / 4
                pose_center.position.y = (corners[0][0][0][1] + corners[0][0][2][1] + corners[0][0][3][1] + corners[0][0][1][1]) / 4
                # on the z-axis we have the radius
                pose_center.position.z = np.sqrt((pose_center.position.x - corners[0][0][0][0]) ** 2 + (pose_center.position.y - corners[0][0][0][1]) ** 2)
                self.detected_markers.append({
                    'id': self.marker_id,
                    'pose': self.marker_pose,
                    'image': self.current_img,
                    'centers': pose_center
                })
                
                    
            else:
                if len(self.detected_markers) >= self.min_marker:
                    if self.detected_markers[0]['id'] == self.marker_id:
                        self.end_recognition = True

    def reorder(self):
        # reorder the markers from the smaller id to the biggest one
        self.detected_markers.sort(key=lambda x: x['id'])
        
def main():
    rclpy.init()
    node = MarkerClass_Subscriber()
    rclpy.spin(node)
   
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

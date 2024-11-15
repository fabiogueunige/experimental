#  NON SERVE PIÙ, SI PUO FARE UN MARKER SUB CHE VIENE MEGLIO
# E POI UN MARKER PUB CHE PUBBLICA IL MARKER CHE SI VUOLE RAGGIUNGERE
# E INFINE UN CONTROLLORE CHE SI MUOVE VERSO IL MARKER PUBBLICATO


import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Twist, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers



class MarkerClass_Subscriber (Node):
    def __init__(self):
        super().__init__('robot_rotation')
        self.marker_id = 0
        self.marker_pose = Pose()
        self.subscription_marker = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_marker_callback,
            10)
        self.subscription_marker  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.min_marker = 3
        self.detected_markers = ArucoMarkers()
        self.linear = 0.0
        self.angular = 1.0
        

    def aruco_marker_callback(self, msg_marker):
        self.marker_id = msg_marker.marker_ids[-1]
        self.marker_pose = msg_marker.poses[-1]
        self.get_logger().info(f'Received message with marker {self.marker_id} and pose {self.marker_pose}')
        self.robot_control ()

    def send_cmd(self ):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher_.publish(msg)

    def robot_control(self):
        
        #self.get_logger().info(f'Print {self.marker_id}')
        
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
                        self.get_logger().info("Returned to the starting marker. Stopping.")
                        self.angular = 0.0
                        self.send_cmd()
                        return
        
        self.send_cmd()
        

def main():
    rclpy.init()

    marker = MarkerClass_Subscriber()
    rclpy.spin(marker)

    marker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



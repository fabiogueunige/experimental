#  NON SERVE PIÙ, SI PUO FARE UN MARKER SUB CHE VIENE MEGLIO
# E POI UN MARKER PUB CHE PUBBLICA IL MARKER CHE SI VUOLE RAGGIUNGERE
# E INFINE UN CONTROLLORE CHE SI MUOVE VERSO IL MARKER PUBBLICATO


import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
import numpy as np
from geometry_msgs.msg import PoseArray, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers

min_marker = 3
class CmdPublisher(Node):
    def __init__(self):
        super().__init__('robot_rotation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

def robot_control(cmd_pub):
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 0.3

    detected_markers = []

    while rclpy.ok():
        rclpy.spin_once(marker)
        if marker.aruco_marker is None:
            cmd_pub.get_logger().info('No marker detected yet')
            continue
        else:
            if marker.aruco_marker.marker_ids[-1] not in detected_markers.marker_ids:
                cmd_pub.get_logger().info(f'Detected marker ID: {marker.aruco_marker.marker_ids[-1]}')
                detected_markers.marker_ids.append(marker.aruco_marker.marker_ids[-1])
                detected_markers.poses.append(marker.aruco_marker.poses[-1])
                    
            else:
                if len(detected_markers.marker_ids) >= min_marker:
                    if detected_markers.marker_ids[0] == marker.aruco_marker.marker_ids[-1]:
                        cmd_pub.get_logger().info("Returned to the starting marker. Stopping.")
                        angular = 0.0
                        cmd_pub.send_cmd(linear, angular)
                        return
        
        cmd_pub.send_cmd(linear, angular)


def main():
    rclpy.init()

    cmd_pub = CmdPublisher()
    robot_control(cmd_pub)

    rclpy.spin(cmd_pub)

    cmd_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



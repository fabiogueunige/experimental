import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
from robot_urdf.cmd_pub import CmdPublisher
import numpy as np
from geometry_msgs.msg import PoseArray, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers


def main():
    rclpy.init('robot_control')

    vel_pub = CmdPublisher()
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 1.0
    while rclpy.ok():
        
        if  marker.end_recognition:
            angular = 0.0
            vel_pub.send_cmd(linear, angular)
            break

        
        vel_pub.send_cmd(linear, angular)
        rclpy.spin_once(marker)
        rclpy.spin_once(vel_pub)

    
    marker.destroy_node()
    vel_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



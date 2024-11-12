"""
"""
import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
import numpy as np
from geometry_msgs.msg import PoseArray, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers

cmd_pub = None

class cmd_publisher(Node):
    def __init__(self):
        super().__init__('robot_rotation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

def robot_control():
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 1.0

    while(1):
        if marker.aruco_marker is not None:
            print(marker.aruco_marker)
            if marker.aruco_marker.marker_ids [0] == marker.aruco_marker.marker_ids [-1]:
                # if len(marker.aruco_marker) > 1:
                if marker.aruco_marker.poses[0] == marker.aruco_marker.poses[-1]:
                    marker.aruco_marker.marker_ids.pop()
                    marker.aruco_marker.poses.pop()
                    break
        
        cmd_pub.send_cmd(linear, angular)


def main():
    rclpy.init()

    cmd_pub = cmd_publisher()
    robot_control()

    rclpy.spin(cmd_pub)

    cmd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



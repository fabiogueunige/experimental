"""
"""
import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
import numpy as np
from geometry_msgs.msg import PoseArray, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('robot_rotation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

"""
def robot_control(cmd_pub):
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 1.0

    detected_markers = {}
    start_marker_id = None
    start_marker_pose = None

    while rclpy.ok():
        rclpy.spin_once(marker, timeout_sec=0.1)
        if marker.aruco_marker is not None:
            for i, marker_id in enumerate(marker.aruco_marker.marker_ids):
                pose = marker.aruco_marker.poses[i]
                if marker_id not in detected_markers:
                    detected_markers[marker_id] = pose
                    if start_marker_id is None:
                        start_marker_id = marker_id
                        start_marker_pose = pose
                elif marker_id == start_marker_id and pose == start_marker_pose:
                    print("Returned to the starting marker. Stopping.")
                    cmd_pub.send_cmd(0.0, 0.0)
                    return

        cmd_pub.send_cmd(linear, angular)
"""

def robot_control(cmd_pub):
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 1.0

    while rclpy.ok():
        if marker.aruco_marker is not None:
            print(marker.aruco_marker)
            if marker.aruco_marker.marker_ids [0] == marker.aruco_marker.marker_ids [-1]:
                if len(marker.aruco_marker) > 1:
                    if marker.aruco_marker.poses[0] == marker.aruco_marker.poses[-1]:
                        marker.aruco_marker.marker_ids.pop()
                        marker.aruco_marker.poses.pop()
                        break
        
        cmd_pub.send_cmd(linear, angular)


def main():
    rclpy.init()

    cmd_pub = CmdPublisher()
    robot_control(cmd_pub)

    rclpy.spin(cmd_pub)

    CmdPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



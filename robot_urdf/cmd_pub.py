import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
import numpy as np
from geometry_msgs.msg import PoseArray, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = CmdPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
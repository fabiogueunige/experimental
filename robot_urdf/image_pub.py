import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
from robot_urdf.cmd_pub import CmdPublisher
from geometry_msgs.msg import Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/img', 10)

    def publish_image(self, img):
        msg = Image()
        msg = img
        self.publisher_.publish(msg)


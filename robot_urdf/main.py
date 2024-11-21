import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
from robot_urdf.cmd_pub import CmdPublisher
from robot_urdf.image_pub import ImagePublisher
from geometry_msgs.msg import Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def main():
    rclpy.init()
    vel_pub = CmdPublisher()
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 0.5
    bridge = CvBridge()
    img_pub = ImagePublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(marker)
            if not marker.marker_id:
                marker.get_logger().info('No marker detected yet')
            
            if marker.end_recognition:
                angular = 0.0
                vel_pub.send_cmd(linear, angular)
                break


            vel_pub.send_cmd(linear, angular)

        marker.reorder()
        for mk in marker.detected_markers:
            #if isinstance(mk['image'], Image):
            #        marker.get_logger().info('Image received')
                
            try: 
                cv_image = bridge.imgmsg_to_cv2(mk['image'], desired_encoding='mono8')
            except CvBridgeError as e:
                print(e) 

            # cv2.circle(cv_image, (mk['corners'][0][0], mk['corners'][0][1]), 5, (0, 0, 255), -1)

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)

            try:
                img_pub.publish_image(bridge.cv2_to_imgmsg(cv_image, encoding='mono8'))

            except CvBridgeError as e:
                print(e) 
                

    except KeyboardInterrupt:
        print("Shutting down robot_control node.")
    finally:
        marker.destroy_node()
        vel_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
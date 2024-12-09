import rclpy
from robot_urdf.marker_sub import MarkerClass_Subscriber
from robot_urdf.cmd_pub import CmdPublisher
from robot_urdf.image_pub import ImagePublisher
import cv2
from cv_bridge import CvBridge, CvBridgeError

def main():
    rclpy.init()
    vel_pub = CmdPublisher()
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 0.5 #0.5
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
            try: 
                cv_image = bridge.imgmsg_to_cv2(mk['image'], desired_encoding='mono8')
            except CvBridgeError as e:
                print(e) 

            center_x = int(mk['centers'].position.x)
            center_y = int(mk['centers'].position.y)
            radius = int(mk['centers'].position.z)

            cv2.circle(cv_image, (center_x, center_y), radius, (0, 255, 0), 2)
            
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(0)

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
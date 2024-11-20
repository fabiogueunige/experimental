import rclpy
from rclpy.node import Node
from robot_urdf.marker_sub import MarkerClass_Subscriber
from robot_urdf.cmd_pub import CmdPublisher
from geometry_msgs.msg import Twist
from ros2_aruco_interfaces.msg import ArucoMarkers

def main():
    rclpy.init()

    vel_pub = CmdPublisher()
    marker = MarkerClass_Subscriber()
    linear = 0.0
    angular = 0.5

    try:
        while rclpy.ok():
            rclpy.spin_once(marker)
            if not marker.marker_id:
                marker.get_logger().info('No marker detected yet')
            else:
                marker.get_logger().info(f'Detected marker ID: {marker.marker_id}')

            if marker.end_recognition:
                angular = 0.0
                vel_pub.send_cmd(linear, angular)
                break


            vel_pub.send_cmd(linear, angular)

        marker.reorder()
        marker.get_logger().info(f' {marker.detected_markers.marker_ids}')

    except KeyboardInterrupt:
        print("Shutting down robot_control node.")
    finally:
        marker.destroy_node()
        vel_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
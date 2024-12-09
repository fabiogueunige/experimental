import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CmdJointPublisher(Node):
    def __init__(self):
        super().__init__('vel_joint_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray,'arm_joint_01', 1)

    def send_cmd(self, linear, angular):
        # to change the type of message to Float64MultiArray
        msg = Float64MultiArray()
        msg.data = [linear, angular]
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = CmdJointPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
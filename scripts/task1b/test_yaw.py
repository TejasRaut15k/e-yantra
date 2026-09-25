import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import time
import math

def get_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.pub = self.create_publisher(Float64MultiArray, '/delta_joint_cmds', 10)
        self.sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.cb, 10)
        self.yaw = None
        self.started = False

    def cb(self, msg):
        self.yaw = get_yaw(msg.pose.orientation)
        if not self.started:
            self.started = True
            self.get_logger().info(f"Initial yaw: {self.yaw:.4f}")
            # Send joint command: pan=+0.1, wrist3=-0.1
            cmd = Float64MultiArray()
            # pan, lift, elbow, w1, w2, w3
            cmd.data = [0.1, 0.0, 0.0, 0.0, 0.0, -0.1]
            self.pub.publish(cmd)
            self.get_logger().info("Sent cmd: pan=+0.1, w3=-0.1")
        else:
            self.get_logger().info(f"Current yaw: {self.yaw:.4f}")

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

import math
import rclpy
import numpy as np
from matplotlib import pyplot as plt
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class PSM(Node):

    def __init__(self):
        super().__init__('psm_grasp')

        self.measured_cp = None
        self.measured_jaw = None

        # Subscribers
        self.subscription_cp = self.create_subscription(
            PoseStamped,
            '/PSM1/measured_cp',
            self.cb_meassured_cp,
            10)
        self.subscription_jaw = self.create_subscription(
            JointState,
            '/PSM1/jaw/measured_js',
            self.cb_meassured_jaw,
            10)

        # Publishers
        self.pub_cp = self.create_publisher(
            PoseStamped,
            '/PSM1/servo_cp',
            10)
        self.pub_jaw = self.create_publisher(
            JointState,
            '/PSM1/jaw/servo_jp',
            10)


    # Callback for pose
    def cb_meassured_cp(self, msg):
        self.measured_cp = msg
        #print(self.measured_cp)

    # Callback for pose
    def cb_meassured_jaw(self, msg):
        self.measured_jaw = msg
        #print(self.measured_jaw)

    def move_tcp_to(self, target, v, dt):
        # Wait for position to be received
        loop_rate = self.create_rate(100, self.get_clock()) # Hz
        while self.measured_cp is None and rclpy.ok():
            self.get_logger().info('Waiting for pose...')
            rclpy.spin_once(self)

        pose_msg = self.measured_cp
        pose_msg.pose.position.x = target[0]
        pose_msg.pose.position.y = target[1]
        pose_msg.pose.position.z = target[2]

        self.pub_cp.publish(pose_msg)
        print("Motion command sent.")





def main(args=None):
    rclpy.init(args=args)
    psm = PSM()
    psm.move_tcp_to([0.0, 0.05, -0.12], 0.01, 0.01)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

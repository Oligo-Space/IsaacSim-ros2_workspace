import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import numpy as np

class AngleIntercepter(Node):
    '''
    This node intercepts the joint trajectory messages from the reaction wheels
    to wrap them between -2pi and 2pi.
    '''
    def __init__(self):
        super().__init__('angle_intercepter')
        self.angle_publishers = {}
        for i in ['rw1', 'rw2', 'rw3', 'rw4']:
            self.create_subscription(JointTrajectory, f"{i}_controller/joint_trajectory_goobert", self.joint_trajectory_callback, 10)
            self.angle_publishers[i] = self.create_publisher(JointTrajectory, f"{i}_controller/joint_trajectory", 10)

    def joint_trajectory_callback(self, msg):
        wheel = msg.joint_names[0].split('_')[0]
        p = msg.points[0].positions[0]
        # self.get_logger().info(f"Wheel: {wheel}, Position before: {p}")
        
        two_pi = 2 * np.pi
        while p < -two_pi:
            p += two_pi
        while p > two_pi:
            p -= two_pi        

        # self.get_logger().info(f"Wheel: {wheel}, Position after: {p}")
        msg.points[0].positions[0] = p
        self.angle_publishers[wheel].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AngleIntercepter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
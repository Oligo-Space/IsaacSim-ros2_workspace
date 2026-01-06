import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

class TestControl(Node):
    '''
    Publishes torque commands to test that control is accurate
    '''
    def __init__(self):
        super().__init__("test_control")

        self.torque_publisher = self.create_publisher(
            Float32MultiArray,
            "/torque_input",
            10
        )
        self.start = False
        self.create_subscription(Bool,"/start_pub",self.toggle_test,10)
        self.create_subscription(Float32MultiArray,"/torque_input",self.update_torque,10)

        hz = 20

        self.create_timer(1/hz, self.publish_torque)
        self.torque = [10.0, 10.0, 10.0]
        
    def update_torque(self, msg):
        self.torque = msg.data

    def toggle_test(self, msg):
        self.start = msg.data

    def publish_torque(self):
        if not self.start:
            return
        msg = Float32MultiArray()
        msg.data = self.torque
        self.torque_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestControl()
    rclpy.spin(node)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from sympy.logic.boolalg import false
from torch._numpy import False_
class PubAngularAcc(Node):
    def __init__(self):
        super().__init__("pub_angular_acc")

        self.publisher = self.create_publisher(Float32MultiArray, "/rw_speed", 10)
        self.create_subscription(Bool, "/start", self.start_, 10)

        self.start = False
        
        self.create_timer(0.1, self.publish_angular_acc)
        self.last_speed = np.array([0.0,0.0,0.0,0.0])


    def start_(self, msg):
        self.start = msg.data

    def publish_angular_acc(self):
        if self.start:
            self.last_speed -= 100.0

            msg = Float32MultiArray()
            msg.data = list( self.last_speed )
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PubAngularAcc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
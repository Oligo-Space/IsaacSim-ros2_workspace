import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

import numpy as np

class TorqueFromRW(Node):
    def __init__(self):
        super().__init__("torque_from_rw")

        self.torque_publisher = self.create_publisher(Float32MultiArray, '/torque_input', 10)

        self.create_subscription(Float32MultiArray, '/rw_speed', self.update_torque, 10)
        self.create_subscription(Bool, '/arm_initialized', self.initialized, 10)
        self.initialized = True
        self.last_speed = None
        self.last_t = None
        #using basic inertia of a solid cylinder about vertical axis
        #from cubespace datasheet for CW1200
        self.rw_intertia = 1/2 * (490/1000) * (36.5/1000)**2

        self.rw_axis = np.array([
            [0, 0, -1],
            [1, 0, 0],
            [-1, 0, 0],
            [0,1,0]
        ])

    def initialized(self, msg):
        self.initialized = msg.data

    def update_torque(self, msg):
        speeds = np.array(msg.data)
        t = self.get_clock().now().nanoseconds/1e9
        if self.last_t and self.initialized:
            dt = t - self.last_t
            alpha = (speeds - self.last_speed)/dt # Calculate angular acceleration

            torque = np.zeros( (len(speeds),3) ) # Initialize torque vector to (num_wheels, 3) for 3 axis torque
            for r in range(len(speeds)):
                #calculate the direction of torque for each wheel
                torque[r,:] = self.rw_axis[r,:]*alpha[r]*self.rw_intertia #apply element-wise, axes are normalized
            torque = np.sum(torque, axis=0) #get resultant torque vector
            msg = Float32MultiArray()
            msg.data = torque.tolist()
            self.torque_publisher.publish(msg)
        self.last_t = t
        self.last_speed = speeds

def main(args=None):
    rclpy.init(args=args)
    node = TorqueFromRW()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
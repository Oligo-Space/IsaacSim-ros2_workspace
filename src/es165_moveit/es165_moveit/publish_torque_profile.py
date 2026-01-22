import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String, Bool, Float32MultiArray

from collections import deque

class PublishTorqueProfile(Node):
    def __init__(self):
        super().__init__('publish_torque_profile')
        # Globals
        self.profile = None
        self.profile_loaded = False
        self.profile_running = False


        self.create_subscription(String, "/profile_path", self.initialize_profile, 10)
        self.create_subscription(Bool, "/start_profile", self.run_profile,10)
        
        self.torque_publisher = self.create_publisher(Float32MultiArray, "/torque_input", 10)
        self.loaded_publisher = self.create_publisher(Bool, "/profile_loaded",10)

        # Publish false on startup
        self.loaded_publisher.publish(Bool(data=False))
    
    def initialize_profile(self, msg):
        self.profile_loaded = False
        self.profile = []
        try:
            with open(msg.data, "r") as f:
                lines = f.readlines()
                for l in lines:
                    split = l.split()
                    self.profile.append(tuple(split))
            self.profile = deque(self.profile)
            self.profile_loaded = True
            self.loaded_publisher.publish(Bool(data=True))
            self.get_logger().info("Profile loaded!")
        except FileNotFoundError:
            self.get_logger().error("File does not exist")
    
    def run_profile(self,msg):
        if msg.data:
            self.profile_running = True
            while len(self.profile) > 0:
                task = self.profile.popleft()
                if task[0] == "T":
                    assert len(task)==4, f"Invalid torque entry. Syntax: T [TX] [TY] [TZ]"
                    msg = Float32MultiArray()
                    msg.data = [float(task[1]), float(task[2]), float(task[3])]
                    self.torque_publisher.publish(msg)
                elif task[0] == "W":
                    assert len(task)==2, "Invalid wait time entry. Syntax: W [Time (s)]"
                    self.get_clock().sleep_for(Duration(seconds=float(task[1])))
                else:
                    self.get_logger().error("Invalid task, check task file")
            self.profile_running = False
            return True





def main(args=None):
    rclpy.init(args=args)
    node = PublishTorqueProfile()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
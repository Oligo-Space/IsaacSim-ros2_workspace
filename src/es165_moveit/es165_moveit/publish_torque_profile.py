import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool, Float32MultiArray


from collections import deque
import numpy as np
import threading
import os

class PublishTorqueProfile(Node):
    def __init__(self):
        super().__init__('publish_torque_profile')
        # Globals
        self.profile = None
        self.profile_loaded = False
        self.profile_running = False

        self.loaded_profiles = {}
        self.stop_profile = False
        self.config_path = os.path.join(get_package_share_directory("es165_moveit"),"config/")


        self.lock = threading.Lock()


        self.create_subscription(String, "/profile_path", self.initialize_profile, 10)
        self.create_subscription(Bool, "/start_profile", self.run_profile,10)
        self.create_subscription(Bool, "/stop_profile", self.stop, 10)
        
        self.torque_publisher = self.create_publisher(Float32MultiArray, "/torque_input", 10)
        self.loaded_publisher = self.create_publisher(Bool, "/profile_loaded",10)

        # Publish false on startup
        self.loaded_publisher.publish(Bool(data=False))
    
    def stop(self,msg):
        self.get_logger().info("Received Stop")
        self.stop_profile = msg.data

    def get_queue(self,profile):
        q = []
        profile = os.path.join(self.config_path,profile)
        self.get_logger().info(f'{profile}')
        with open(profile,"r") as f:
            lines = f.readlines()
            for l in lines:
                split = l.split()
                if split[0]== "L":
                    q.extend(self.get_queue(split[1]))
                else:
                    q.append(split)
        self.get_logger().info(f"profile {profile}, q: {q}")
        return q


    def initialize_profile(self, msg):
        self.profile_loaded = False
        self.profile = []
        try:
            self.profile = deque(self.get_queue(msg.data))
            self.profile_loaded = True
            self.loaded_publisher.publish(Bool(data=True))
            self.get_logger().info("Profile loaded!")
        except FileNotFoundError:
            self.get_logger().error("File does not exist")
    
    def run_profile(self,msg):
        if msg.data:
            self.profile_running = True
            self.stop_profile = False
            check_applied = (False,None)
            while self.profile and len(self.profile) > 0:
                with self.lock:
                    stop = self.stop_profile
                if stop:
                    break
                task = self.profile.popleft()
                self.get_logger().info(f'Task: {task}')

                if check_applied[0]:
                    msg = Float32MultiArray()
                    torques = list(np.array(check_applied[1],dtype=np.float64))
                    if task[0] == "A":
                        msg.data = [torques[0],torques[1],torques[2],float(task[1])]
                        check_applied = (False,None)
                        self.torque_publisher.publish(msg)
                        self.get_clock().sleep_for(Duration(seconds=float(task[1])))
                        continue
                    else:
                        msg.data = torques
                        self.torque_publisher.publish(msg)
                        self.get_clock().sleep_for(Duration(seconds=0.1))

                if task[0] == "T":
                    assert len(task)==4, f"Invalid torque entry. Syntax: T [TX] [TY] [TZ]"
                    check_applied = (True, task[1:])
                    if len(self.profile) == 0:
                        msg = Float32MultiArray()
                        msg.data = list(np.array(task[1:],dtype=np.float64))
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

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
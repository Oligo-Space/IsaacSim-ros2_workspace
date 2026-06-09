import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from rclpy.task import Future
from std_srvs.srv import Trigger
from threading import Lock
import numpy as np
import ast

class RWController(Node):
    def __init__(self):
        super().__init__("rw_control")
        
        self.create_subscription(Float32MultiArray, '/rw_speed', self.update_goal, 10)

        #Publishers

        #Services
        self.start_service = self.create_client(
            Trigger,
            "/rw1_servo_node/start_servo",
        )

        self.stop_service = self.create_client(
            Trigger,
            "/rw1_servo_node/stop_servo",
        )
       # self.ee_inertia = np.eye(3)

        self.twist_publishers = {}
        self.start_services = {}
        self.stop_services = {}
        self.rws = ["rw1", "rw2", "rw3", "rw4"]
        for rw in self.rws:
            self.start_services[rw] = self.create_client(
                Trigger,
                f"/{rw}_servo_node/start_servo",
            )

            self.stop_services[rw] = self.create_client(
                Trigger,
                f"/{rw}_servo_node/stop_servo",
            )
            self.twist_publishers[rw] = self.create_publisher(TwistStamped, f"/{rw}_servo_node/delta_twist_cmds", 10)


        self.is_enabled = {rw: False for rw in self.rws} # Dictionary to track enabled status of each servo 
        self.all_enabled = False
        for rw in self.rws:
            self.start_servo(rw)

        self.create_timer(0.5, self.check_servo_status)

    # def __del__(self):
    #     '''
    #     Called when node is destroyed, stops the servo client
    #     '''
    #     for rw in self.rws:
    #         self.stop_servo(rw)
    

    def check_servo_status(self):
        c = 0
        if not self.all_enabled:
            for rw in self.rws:
                if not self.is_enabled[rw]:
                    self.get_logger().info(f"Servo {rw} is not enabled, starting servo")
                    self.start_servo(rw)
                else:
                    c+=1
            if c == len(self.rws):
                self.all_enabled = True

    def start_servo(self, rw):
        future = self.start_services[rw].call_async(Trigger.Request())
        # Use lambda to create a callable that captures 'rw' and passes 'future' when called
        future.add_done_callback(lambda f: self.start_servo_callback(rw, f))

    def start_servo_callback(self, rw, future):
        try:
            response = future.result()
            self.get_logger().info(f"Servo {rw} started")
            self.is_enabled[rw] = True
        except Exception as e:
            self.get_logger().error(f"Servo start failed: {e}")

    def stop_servo(self, rw):
        future = self.stop_services[rw].call_async(Trigger.Request())
        # Use lambda to create a callable that captures 'rw' and passes 'future' when called
        future.add_done_callback(lambda f: self.stop_servo_callback(rw, f))

    def stop_servo_callback(self, rw, future):
        try:
            response = future.result()
            self.get_logger().info(f"Servo {rw} stopped")
            self.is_enabled[rw] = False
        except Exception as e:
            self.get_logger().error(f"Servo stop failed: {e}")



    def update_goal(self, msg):
        if not self.all_enabled:
            self.get_logger().error("Servo is not enabled, skipping move request")
            return
        
        speeds = np.array(msg.data,dtype=np.float64)*1e5
        for rw in range(len(self.rws)):
            angular_velocity = speeds[rw]
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"
            twist.twist.angular.z = angular_velocity
            self.twist_publishers[self.rws[rw]].publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RWController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

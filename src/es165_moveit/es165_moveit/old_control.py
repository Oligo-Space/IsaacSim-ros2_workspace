import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String, Bool
from geometry_msgs.msg import Vector3, TransformStamped, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from tf2_ros import TFMessage, TransformBroadcaster


import numpy as np
import ast

class ControlRW(Node):
    def __init__(self):
        super().__init__("control_rw")

        # self.create_subscription(TFMessage, "/rw_poses", self.update_poses, 10)
        self.create_subscription(String, "/rw1_pose", self.update_rw1_pose, 10)
        # self.create_subscription(String, "/rw2_orientation", self.update_rw2_pose, 10)
        # self.create_subscription(String, "/rw3_orientation", self.update_rw3_pose, 10)
        # self.create_subscription(String, "/rw4_orientation", self.update_rw4_pose, 10)
        self.create_subscription(Float32MultiArray, "/rw_speed", self.send_speed_command, 10)

        # self.transform_publisher = self.create_publisher(TFMessage, "/rw_transforms", 10)
        # self.orientation_publisher = self.create_publisher(Quaternion, "/rw1_orientation", 10)

        self.start_rw_pub = self.create_publisher(Bool, "/start_rw", 10)


        # self.child_frames = ["rw1","rw2","rw3","rw4"]
        self.child_frames = ["rw1"]
        self.poses = dict.fromkeys(self.child_frames, None)
        self.rw_publishers = {}
        for rw in self.child_frames:
            self.rw_publishers[rw] = self.create_publisher(Vector3, f"/{rw}_update", 10)
        self.transform_broadcaster = TransformBroadcaster(self)
        self.last_t = None
        # self.send_euler([0.0,0.0,0.0],"rw1")
        self.start_rw_pub.publish(Bool(data=True))

    def update_rw1_pose(self, msg):
        self.poses["rw1"] = (ast.literal_eval(msg.data.split("+")[0]), ast.literal_eval(msg.data.split("+")[1]))
    def update_rw2_pose(self, msg):
        self.poses["rw2"] = (ast.literal_eval(msg.data.split("+")[0]), ast.literal_eval(msg.data.split("+")[1]))
    def update_rw3_pose(self, msg):
        self.poses["rw3"] = (ast.literal_eval(msg.data.split("+")[0]), ast.literal_eval(msg.data.split("+")[1]))
    def update_rw4_pose(self, msg):
        self.poses["rw4"] = (ast.literal_eval(msg.data.split("+")[0]), ast.literal_eval(msg.data.split("+")[1]))
    # def update_rw1_pose(self, msg):
    #     self.poses["rw1"] = ast.literal_eval(msg.data) # x y z w
    # def update_rw2_pose(self, msg):
    #     self.poses["rw2"] = ast.literal_eval(msg.data)
    # def update_rw3_pose(self, msg):
    #     self.poses["rw3"] = ast.literal_eval(msg.data)
    # def update_rw4_pose(self, msg):
    #     self.poses["rw4"] = ast.literal_eval(msg.data)

    # def update_poses(self, msg):
    #     for transform in msg.transforms:
    #         self.poses[transform.child_frame_id] = [(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z) ,\
    #              (transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z)]
    
    def send_transform(self, pose, quat, rw):
        msg = TFMessage()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = rw
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        msg.transforms = [t]
        self.transform_broadcaster.sendTransform(t)
        # self.transform_publisher.publish(msg)
    # def send_orient(self, orient, rw):
    #     msg = Quaternion()
    #     msg.x = orient[0]
    #     msg.y = orient[1]
    #     msg.z = orient[2]
    #     msg.w = orient[3]
    #     self.get_logger().info(f"Sending orientation: {msg}")
    #     self.rw_publishers[rw].publish(msg)

    # def send_euler(self, euler, rw):
    #     msg = Vector3()
    #     msg.x = euler[0]
    #     msg.y = euler[1]
    #     msg.z = euler[2]
    #     self.rw_publishers[rw].publish(msg)
    
    def send_speed_command(self,msg):
        '''
        msg is a Float32MultiArray with speed and direction of speed
        + is CW
        - is CCW
        '''
        t = self.get_clock().now().nanoseconds/1e9
        if self.last_t and not any([i is None for i in self.poses.values()]):
            dt = t-self.last_t
            speeds = list(np.array(msg.data,dtype=np.float64))
            for n in range(len(self.child_frames)):
                pose,quat = self.poses[self.child_frames[n]]
                q_mul = quaternion_from_euler(0,0,speeds[n]*dt)
                updated_quat = quaternion_multiply(quat, q_mul)
                # self.send_orient(updated_quat,self.child_frames[n])
                self.send_transform(pose,updated_quat,self.child_frames[n])
                # euler = euler_from_quaternion([updated_quat[1],updated_quat[2],updated_quat[3],updated_quat[0]])
                # self.send_euler(euler,self.child_frames[n])
        self.last_t = t

def main(args=None):
    rclpy.init(args=args)
    node = ControlRW()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
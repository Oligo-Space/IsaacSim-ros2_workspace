import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32


import numpy as np


class VisualizePCD(Node):
    '''
    Visualizes the reachable poses of the robotic arm using RViZ
    '''
    def __init__(self):
        super().__init__("visualize_pcd")
        poses = "/home/oligo/IsaacSim-ros_workspaces/humble_ws/src/es165_moveit/es165_moveit/reachable_poses.txt"
        self.publisher = self.create_publisher(PointCloud, "pcd_points", 10)
        poses_array = np.loadtxt(poses)
        self.pt_map = self.parse_poses(poses_array)
        self.create_timer(1.0, self.visualize_pcd)


    def parse_poses(self, poses):
        pt_map = {}
        for p in poses:
            pt_nm = (p[0], p[1], p[2])
            if pt_nm in pt_map:
                pt_map[pt_nm] += 1
            else:
                pt_map[pt_nm] = 1
        return pt_map


    def visualize_pcd(self):
        max_val = max(self.pt_map.values())

        pts = []
        intensities = []
        for p in self.pt_map:
            point_msg = Point32()
            point_msg.x = p[0]
            point_msg.y = p[1]
            point_msg.z = p[2]
            pts.append(point_msg)
            int_msg = ChannelFloat32()
            int_msg.name = "rgb"
            norm = self.pt_map[p] / max_val
            int_msg.values = [norm,norm,norm]
            intensities.append(int_msg)

        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.points = pts
        msg.channels = intensities
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizePCD()
    rclpy.spin(node)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import struct
import colorsys
import pye57
import pickle
class VisualizePCD(Node):
    '''
    Visualizes the reachable poses of the robotic arm using RViZ
    '''
    def __init__(self):
        super().__init__("visualize_pcd")
        poses = "/home/oligo/IsaacSim-ros_workspaces/humble_ws/src/es165_moveit/es165_moveit/reachable_poses.txt"
        self.publisher = self.create_publisher(PointCloud2, "pcd_points", 10)
        poses_array = np.loadtxt(poses)
        # self.pt_map = self.parse_poses(poses_array)
        self.pt_map = pickle.load(open("/home/oligo/IsaacSim-ros_workspaces/humble_ws/pt_map.pkl", "rb"))
        self.create_timer(1.0, self.visualize_pcd)

    def save_cloud_to_e57(self,msg, shape,filename="output.e57"):
        # 1. Extract points from ROS message
        # We get x, y, z and optional intensity
        gen = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        gen = np.array(
            [
                [p[0], p[1], p[2], p[3]] for p in gen
            ]
        )
        self.get_logger().info(f"Points: {gen}, shape: {np.array(list(gen)).shape}")
    
        data = {
            "cartesianX": gen[:, 0],
            "cartesianY": gen[:, 1],
            "cartesianZ": gen[:, 2],
        }
        
        # Add intensity if available in the message
        if gen.shape[1] > 3:
            data["intensity"] = gen[:, 3]

        # 3. Write using pye57
        e57_writer = pye57.E57(filename, mode='w')
        e57_writer.write_scan_raw(data)
        e57_writer.close()
        
        print(f"Successfully saved {len(gen)} points to {filename}")

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
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.frame_id = "base_link"
        header.stamp = self.get_clock().now().to_msg()
        ps = []
        for pt in self.pt_map:
            norm = self.pt_map[pt] / max_val
            hue = norm
            saturation = 1.0
            brightness = norm ** 0.5
            r,g,b = colorsys.hsv_to_rgb(hue, saturation, brightness)
            rgb = [int(r*255), int(g*255), int(b*255)]
            rgb_packed = self.pack_rgb(rgb)
            ps.append([pt[0], pt[1], pt[2], rgb_packed])
        
        shape = np.array(ps).shape
        pc2_msg = pc2.create_cloud(header, fields,ps)
        self.save_cloud_to_e57(pc2_msg, shape, "output.e57")
        self.publisher.publish(pc2_msg)
    

    def pack_rgb(self, rgb):
        """
        Packs a 3-length RGB array into a single float32 for PointCloud2.
        """
        # Bit-shift to format: 00RRGGBB
        packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2]
        
        # PointCloud2 expects the 4th field as a float, 
        # but we need to treat the bits as a float32 without changing the value.
        s = struct.pack('>I', packed)
        return struct.unpack('>f', s)[0]

def main(args=None):
    rclpy.init(args=args)
    node = VisualizePCD()
    rclpy.spin(node)
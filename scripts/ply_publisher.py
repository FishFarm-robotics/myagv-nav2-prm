#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d

class PLYPublisher(Node):
    def __init__(self):
        super().__init__('ply_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/ply_points', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Load PLY file
        self.pcd = o3d.io.read_point_cloud("/home/fishfarm/ros/myagv_ros2/ply_files/data/merged_clusters.ply")
        self.get_logger().info('PLY file loaded')
        
    def timer_callback(self):
        # Convert Open3D point cloud to numpy array
        points = np.asarray(self.pcd.points)
        
        # Create PointCloud2 message
        header = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        
        # Create point cloud message
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        
        # Publish
        self.publisher.publish(cloud_msg)
        self.get_logger().info('Publishing point cloud')

def main():
    rclpy.init()
    node = PLYPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
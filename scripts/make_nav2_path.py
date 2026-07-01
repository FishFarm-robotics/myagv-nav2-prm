#!/usr/bin/env python3.8
"""
make_nav2_path.py - Build a path using Nav2 on an occupancy-grid map (.pgm + .yaml).

usage:
  ./make_nav2_path.py --map ~/maps/office.yaml --start 0 0 --goal 2 1.5 --out ~/maps/nav2_path.yaml
"""

import cv2, yaml, argparse, math, os, numpy as np
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav_msgs.msg import Path, OccupancyGrid
from nav2_msgs.action import ComputePathToPose as ComputePathToPoseAction
from ament_index_python.packages import get_package_share_directory

class Nav2PathPlanner(Node):
    def __init__(self):
        super().__init__('nav2_path_planner')
        self._action_client = ActionClient(self, ComputePathToPoseAction, 'compute_path_to_pose')
        self.get_logger().info('Waiting for compute_path_to_pose action server...')
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting again...')
        self.get_logger().info('Action server available!')
        
        # Subscribe to map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.map_received = False
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg
        self.map_received = True
        self.get_logger().info('Map received!')
        self.get_logger().info(f'Map origin: x={msg.info.origin.position.x}, y={msg.info.origin.position.y}')
        self.get_logger().info(f'Map resolution: {msg.info.resolution}')
        self.get_logger().info(f'Map size: {msg.info.width}x{msg.info.height}')

    def get_path(self, start, goal):
        # Wait for map
        while not self.map_received:
            self.get_logger().info('Waiting for map...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().info('Creating path request...')
        self.get_logger().info(f'Start pose: x={start.pose.position.x}, y={start.pose.position.y}')
        self.get_logger().info(f'Goal pose: x={goal.pose.position.x}, y={goal.pose.position.y}')
        
        goal_msg = ComputePathToPoseAction.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = 'navfn_planner'
        
        try:
            self.get_logger().info('Sending goal...')
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, self._send_goal_future)
            
            goal_handle = self._send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return None

            self.get_logger().info('Goal accepted, waiting for result...')
            self._get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, self._get_result_future)
            
            result = self._get_result_future.result().result
            if result is not None:
                self.get_logger().info('Path received!')
                self.get_logger().info(f'Path length: {len(result.path.poses)} poses')
                return result.path
            else:
                self.get_logger().error('Failed to get path: No result returned')
                return None
        except Exception as e:
            self.get_logger().error(f'Failed to get path: {str(e)}')
            return None

def load_map(map_file):
    with open(map_file, 'r') as f:
        map_data = yaml.safe_load(f)
    return map_data['resolution'], map_data['origin']

def create_pose(node, x, y, theta=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(theta/2)
    pose.pose.orientation.w = math.cos(theta/2)
    return pose

def save_yaml(points, out_file):
    data = {'poses': points}
    with open(out_file, 'w') as f:
        yaml.dump(data, f)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True, help='map YAML')
    ap.add_argument('--start', nargs=2, type=float, required=True)
    ap.add_argument('--goal', nargs=2, type=float, required=True)
    ap.add_argument('--out', default='nav2_path.yaml')
    args = ap.parse_args()

    try:
        # Load map and get origin
        resolution, origin = load_map(args.map)
        print(f"Map origin: {origin}")
        print(f"Map resolution: {resolution}")

        rclpy.init()
        planner = Nav2PathPlanner()

        # Create start and goal poses, considering map origin
        start_x = args.start[0] + origin[0]
        start_y = args.start[1] + origin[1]
        goal_x = args.goal[0] + origin[0]
        goal_y = args.goal[1] + origin[1]

        print(f"Original start position: ({args.start[0]}, {args.start[1]})")
        print(f"Original goal position: ({args.goal[0]}, {args.goal[1]})")
        print(f"Adjusted start position: ({start_x}, {start_y})")
        print(f"Adjusted goal position: ({goal_x}, {goal_y})")

        start_pose = create_pose(planner, start_x, start_y)
        goal_pose = create_pose(planner, goal_x, goal_y)

        # Get path from Nav2
        path = planner.get_path(start_pose, goal_pose)
        
        if path is not None:
            # Extract points from path
            points = [(pose.pose.position.x, pose.pose.position.y) 
                     for pose in path.poses]
            save_yaml(points, args.out)
            print(f"Path saved to {args.out}")
            print(f"Path contains {len(points)} points")
        else:
            print("Failed to get path from Nav2")

    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'planner' in locals():
            planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
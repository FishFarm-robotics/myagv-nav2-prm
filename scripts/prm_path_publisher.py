#!/usr/bin/env python3
import rclpy, yaml, math, sys
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
class Pub(Node):
    def __init__(s,f):
        super().__init__('prm_path_pub')
        with open(f) as y:s.w=yaml.safe_load(y)['poses']
        s.pub=s.create_publisher(Path,'/prm_path',10)
        s.t=s.create_timer(0.5,s.cb)
    def cb(s):
        p=Path(); p.header.frame_id='map'
        for x,y,ya in s.w:
            ps=PoseStamped(); ps.header=p.header
            ps.pose.position.x,ps.pose.position.y=x,y
            ps.pose.orientation.z=math.sin(ya/2); ps.pose.orientation.w=math.cos(ya/2)
            p.poses.append(ps)
        s.pub.publish(p)
if __name__=='__main__':
    rclpy.init(); rclpy.spin(Pub(sys.argv[1])); rclpy.shutdown() 
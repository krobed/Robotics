#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rrt_planner import RRTPlanner

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(PoseStamped, '/ground_truth_pose', self.odom_callback, 10)

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.map_data = None
        self.map_info = None
        self.map_received = False

        self.start_position = None 
        self.goal_position = None
        self.path_plotted=False
        self.timer = self.create_timer(1.0, self.timer_callback)

    def map_callback(self, msg):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data).reshape((h, w)) 
        binary_map = np.where(data > 50, 1, 0)
        self.map_data = binary_map
        
        self.map_info = msg.info
        self.map_received = True

    def odom_callback(self, msg):
        pos = msg.pose.position
        self.start_position = (pos.x, pos.y)

    def goal_callback(self, msg):
        
        pos = msg.pose.position
        self.goal_position = (pos.x, pos.y)
        self.get_logger().info(f'Nueva meta recibida: {self.goal_position}')

    def timer_callback(self):

        if not self.map_received:
            self.get_logger().info('Esperando al mapa...')
            return
        if self.start_position is None:
            self.get_logger().info('Esperando odometría para obtener posición inicial...')
            return
        if self.goal_position is None:
            self.get_logger().info('Esperando posicion objetivo...')
            return
        plan = self.run_rrt_star(self.start_position, self.goal_position)
        
        if plan is not None:
            self.publish_path(plan)


    
    def run_rrt_star(self, start, goal):
        rrt = RRTPlanner(self.map_data, self.map_info,start,goal)
        plan = rrt.generate_rrt()
        if not self.path_plotted:
            rrt.plot_rrt()
            self.path_plotted =True
        return list(plan)
    
    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        # self.get_logger().info(f'Plan: {path_points}')
        # self.get_logger().info('Plan publicado en /plan')


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import argparse
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
import sys
import numpy as np

class GoalPublisher(Node):
    def __init__(self, goal_poses):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.gt_sub = self.create_subscription(PoseStamped, '/ground_truth_pose', self.gt_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.goal_sent = False
        self.goal_poses = goal_poses
        self.x, self.y = self.goal_poses.pop(0)
        self.theta = 0.0
        self.state = np.zeros(3)

    def publish_goal(self):
        if self.goal_sent:
            return
        q = quaternion_from_euler(0, 0, self.theta)

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose.position.x = float(self.x)
        goal.pose.position.y = float(self.y)
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.publisher.publish(goal)
        self.get_logger().info(f"Published goal: x={self.x}m, y={self.y}m")
        self.goal_sent = True

    def gt_callback(self, msg: PoseStamped): #Ground truth para la posición
        pos = msg.pose.position
        ori = msg.pose.orientation
        _, _, theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.state[0] = pos.x
        self.state[1] = pos.y
        self.state[2] = theta
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if np.hypot(self.x - self.state[0], self.y - self.state[1]) < 0.2:
            self.goal_sent=False
            self.x,self.y = self.goal_poses.pop(0)

            

def main(args=None):
    goal_poses = [
        # [3,9.9],
        # [4,2],
        # [-3,-8],
        # [1,7],
        # [-3,4],
        [2,-9],
        [-3,7],
        [4,-7],
        [1,4],
        [-3,-8],
        [0,0],
        [-3,7],
        [4,2],
        [4,-7],
        [0,0],
    ]
    rclpy.init(args=args)
    node = GoalPublisher(goal_poses)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import threading
import numpy as np

class PosePlotter(Node):

    def __init__(self):
        super().__init__('pose_plotter')

        # Data storage
        self.gt_x, self.gt_y = [], []

        # Subscribers (adjust topics if needed)
        self.create_subscription(PoseStamped, '/ground_truth_pose', self.gt_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        # Plotting in a separate thread
        threading.Thread(target=self.plot_loop, daemon=True).start()
        self.isPath = None
        self.global_path_x = None
        self.global_path_y = None

    def gt_callback(self, msg: PoseStamped):
        self.gt_x.append(msg.pose.position.x)
        self.gt_y.append(msg.pose.position.y)
        
    def plan_callback(self, msg):
        if self.isPath is None:
            self.global_path_x =[pose.pose.position.x for pose in msg.poses]
            self.global_path_y =[pose.pose.position.y for pose in msg.poses]
            
            self.goal_index = 0
            self.isPath=True

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots()
        while rclpy.ok():
            ax.clear()
            ax.set_title("Robot Pose")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            if self.global_path_x is not None:
                ax.plot(self.global_path_x,self.global_path_y, 'r--', label='RRT* Path')
            if len(self.gt_x)>10:
                min_len = min(len(self.gt_x), len(self.gt_y))
                self.gt_x = self.gt_x[:min_len]
                self.gt_y = self.gt_y[:min_len]
                try:
                    ax.plot(self.gt_x, self.gt_y, 'g--', label='DWA Trajectory')
                except: pass
            ax.grid()
            ax.legend()
            plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PosePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import threading
import numpy as np
from tf_transformations import euler_from_quaternion
import os

class PosePlotter(Node):

    def __init__(self):
        super().__init__('pose_plotter')

        # Data storage
        self.gt_x, self.gt_y = [], []
        self.od_x, self.od_y = [], []
        self.crashes=[]
        self.global_path_x = []
        self.global_path_y = []
        self.local_path_x = []
        self.local_path_y = []
        self.prev_goal = None
        self.goal_position = None
        self.isGNew = True
        self.isLNew = True
        self._map = None
        self.map_info = None
        self.map_received = False
    
        # Subscribers (adjust topics if needed)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(PoseStamped, '/ground_truth_pose', self.gt_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/crash', self.crash_callback, 10)
        self.global_plan_sub = self.create_subscription(Path, '/global_plan', self.global_plan_callback, 10)
        
        # Plotting in a separate thread
        threading.Thread(target=self.plot_loop, daemon=True).start()
        
    def map_callback(self, msg):
        if not self.map_received:
            w = msg.info.width
            h = msg.info.height
            data = np.array(msg.data).reshape((h, w)) 
            self._map = data
            self._map[self._map==100]=1
            self._map[self._map==0]=99
            self._map[self._map==-1]=50
            self.map_info = msg.info
            self._map_height,self._map_width = h,w
            self.map_received = True

    def goal_callback(self, msg):
        pos = msg.pose.position
        self.goal_position = (pos.x, pos.y)
        if self.goal_position != self.prev_goal:
            
            self.get_logger().info(f'Nueva meta recibida: {self.goal_position}')
            self.prev_goal == self.goal_position
            self.isGNew = True

    def gt_callback(self, msg: PoseStamped):
        self.gt_x.append(msg.pose.position.x)
        self.gt_y.append(msg.pose.position.y)
        ori = msg.pose.orientation
        _, _, self.theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.od_x.append(pos.x)
        self.od_y.append(pos.y)

    def crash_callback(self,msg):
         x = msg.pose.position.x
         y = msg.pose.position.y
         if [x,y] not in self.crashes:
            self.crashes.append([x,y])

    def global_plan_callback(self, msg):
        if self.isGNew:
            self.global_path_x = [pose.pose.position.x for pose in msg.poses]
            self.global_path_y = [pose.pose.position.y for pose in msg.poses]
            self.gt_x, self.gt_y = [], []
            self.crashes=[]
            self.isGNew = False             
        

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots(figsize=(8,10))
        while rclpy.ok():
            ax.clear()
            ax.set_title(f"Distance to Objective")
            if len(self.global_path_x)!=0:
                ax.plot(self.global_path_x,self.global_path_y, 'b--', label='RRT* Path')
                ax.scatter(self.global_path_x[1:],self.global_path_y[1:], c='b')
                ax.scatter(self.global_path_x[0],self.global_path_y[0], c='black', label='Init Position')
            if len(self.crashes) != 0:
                ax.scatter(self.crashes[0][0],self.crashes[0][1], c='r', label='Crashes')
                for x,y in self.crashes[1:]:
                    ax.scatter(x,y, c='r')
            if len(self.gt_x)>10:
                min_len = min(len(self.gt_x), len(self.gt_y))
                self.gt_x = self.gt_x[:min_len]
                self.gt_y = self.gt_y[:min_len]
                ax.arrow(self.gt_x[-1], self.gt_y[-1],
                    0.4 * np.cos(self.theta),
                    0.4 * np.sin(self.theta),
                    head_width=0.2, head_length=0.2, fc='r', ec='r')
                try:
                    ax.plot(self.gt_x, self.gt_y, 'g', label='Trajectory')
                except: pass
            if self.goal_position is not None:
                ax.scatter(self.goal_position[0],self.goal_position[1], c='cyan', label='Goal Position')
                
            if self.map_received:
                extent = [
                    self.map_info.origin.position.x,
                    self.map_info.origin.position.x + self._map_width * self.map_info.resolution,
                    self.map_info.origin.position.y,
                    self.map_info.origin.position.y + self._map_height * self.map_info.resolution,
                ]
                ax.imshow(np.flipud(self._map), cmap='gray', extent=extent, origin='upper', vmin=0, vmax=100)
            ax.grid()
            ax.legend(bbox_to_anchor=(1.1,1.15))
            if self.goal_position is not None and len(self.global_path_x)>0 :
                a = len('install/el7009_diff_drive_robot/')
                d = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                plt.savefig(f'{os.path.dirname(d)[:-a]}/el7009_diff_drive_robot/trajectory/({round(self.global_path_x[0])},{round(self.global_path_y[0])}){self.goal_position}.png')
            # plt.pause(0.001)
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

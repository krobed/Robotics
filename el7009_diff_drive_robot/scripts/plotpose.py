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
import pandas as pd

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
        self.init_position=None
        self.distance=None
        self.data =None
        self.csv=None
        self.pos=None
        self.plan_length=None
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
        self.goal_position = [pos.x, pos.y]
        
        if self.goal_position != self.prev_goal:
            self.init_position=[round(self.pos.x,1),round(self.pos.y,1)]
            self.get_logger().info(f'Nueva meta recibida: {self.goal_position}')
            self.prev_goal = self.goal_position
            self.isGNew = True
            self.start=self.current_time
            a = len('install/el7009_diff_drive_robot/lib/')
            self.path = (os.path.dirname(os.path.dirname(os.path.abspath(__file__))))[:-a] + '/el7009_diff_drive_robot'
            try:
                self.csv= dict(pd.read_csv(self.path+'/data.csv'))
            except:
                pass
            if self.data is not None and self.data!={}:
                pd.DataFrame(self.data).to_csv(self.path+'/data.csv', mode='a',index=False, header=not os.path.exists(self.path+'/data.csv'))
                self.data={}
            else:
                self.data={}

    def gt_callback(self, msg: PoseStamped):
        self.pos = msg.pose.position
        self.gt_x.append(self.pos.x)
        self.gt_y.append(self.pos.y)
        ori = msg.pose.orientation
        _, _, self.theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.current_time=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.goal_position is not None:
            self.distance=np.linalg.norm(np.array(self.goal_position) - np.array([self.pos.x,self.pos.y]))

        

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
            self.plan_length= sum(np.linalg.norm([self.global_path_x[i+1]-self.global_path_x[i],self.global_path_y[i+1]-self.global_path_y[i]]) for i in range(len(self.global_path_x)-1))   
        

    def plot_loop(self):
        plt.ion()
        fig, ax = plt.subplots(figsize=(8,10))
        while rclpy.ok():
            ax.clear()
            ax.set_title(f"")
            if len(self.global_path_x)!=0:
                ax.plot(self.global_path_x,self.global_path_y, 'b--', label='RRT* Path')
                ax.scatter(self.global_path_x[1:],self.global_path_y[1:], c='b')
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
            if self.goal_position is not None and self.init_position is not None:
                ax.scatter(self.goal_position[0],self.goal_position[1], c='cyan', label='Goal Position')
                ax.scatter(self.init_position[0],self.init_position[1], c='black', label='Init Position')
                
            if self.map_received:
                extent = [
                    self.map_info.origin.position.x,
                    self.map_info.origin.position.x + self._map_width * self.map_info.resolution,
                    self.map_info.origin.position.y,
                    self.map_info.origin.position.y + self._map_height * self.map_info.resolution,
                ]
                ax.imshow(np.flipud(self._map), cmap='gray', extent=extent, origin='upper', vmin=0, vmax=100)
            ax.grid()
            ax.legend(mode='expand', loc='upper center', ncol=5)
            if self.goal_position is not None and self.goal_position!=self.init_position and self.data is not None:
                plt.savefig(f'{self.path}/trajectory/({self.init_position[1]},{self.init_position[1]})({self.goal_position[0]},{self.goal_position[1]}).png')
                self.data['init'] = [str(self.init_position)]
                self.data['goal'] = [str(self.goal_position)]
                self.data['Distance to Goal'] = [self.distance]
                self.data['Plan Distance'] = [self.plan_length]
                self.data['Time'] = [self.current_time-self.start]
                if self.distance<0.2:
                    self.data['Goal Reached']=[True]
                else:
                    self.data['Goal Reached']=[False]
                # self.get_logger().info(f'{self.data}')
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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class DWANode(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        # Publicador de velocidades
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscripciones
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)

        # Timer para ejecutar el planner a intervalos regulares
        self.timer = self.create_timer(0.2, self.timer_callback)

        # Estado del robot: x, y, theta, v, w
        self.state = np.zeros(5)
        self.scan = None

        # Parámetros del robot (Differential robot)
        self.max_speed = 0.3
        self.max_yawrate = 0.3
        self.max_accel = 1.5
        self.max_dyawrate = 1.2
        self.last_odom_time = None
        self.predict_time = 3.0
        self.dt = 0.2
        # Plan global
        self.global_path = []
        self.goal_index = 0
        self.isPath = False

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist
        _, _, theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_odom_time = current_time
        self.state[0] = pos.x
        self.state[1] = pos.y
        self.state[2] = theta
        self.state[3] = vel.linear.x
        self.state[4] = vel.angular.z
        
    def scan_callback(self, msg):
        self.scan = np.array(msg.ranges)
        # self.get_logger().info(f'Sensors: {self.scan}')
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def plan_callback(self, msg):
        if not self.isPath:
            self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
            self.global_path = self.global_path[1:]
            self.get_logger().info(f'Plan : {self.global_path}')
            self.goal_index = 0
            self.isPath=True
    

    def timer_callback(self):
        if self.scan is None or self.last_odom_time is None:
            self.get_logger().info('Esperando sensores...')
            return
        
        if len(self.global_path)==0:
            self.get_logger().info('Esperando un plan')
            return 
        
        obstacles = self.scan_to_obstacles(self.scan)
        if self.global_path and np.hypot(self.global_path[-1][0] - self.state[0], self.global_path[-1][1] - self.state[1]) > 0.1:
            while (self.goal_index < len(self.global_path) - 1 and
                   np.hypot(self.global_path[self.goal_index][0] - self.state[0],
                            self.global_path[self.goal_index][1] - self.state[1]) < 0.2):
                self.goal_index += 1
            goal = self.global_path[self.goal_index]
            # self.get_logger().info(f'Goal {self.goal_index+1} of {len(self.global_path)}')
            control= self.dwa_control(self.state, goal, obstacles)
        else:
            control = [0.0, 0.0]

        msg = Twist()
        msg.linear.x = control[0]
        msg.angular.z = control[1]
        self.cmd_pub.publish(msg)
        

    def scan_to_obstacles(self, scan):
        obstacles = []
        for i, r in enumerate(scan):
            a = self.angle_min + i*self.angle_increment
            if not np.isfinite(r) or r>5.0:
                continue
            if r < 2.0:
                ox = self.state[0] + r * np.cos(self.state[2] + a)
                oy = self.state[1] + r * np.sin(self.state[2] + a)
                obstacles.append((ox, oy))
        return obstacles

    def dwa_control(self, state, goal, obstacles):
        dw = self.calc_dynamic_window(state)
        best_score = -1
        best_u = [0.0, 0.0]

        for v in np.linspace(dw[0], dw[1], 10):
            for w in np.linspace(dw[2], dw[3], 10):
                traj = self.predict_trajectory(state,goal, v, w)
                score = self.evaluate_trajectory(traj, goal, obstacles)
                if score > best_score:
                    best_score = score
                    best_u = [v, w]
        
        return best_u

    def calc_dynamic_window(self, state):
        
        vs = [state[3] - self.max_accel * self.dt,
              state[3] + self.max_accel * self.dt]
        ws = [state[4] - self.max_dyawrate * self.dt,
              state[4] + self.max_dyawrate * self.dt]

        return [
            max(0, vs[0]), min(self.max_speed, vs[1]),
            max(-self.max_yawrate, ws[0]), min(self.max_yawrate, ws[1])
        ]

    def predict_trajectory(self, state,goal, v, w):
        traj = [np.copy(state)]
        time = 0.0
        while time <= self.predict_time:
            x, y, theta = traj[-1][0], traj[-1][1], traj[-1][2]
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += w * self.dt
            theta = normalize_angle(theta)
            traj.append(np.array([x, y, theta, v, w]))
            time += self.dt
            if np.hypot(x-goal[0],y-goal[0])<0.05: return np.array(traj)
        return np.array(traj)

    def evaluate_trajectory(self, traj, goal, obstacles):
        dx = goal[0] - traj[-1][0]
        dy = goal[1] - traj[-1][1]
        angle_to_goal = np.arctan2(dy, dx)
        heading_error = normalize_angle(traj[-1][2]- angle_to_goal)
        heading_score = 1 - abs(heading_error) / np.pi

        min_dist = 5
        for ox, oy in obstacles:
            d = np.hypot(traj[:, 0] - ox, traj[:, 1] - oy)
            if np.any(d < 0.3):
                return -float('inf')
            min_dist = min(min_dist,np.min(d))

        velocity_score = abs(traj[-1][3])

        return 0.5*( 1.4 * heading_score + 1.0 * velocity_score + 0.1*min_dist)

def main(args=None):
    rclpy.init(args=args)
    node = DWANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
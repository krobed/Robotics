#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class DWANode(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        # Publicador de velocidades
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.crash_pub = self.create_publisher(PoseStamped, '/crash', 10)
        self.path_pub = self.create_publisher(MarkerArray, '/local_plan', 10)
        
        # Subscripciones
        self.gt_sub = self.create_subscription(PoseStamped, '/ground_truth_pose', self.gt_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/global_plan', self.plan_callback, 10)

        # Timer para ejecutar el planner a intervalos regulares
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Estado del robot: x, y, theta, v, w
        self.state = np.zeros(5)
        self.scan = None

        # Parámetros del robot (Differential robot)
        self.max_speed = 0.3
        self.max_yawrate = np.radians(40.0)
        self.max_accel = 1.5
        self.max_dyawrate = np.radians(150.0)
        self.last_odom_time = None
        self.predict_time = 2.0
        self.dt = 0.1
        self.last_state_time = 0
        self.min_goal_distance = 0.5
        # Plan global
        self.global_path = []
        self.goal_index = 0
        self.isPath = False

    def odom_callback(self, msg): #Odom para velocidades
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist
        _, _, theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.state[3] = vel.linear.x
        self.state[4] = vel.angular.z

    def gt_callback(self, msg: PoseStamped): #Ground truth para la posición
        pos = msg.pose.position
        ori = msg.pose.orientation
        _, _, theta = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.state[0] = pos.x
        self.state[1] = pos.y
        self.state[2] = theta
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_odom_time = current_time
       
        if abs(self.state[4]) < 0.005 and self.isPath:
            if abs(sum(self.state[:2] - self.global_path[-1]))>0.1:
                self.get_logger().info(f'Robot has been still for {current_time - self.last_state_time}s')
        else:
            # self.get_logger().info(f'Robot is moving')
            self.last_state_time = current_time
        

    def scan_callback(self, msg):
        self.scan = np.array(msg.ranges)
        # self.get_logger().info(f'Sensors: {self.scan}')
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

    def plan_callback(self, msg):
        
        path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if self.global_path != path:
            self.global_path = path
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
        if self.global_path and np.hypot(self.global_path[-1][0] - self.state[0], self.global_path[-1][1] - self.state[1]) > 0.2:
            while (self.goal_index < len(self.global_path) - 1 and
                   np.hypot(self.global_path[self.goal_index][0] - self.state[0],
                            self.global_path[self.goal_index][1] - self.state[1]) < self.min_goal_distance):
                self.goal_index += 1
            goal = self.global_path[self.goal_index]
            # self.get_logger().info(f'Goal {self.goal_index+1} of {len(self.global_path)}')
            control= self.dwa_control(self.state, goal, obstacles)
        else:
            control = [0.0, 0.0]
            self.get_logger().info(f'Goal Reached: {self.global_path[-1]}')

        msg = Twist()
        msg.linear.x = control[0]
        msg.angular.z = control[1]
        self.cmd_pub.publish(msg)
        

    def scan_to_obstacles(self, scan):
        obstacles = []
        for i, r in enumerate(scan):
            if r <= 4:
                a = self.angle_min + i*self.angle_increment
                ox = self.state[0] + r * np.cos(self.state[2] + a)
                oy = self.state[1] + r * np.sin(self.state[2] + a)
                obstacles.append((round(ox), round(oy)))
                if r - self.range_min <=  0.001:
                    self.get_logger().info(f'Crash at {self.state[:2]}')
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = self.state[0]
                    pose.pose.position.y = self.state[1]
                    self.crash_pub.publish(pose)
        return obstacles

    def publish_path(self, trajectories):
        marker_array = MarkerArray()

        for i, trajectory in enumerate(trajectories):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dwa_predictions"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02  # line width
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

            for pose in trajectory:
                point = Point()
                point.x = pose[0]
                point.y = pose[1]
                point.z = 0.0
                marker.points.append(point)

            marker_array.markers.append(marker)

        self.path_pub.publish(marker_array)

    def dwa_control(self, state, goal, obstacles):
        dw = self.calc_dynamic_window(state)
        best_score = []
        best_cost= -float('inf')
        best_u = [0.0, 0.0]
        best_traj = []
        for v in np.linspace(dw[0], dw[1], 5): # Linear velocity
            for w in np.linspace(dw[2], dw[3], 5): # Angular velocity 
                traj = self.predict_trajectory(state,goal, v, w) # Predictions
                score = self.evaluate_trajectory(traj, goal, obstacles) # Calculate score
                cost = [0.1*score[0],1.2*score[1],0.6*score[2]] # Score Function
                if sum(cost) > best_cost: # Save best configuration
                    best_score = cost
                    best_cost = sum(cost)
                    best_u = [v, w]
                best_traj.append(traj)
        self.publish_path(best_traj)
        # self.get_logger().info(f'{best_score}')
        return best_u

    def calc_dynamic_window(self, state):
        
        vs = [state[3] - self.max_accel * self.dt, # Linear velocity window
              state[3] + self.max_accel * self.dt] 
        ws = [state[4] - self.max_dyawrate * self.dt, # Angular velocity window
              state[4] + self.max_dyawrate * self.dt]

        return [
            max(0, vs[0]), min(self.max_speed, vs[1]),
            max(-self.max_yawrate, ws[0]), min(self.max_yawrate, ws[1]) # Clip velocities
        ]

    def predict_trajectory(self, state,goal, v, w):
        traj = [np.copy(state)]
        time = 0.0
        while time <= self.predict_time: # Predict trajectories with every v and w
            x, y, theta = traj[-1][0], traj[-1][1], traj[-1][2]
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += w * self.dt
            theta = normalize_angle(theta)
            traj.append(np.array([x, y, theta, v, w]))
            time += self.dt
            # if np.hypot(x-goal[0],y-goal[0])<0.05: return np.array(traj)
        return np.array(traj) # Return trajectories

    def evaluate_trajectory(self, traj, goal, obstacles):
        dx = goal[0] - traj[-1][0]
        dy = goal[1] - traj[-1][1]
        angle_to_goal = np.arctan2(dy, dx)
        heading_score = np.pi - abs(normalize_angle(angle_to_goal -  traj[-1][2]))
        min_dist = self.range_max
        for ox, oy in obstacles:
            for pose in traj:
                x = pose[0]
                y = pose[1]
                d = (x - ox)**2 + (y - oy)**2
                # self.get_logger().info(f'{d}')
                if d**0.5 <= 0.3:
                    return [-float('inf')]*3 # If collision, return worst score
                min_dist = min(min_dist,d)

        velocity_score = traj[-1][3] 
        return [heading_score, velocity_score, np.sqrt(min_dist)]

def main(args=None):
    rclpy.init(args=args)
    node = DWANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
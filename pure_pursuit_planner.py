import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation


class DifferentialRobot:

    def __init__(self,
                 local_planner,
                 radius=1.,
                 min_linear_vel =-1.,
                 max_linear_vel = 1.,
                 min_angular_vel=-1,
                 max_angular_vel= 1):

        self._radius = radius
        self._max_linear_vel  = max_linear_vel
        self._min_linear_vel  = min_linear_vel
        self._max_angular_vel = max_angular_vel
        self._min_angular_vel = min_angular_vel

        self._position = None
        self._orientation = None

        self._local_planner = local_planner
        local_planner.set_vel_limits(self._min_linear_vel,
                                     self._max_linear_vel,
                                     self._min_angular_vel,
                                     self._max_angular_vel)


    def set_pose(self, position, orientation=0):

        # TODO: assign the input position and orientation to the
        # corresponding class members (the variables that are defined in the constructor)
        # also set the pose for the local planner, so it knows where the robot is

        # TODO: your code here
        self._position = position
        self._orientation = orientation
        self._local_planner.set_pose(position,orientation)



    def set_map(self, input_map):
        # TODO: assign the input map to the corresponding class member and update
        # the self._map_height, self._map_width class members accordingly
        self._map=input_map
        self._map_height, self._map_width = np.shape(input_map)



    def step(self, action, dt=0.1):

        # TODO: get the current robot position and orientation by parsing "self._position"
        # and "self._orientation"
        # Then create an updated version of the position and orientation of the robot by using the
        # following approximation:
        v_x,v_theta = action
        x,y = self._position
        theta = self._orientation
        x = x + v_x * np.cos(theta) * dt
        y = y + v_x * np.sin(theta) * dt
        theta = theta + v_theta * dt

        # Then, check if this predicted pose is in collision with the environment.
        # If the collision exists, return, otherwise, set the predicted pose as the new robot pose

        """
        # TODO: your code here
        position = [???, ???]
        orientation = ???
        """
        position = [x,y]
        orientation = theta
        if self.check_for_collision(position):
            return
        else:
            self.set_pose(position, orientation)


    def check_for_collision(self, position):

        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if self._map[np.clip(int(position[1]) + j, 0,
                                     self._map_height - 1),
                             np.clip(int(position[0]) + i, 0,
                                     self._map_width - 1)] == 1:

                    return True

        return False


    def update_frame(self, frame):

        if self._local_planner._success:
            self._diff_robot_sim.event_source.stop()

        cmd = self._local_planner.get_ctrl_cmd()

        self.step(cmd)

        self._robot_viz.center = (self._position[0],
                                  self._position[1])

        self._robot_heading_viz.set_data(x=self._position[0],
                                         y=self._position[1],
                                         dx=2.0*np.cos(self._orientation),
                                         dy=2.0*np.sin(self._orientation))

        self._robot_trail.append(self._position.copy())
        self._trail_scatter.set_xdata(np.array(self._robot_trail)[:, 0])
        self._trail_scatter.set_ydata(np.array(self._robot_trail)[:, 1])


    def visualize(self):

        fig, ax = plt.subplots()
        ax.imshow(self._map, cmap='binary')
        ax.set_xlim((0, self._map_width))
        ax.set_ylim((0, self._map_height))

        self._robot_trail = []
        self._robot_trail.append(self._position.copy())

        self._robot_heading_viz = mpatches.FancyArrow(self._position[0],
                                                      self._position[1],
                                                      2.0*np.cos(self._orientation),
                                                      2.0*np.sin(self._orientation),
                                                      width=0.4,
                                                      zorder=1)

        self._robot_viz = mpatches.Circle((self._position[0],
                                           self._position[1]),
                                           self._radius,
                                           edgecolor='black', zorder=2)

        ax.plot(self._local_planner._plan[:, 0],
                self._local_planner._plan[:, 1], color='C1', zorder=0)

        self._trail_scatter = ax.plot(np.array(self._robot_trail)[:, 0],
                                      np.array(self._robot_trail)[:, 1])[0]

        ax.add_artist(self._robot_viz)
        ax.add_artist(self._robot_heading_viz)

        ax.set_aspect('equal')

        self._diff_robot_sim = animation.FuncAnimation(fig=fig,
                                                 func=self.update_frame,
                                                 frames=1000,
                                                 interval=5)
        plt.show()


class PurePursuitPlanner:

    def __init__(self,
                 kx=1.0,
                 look_ahead_dist=5.0):

        self._kx = kx
        self._look_ahead_dist = look_ahead_dist

        self._plan_idx = 0

        self._current_position = None
        self._current_orientation = None

        self._success = False
        self._dist_thresh = 1.0


    def set_plan(self, plan):
        self._plan = plan
        self._plan_idx = 0


    def set_vel_limits(self, min_linear_vel, max_linear_vel,
                             min_angular_vel, max_angular_vel):

        self._max_linear_vel  =  max_linear_vel
        self._min_linear_vel  =  min_linear_vel
        self._max_angular_vel =  max_angular_vel
        self._min_angular_vel =  min_angular_vel


    def set_pose(self, position, orientation=0):
        self._current_position = position
        self._current_orientation = orientation


    def get_local_pose(self, waypoint):

        # TODO: get the way point position [x_pos, y_pos] in local coordinates, with respect to
        # the robots' local frame
        # Note that both the robot's pose and the waypoint position are in global coordinates (map
        # coordinates)
        x,y= self._current_position
        theta = self._current_orientation
        dx = waypoint[0] -x
        dy = waypoint[1] -y
        x_pos = dx * np.cos(-theta) - dy * np.sin(-theta)
        y_pos = dx * np.sin(-theta) + dy * np.cos(-theta)
        return [x_pos, y_pos]


    def get_waypoint(self):

        # TODO: iterate over the plan (self._plan, where each plan element is a waypoint), starting from
        # self._plan_idx and up until the end of the plan itself.
        # For a given waypoint position inside this iteration loop, compute its distance to
        # the current robot position.
        # If the distance to the waypoint is <= the look_ahead_distance, move to the next waypoint by
        # incrementing self._plan_idx in 1 (if the resulting index is within the plan bounds) and break
        # the loop

        # Afterwards, compute the distance between the robot position and the waypoint for the
        # self._plan_idx

        # If the plan idx is such that we are in the final waypoint, and the distance to said waypoint
        # is less that self._dist_thresh, set self._success

        # Finally, return the updated waypoint

        # NOTE: Note that we may not update self._plan_idx always, but we nevertheless always have to return
        # a waypoint, so, we may return the same waypoint several times until the robot get close enough so that
        # we advance to a following waypoint in the plan

        # Hint

        for wp in self._plan[self._plan_idx:]:
            x,y = self._current_position
            dist_to_wp = np.linalg.norm([wp[0]-x,wp[1]-y])
            if dist_to_wp <= self._look_ahead_dist:
                if len(self._plan)-1 > self._plan_idx:
                    self._plan_idx+=1
                break

        plan_position = self._plan[self._plan_idx]
        
        dist = np.linalg.norm([plan_position[0]-x,plan_position[1]-y])
        if self._plan_idx==len(self._plan)-1 and dist<self._dist_thresh:
            self._success = True

        return self._plan[self._plan_idx]


    def get_ctrl_cmd(self):

        # TODO:
        # - get a new waypoint using the get_waypoint method
        # - get the way point in local coordinates w.r.t the robot

        waypoint = self.get_waypoint()
        local_position = self.get_local_pose(waypoint)

        x_position = local_position[0]
        y_position = local_position[1]

        # TODO: compute the linear vel using kx, the local position and the linear vel limits
        linear_vel = self._kx * x_position
        linear_vel = np.clip(linear_vel, self._min_linear_vel, self._max_linear_vel)

        # TODO: compute the angular vel using the look_ahead_distance,
        # the local position and the angular vel limits
        curvature = (2 * y_position) / (self._look_ahead_dist ** 2)
        angular_vel = curvature*linear_vel
        angular_vel = np.clip(angular_vel,self._min_angular_vel,self._max_angular_vel)

        # NOTE: you need the vel limits to clip the result of directly computing the control commands
        # using the pure pursuit equations. Do not use the limits as normalization factors,
        # just clip the commands so they are always within bounds using np.clip.

        return [linear_vel, angular_vel]


import numpy as np

from pure_pursuit_planner import DifferentialRobot
from pure_pursuit_planner import PurePursuitPlanner
from rrt_planner import RRTPlanner

# For RRT and integration tests
def add_map_obstacles_1(input_map):
    # DO NOT MODIFY
    assert input_map.shape == (64, 64)
    input_map[20:22, 10:50] = 1
    input_map[22:42, 48:50] = 1
    input_map[40:42, 10:50] = 1

def add_map_obstacles_2(input_map):
    # DO NOT MODIFY
    assert input_map.shape == (64, 64)
    input_map[10:12, 14:50] = 1
    input_map[20:22, 14:50] = 1
    input_map[40:42, 14:50] = 1
    input_map[50:52, 14:50] = 1
    input_map[5:59 ,  7:9 ] = 1
    input_map[5:59 , 55:57] = 1


def add_map_obstacles_3(input_map):
    # DO NOT MODIFY
    assert input_map.shape == (64, 64)
    input_map[10:12, 0:60] = 1
    input_map[20:22, 5:64] = 1
    input_map[30:32, 0:60] = 1
    input_map[40:42, 5:64] = 1
    input_map[50:52, 0:60] = 1


def rrt_exps(init_pos=None, tar_pos=None, traverse_distance=2, iterations=2000, obstacles=0, width=64, height=64):

    max_attemts = 1

    input_map = np.zeros((width, height))
    if obstacles==1:
        add_map_obstacles_1(input_map)
    elif obstacles==2:
        add_map_obstacles_2(input_map)
    elif obstacles==3:
        add_map_obstacles_3(input_map)

    rrt_planner = RRTPlanner(input_map,traverse_distance=traverse_distance,nb_iterations=iterations)
    rrt_planner.set_init_position(init_pos)
    rrt_planner.set_target_position(tar_pos)

    plan = rrt_planner.generate_rrt()

    if plan is None:
        counter = 1
        while plan is None:
            rrt_planner.set_random_seed(counter)
            plan = rrt_planner.generate_rrt()
            counter+=1
            if counter > max_attemts:
                break

    rrt_planner.plot_rrt()


def pure_pursuit_exps(init_pose=[20,30], kx=1.0,lad=5.0,theta=0,omega_lim=[-1,1]):

    input_map = np.zeros((64, 64))

    # Parametric plan generation (do not modify the plan)
    time = np.linspace(-np.pi, np.pi, 200)

    x_position_plan = np.sin(time)**3
    y_position_plan = (13 * np.cos(time) - (5 * np.cos(2 * time)) - \
                      ( 2 * np.cos(3 * time)) - (np.cos(4 * time))) / 16.

    y_position_plan -= min(y_position_plan) + 1.0

    plan = np.zeros((len(time), 2))
    plan[:, 0] = x_position_plan
    plan[:, 1] = y_position_plan

    # Plan scaling
    plan = plan * 30 + 32

    diff_robot = DifferentialRobot(local_planner=PurePursuitPlanner(kx=kx,look_ahead_dist=lad), min_angular_vel=omega_lim[0],max_angular_vel=omega_lim[1])
    diff_robot.set_map(input_map)

    # TODO: modify the following line to change initial pose
    diff_robot.set_pose(init_pose, orientation=theta)

    diff_robot._local_planner.set_plan(plan)

    diff_robot.visualize()


def integration_exps(lad = 1,obstacles=0):

    input_map = np.zeros((64, 64))
    if obstacles==1:
        add_map_obstacles_1(input_map)
        init_pos = [32, 32]
        target_pos = [55, 32]
        iter = 2000
    elif obstacles==2:
        add_map_obstacles_2(input_map)
        init_pos = [2,60]
        target_pos = [60,12]
        iter = 4000
    elif obstacles==3:
        add_map_obstacles_3(input_map)
        init_pos = [2,60]
        target_pos = [2,2]
        iter = 20000

    diff_robot = DifferentialRobot(local_planner=PurePursuitPlanner(kx=1, look_ahead_dist=lad) )
    rrt_planner = RRTPlanner(input_map, nb_iterations=iter,traverse_distance=2)
    diff_robot.set_map(input_map)

    diff_robot.set_pose(init_pos)
    rrt_planner.set_init_position(init_pos)
    rrt_planner.set_target_position(target_pos)

    plan = rrt_planner.generate_rrt()

    max_attemts = 10
    if plan is None:
        counter = 1
        while plan is None:
            rrt_planner.set_random_seed(counter)
            plan = rrt_planner.generate_rrt()
            counter+=1
            if counter > max_attemts:
                break

    diff_robot._local_planner.set_plan(plan)

    rrt_planner.plot_rrt()
    diff_robot.visualize()


if __name__ == '__main__':

    """ Global Planning """
    # # Exp1
    # iter = [10,100,500]
    # for i in iter:
    #     rrt_exps(iterations=i)
    
    # # Exp2
    # iter=200
    # traverse_distance= [1,2,4]
    # for t in traverse_distance:
    #     rrt_exps(iterations=iter, traverse_distance=t)
    
    # # Exp3
    # width = 128
    # height= 64
    # iter = [10,100,500]
    # for i in iter:
    #     rrt_exps(iterations=i, width= width, height=height)
    
    # # Exp4
    # iter=200
    # traverse_distance= [1,2,4]
    # for t in traverse_distance:
    #     rrt_exps(iterations=iter, traverse_distance=t,width=width,height=height)
    
    # Exp5
    iter = [2000,4000]
    for i in iter:
        rrt_exps(tar_pos= (55,32), iterations=i, obstacles=1)
    
    # # Exp6
    # for i in iter:
    #     rrt_exps(init_pos=(2,60), tar_pos= (60,12), iterations=i, obstacles=2)
    
    # # Exp7
    # iter = [5000,10000,20000]
    # for i in iter:
    #     rrt_exps(init_pos=(2,60), tar_pos= (2,2), iterations=i, obstacles=3)

    """ Local Planning """
    # # Exp1
    # pure_pursuit_exps(kx=1.0,lad=5.0, init_pose=[30,10], theta=0.0)
    # pure_pursuit_exps(kx=1.0,lad=5.0, init_pose=[20,30], theta=0.0)
    # # Exp2
    # pure_pursuit_exps(kx=1.0,lad=1.0, init_pose=[30,10], theta=0.0)
    # pure_pursuit_exps(kx=1.0,lad=1.0, init_pose=[20,30], theta=0.0)
    # pure_pursuit_exps(kx=1.0,lad=10.0, init_pose=[30,10], theta=0.0)
    # pure_pursuit_exps(kx=1.0,lad=10.0, init_pose=[20,30], theta=0.0)
    # # Exp3
    # omega_lim=[-0.1,0.1]
    # lad = [1.0,5.0,10.0]
    # pose = [[20,30],[30,10]]
    # for l in lad:
    #     for p in pose:
    #         pure_pursuit_exps(kx=1.0,lad=l, init_pose=p, theta=0.0, omega_lim=omega_lim)

    """ Integration """
    # # Exp1
    # integration_exps(lad=5, obstacles=1)
    # Exp2
    # integration_exps(lad=5, obstacles=2)
    # integration_exps(lad=5, obstacles=3)
    # Exp3
    # lad = [10]
    # obstacles= [2,3]
    # for l in lad:
    #     for o in obstacles:
    #         integration_exps(lad=l, obstacles=o)




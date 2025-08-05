#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import os

class Nodo:

    def __init__(self, x, y):
        #TODO: Specify position as a tuple (x, y) and parent as an empty list
        # The parent list may be modified when the node is added to a tree,
        # to contain the index of the parent node and the parent node itself
        # For the first node in the tree, the parent list is an empty list
        self.position = (x,y)
        self.parent = []
        self.cost = 0


class RRTPlanner:

    def __init__(self,
                 input_map,
                 map_info=None,
                 init_position=None,
                 target_position=None,
                 nb_iterations=50000,
                 traverse_distance= 1.0,
                 random_seed=0):

        self._map  = input_map
        self._map[self._map==100]=1
        self._map[self._map==0]=99
        self._map[self._map==-1]=50
        
        self.map_info = map_info
        self._map_height,self._map_width = np.shape(input_map)

        self._nb_iterations = nb_iterations
        self._traverse_distance = traverse_distance

        self._init_position = init_position


        if self._init_position is None:
            self._init_position = (self._map_width/2,self._map_height/2)

        self._target_position = target_position

        # Initialize tree
        self._tree = []

        x,y = self._init_position
        self._tree.append(Nodo(x,y))
   
        self._plan = None

        np.random.seed(random_seed)

    def world_to_map(self, x, y):
        mx = int(round((x - self.map_info.origin.position.x) / self.map_info.resolution))
        my = int(round((y - self.map_info.origin.position.y) / self.map_info.resolution))
        return mx, my

    def set_random_seed(self, random_seed):
        np.random.seed(random_seed)
        self.reset_tree()


    def reset_tree(self):
        x,y = self._init_position
        self._tree = [Nodo(x,y)]


    def set_init_position(self, init_position):
        if init_position != None:
            self._init_position = init_position
            self.reset_tree()


    def set_target_position(self, target_position):
        if target_position != None:
            self._target_position = target_position


    def sample_random_position(self):
        height = self.map_info.origin.position.y +  self._map_height*self.map_info.resolution
        width = self.map_info.origin.position.x +  self._map_width*self.map_info.resolution
        sampled_x = np.random.randint(self.map_info.origin.position.x, width)
        sampled_y = np.random.randint(self.map_info.origin.position.y, height)

        return (sampled_x, sampled_y)

    def get_near_nodes(self, new_node, radius):
        near_nodes = []
        for idx, node in enumerate(self._tree):
            dist = np.linalg.norm(np.array(node.position) - np.array(new_node.position))
            if dist <= radius:
                near_nodes.append((idx, node))
        return near_nodes

    def get_nearest_neighbour(self, position):
        min_distance_idx = -1
        min_distance= float('inf')
        x = position[0]
        y = position[1]
        for idx in range(len(self._tree)):
            node = self._tree[idx]
            n_pos = node.position
            distance = np.linalg.norm([n_pos[0]-x , n_pos[1] - y])
            if distance < min_distance:
                min_distance = distance
                min_distance_idx = idx 
            
        return min_distance_idx, self._tree[min_distance_idx]


    def get_new_position(self, random_position, nearest_position):
        x_rand, y_rand = random_position
        x_near, y_near = nearest_position
        alpha = np.arctan2(y_rand-y_near,x_rand-x_near)
        new_x = x_near + self._traverse_distance*np.cos(alpha)
        new_y = y_near + self._traverse_distance*np.sin(alpha)
        x_min = self.map_info.origin.position.x
        x_max = x_min + self._map_width * self.map_info.resolution
        y_min = self.map_info.origin.position.y
        y_max = y_min + self._map_height * self.map_info.resolution

        new_x = np.clip(new_x, x_min, x_max)
        new_y = np.clip(new_y, y_min, y_max)
        return new_x, new_y

    def recover_plan(self):
        plan = []
        plan.append(list(self._target_position))
        parent = False
        if len(self._tree) >0:
            node = self._tree[-1]
            while not parent:
                plan.append(list(node.position))
                if len(node.parent)!=0:
                    node = node.parent[-1]
                else:
                    parent = True
        plan.reverse()
        return np.array(plan)


    def check_for_collision(self, new_position):
        mx, my = self.world_to_map(*new_position)
        if self._map[np.clip(int(my), 0, self._map_height - 1), np.clip(int(mx), 0, self._map_width - 1)] != 99:
            return True
        return False

    def check_path_for_collision(self, start, end):
        num_points = int(np.linalg.norm(np.array(end) - np.array(start)) / 0.05)
        if num_points == 0:
            return self.check_for_collision(end)

        x_vals = np.linspace(start[0], end[0], num=num_points)
        y_vals = np.linspace(start[1], end[1], num=num_points)

        for x, y in zip(x_vals, y_vals):
            if self.check_for_collision((x, y)):
                return True
        return False


    def generate_rrt(self):

        
        for _ in range(self._nb_iterations):
            random_position = self.sample_random_position()
            nearest_idx, nearest_neighbour = self.get_nearest_neighbour(random_position)
            nearest_position = nearest_neighbour.position
            new_position = self.get_new_position(random_position, nearest_position)

            if self.check_path_for_collision(new_position, nearest_position):
                continue
            x,y = new_position
            new_position_node = Nodo(x,y)
            new_position_node.parent = [nearest_idx, nearest_neighbour]
            new_position_node.cost = nearest_neighbour.cost + np.linalg.norm(np.array(new_position_node.position) - np.array(nearest_position))
            self._tree.append(new_position_node)
            radius = self._traverse_distance*4
            near_nodes = self.get_near_nodes(new_position_node, radius)
            for idx, near_node in near_nodes:
                potential_cost = near_node.cost + np.linalg.norm(np.array(new_position_node.position) - np.array(near_node.position))
                if potential_cost < new_position_node.cost and not self.check_path_for_collision(new_position,near_node.position):
                    new_position_node.cost = potential_cost
                    new_position_node.parent = [idx, near_node]
            new_idx = len(self._tree) - 1
            for idx, near_node in near_nodes:
                potential_cost = new_position_node.cost + np.linalg.norm(np.array(new_position_node.position) - np.array(near_node.position))
                if potential_cost < near_node.cost and not self.check_path_for_collision(new_position,near_node.position):
                    self._tree[idx].parent = [new_idx, new_position_node]
                    self._tree[idx].cost = potential_cost

            if self._target_position is not None:
                x_tar, y_tar = self._target_position
                dist = np.linalg.norm([y_tar - y, x_tar -x])
                if dist <= self._traverse_distance and not self.check_path_for_collision(new_position,self._target_position):
                    goal_node = Nodo(*self._target_position)
                    goal_node.parent = [new_idx, new_position_node]
                    goal_node.cost = new_position_node.cost + dist
                    self._tree.append(goal_node)
                    self._plan = self.recover_plan()
                    break
            

        return self._plan


    def plot_rrt(self):

        positions = []
        edges = []
        for idx, node in enumerate(self._tree):
            positions.append(node.position)
            if node.parent != []:
                edges.append([node.parent[0],idx])
        positions = np.array(positions)
        edges = np.array(edges)

        x_pos = positions[:, 0]
        y_pos = positions[:, 1]

        fig, ax = plt.subplots(figsize=(8,8))
        extent = [
            self.map_info.origin.position.x,
            self.map_info.origin.position.x + self._map_width * self.map_info.resolution,
            self.map_info.origin.position.y,
            self.map_info.origin.position.y + self._map_height * self.map_info.resolution,
        ]
        
        ax.imshow(np.flipud(self._map), cmap='gray', extent=extent, origin='upper', vmin=0, vmax=100)
        ax.plot(x_pos[edges.T], y_pos[edges.T], color='C0')
        ax.scatter(positions[:,0], positions[:,1], s=20)

        if self._target_position is not None:
            ax.scatter(self._target_position[0], self._target_position[1], s=50)

        if self._plan is not None:
            ax.scatter(self._plan[:,0], self._plan[:,1], s=20)
            ax.plot(self._plan[:,0], self._plan[:,1], color='red')

        ax.scatter(self._init_position[0], self._init_position[1], s=50)

        ax.set_aspect('equal')
        d = os.path.dirname(os.path.abspath(__file__))
        plt.savefig(f'{os.path.dirname(d)}/rrtree/({round(self._init_position[0])},{round(self._init_position[1])})({self._target_position[0]},{self._target_position[1]}).png')

#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt

from tools import *
from rrt import *
from potential_fields import *


def move_obstacles(obstacles):
    # obstacles[3] += np.array([0.004, 0.005])
    # small cubes movement
    obstacles[-3] += np.array([0.02, 0.0])
    obstacles[-2] += np.array([-0.006, 0.006])
    obstacles[-1] += np.array([0.0, 0.01])
    return obstacles

class Params:
    def __init__(self):
        self.animate = 0 # show RRT construction, set 0 to reduce time of the RRT algorithm
        self.visualize = 1 # show constructed paths at the end of the RRT and path smoothing algorithms
        self.maxiters = 5000 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.minDistGoal = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.extension = 0.4 # [m], extension parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
        self.drone_vel = 2.0 # [m/s]
        self.ViconRate = 100 # [Hz]
        self.max_sp_dist = 1.0 * self.drone_vel # [m], maximum distance between current robot's pose and the sp from global planner
        self.influence_radius = 1.3 # potential fields radius, defining repulsive area size near the obstacle
        self.goal_tolerance = 0.05 # [m], maximum distance threshold to reach the goal
        self.num_robots = 4

# Initialization
params = Params()
xy_start = np.array([1.2, 1.0])
xy_goal =  np.array([1.5, -1.4])
# Obstacles map construction
obstacles = [
              # bugtrap
              np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
              np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
              np.array([[0.5, 1.5], [1.5, 1.5], [1.5, 1.8], [0.5, 1.8]]),
              # angle
              np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
              np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),
              # walls
              np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.49], [-2.5, -2.49]]),
              np.array([[-2.5, 2.49], [2.5, 2.49], [2.5, 2.5], [-2.5, 2.5]]),
              np.array([[-2.5, -2.49], [-2.49, -2.49], [-2.49, 2.49], [-2.5, 2.49]]),
              np.array([[2.49, -2.49], [2.5, -2.49], [2.5, 2.49], [2.49, 2.49]]),

              # moving obstacle
              np.array([[-2.3, 2.0], [-2.2, 2.0], [-2.2, 2.1], [-2.3, 2.1]]),
              np.array([[2.3, -2.3], [2.4, -2.3], [2.4, -2.2], [2.3, -2.2]]),
              np.array([[0.0, -2.3], [0.1, -2.3], [0.1, -2.2], [0.0, -2.2]]),
            ]

# plt.figure(figsize=(12,12))
# plt.grid()
# draw_map(obstacles)
# plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=10, label='start')
# plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=10, label='goal')
# plt.legend()


# Global Planner: RRT path
# plt.figure(figsize=(10,10))
# draw_map(obstacles)
# plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20)
# plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20)

P_long = rrt_path(obstacles, xy_start, xy_goal, params)
P = ShortenPath(P_long, obstacles, smoothiters=30) # P = [[xN, yN], ..., [x1, y1], [x0, y0]]
traj_global = waypts2setpts(P, params)

# plt.plot( P_long[:,0], P_long[:,1], color='blue', linewidth=5, label='RRT path' )
# plt.plot(P[:,0], P[:,1], linewidth=5, color='orange', label='shortened path')
# plt.legend()
# plt.grid()


# Layered Motion Planning: RRT (global) + Potential Field (local)
if __name__ == '__main__':
    plt.figure(figsize=(10,10))
    sp_ind = 0
    route = np.array([traj_global[0,:]])
    sp1 = route[-1,:]
    followers_sp = formation(params.num_robots, leader_des=sp1, v=np.array([0,-0.3]), l=0.3)
    while True: # loop through all the setpoint from global planner trajectory, traj_global
        
        dist_to_goal = norm(sp1-xy_goal)
        if dist_to_goal < params.goal_tolerance: # [m]
            print 'Goal is reached'
            break
        obstacles = move_obstacles(obstacles) # change poses of some obstacles on the map         
        obstacles_grid = grid_map(obstacles)

        # leader's setpoint from global planner
        sp_global = traj_global[sp_ind,:]

        # correct leader's pose with local planner
        f1 = combined_potential(obstacles_grid, sp_global, params.influence_radius)
        sp1 = gradient_planner_next(sp1, f1, params)
        route = np.vstack( [route, sp1] )

        # adding following robots in the swarm:
        followers_sp_global = formation(params.num_robots, leader_des=sp_global, v=normalize(sp_global-sp1), l=0.3) # formation poses from global planner
        for p in range(len(followers_sp)): # formation poses correction with local planner
            # robots repel from each other inside the formation
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [sp1]) if i!=p] # all poses except the robot[p]
            robots_obstacles = poses2polygons( robots_obstacles_sp ) # each drone is defined as a small cube for inter-robots collision avoidance
            obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
            obstacles_grid1 = grid_map(obstacles1) # obstacles grid representation from polygons
            # robot[p] position correction with local planner
            f = combined_potential(obstacles_grid1, followers_sp_global[p], params.influence_radius)
            followers_sp[p] = gradient_planner_next(followers_sp[p], f, params)

        # visualization
        plt.cla()
        draw_map(obstacles)
        draw_gradient(f)
        plt.plot(sp1[0], sp1[1], '^', color='blue', markersize=10, zorder=15) # leader's pose
        for sp in followers_sp: plt.plot(sp[0], sp[1], '^', color='blue', markersize=10, zorder=15) # followers poses
        plt.plot(route[:,0], route[:,1], linewidth=3, color='green', label="Robot's path, corrected with local planner", zorder=10)
        plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
        plt.plot(traj_global[sp_ind,0], traj_global[sp_ind,1], 'ro', color='blue', markersize=5, label='Global planner setpoint')
        plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
        plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')
        plt.legend()
        plt.draw()
        plt.pause(0.01)

        # update loop variable
        if sp_ind < traj_global.shape[0]-1 and norm(sp_global - sp1) < params.max_sp_dist: sp_ind += 1


    # close windows if Enter-button is pressed
    plt.draw()
    plt.pause(0.1)
    raw_input('Hit Enter to close')
    plt.close('all')
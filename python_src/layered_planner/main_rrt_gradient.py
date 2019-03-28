#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
from scipy.ndimage.morphology import distance_transform_edt as bwdist

from numpy.linalg import norm
from math import *
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path
import time


from tools import *
from rrt import *
from potential_fields import *


def layered_planner(P, obstacles_grid):
    """
    Layered Motion Planning:
    inputs: -path from global planner, P
            -obstacles map representation, obstacles_grid
    output: -route, path corrected with potential fields-based
             local planner
    """
    route = np.array([P[-1,:]])
    for i in range(len(P)-1, 0, -1):
        start = route[-1,:]
        goal = P_short[i-1]
        # Combined potential
        f = combined_potential(obstacles_grid, goal)
        # Plan route between 2 consequetive waypoints from P
        V = 0.3 # [m/s]
        freq = 100; dt = 1./freq
        dx = V * dt
        route_via = gradient_planner(f, start, goal, 50)
        # plt.plot(start[0],start[1],'bo',color='red', markersize=10)
        # plt.plot(goal[0], goal[1],'bo',color='green', markersize=10)
        # print norm(start-goal) / dx, len(route_via)
        route = np.vstack([route, route_via])

    return route


class Params:
    def __init__(self):
        self.animate = 0 # show RRT construction, set 0 to reduce time of the RRT algorithm
        self.visualize = 1 # show constructed paths at the end of the RRT and path smoothing algorithms
        self.maxiters = 5000 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.minDistGoal = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.extension = 0.2 # [m], extension parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
        self.drone_vel = 0.3 # [m/s]
        self.ViconRate = 200 # [Hz]

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
            ]
# obstacles = []
obstacles_grid = grid_map(obstacles)

# plt.figure(figsize=(12,12))
# plt.grid()
# draw_map(obstacles)
# plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=10, label='start')
# plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=10, label='goal')
# plt.legend()


# Global Planner: RRT path
P = rrt_path(obstacles, xy_start, xy_goal, params)
P_short = ShortenPath(P, obstacles)


plt.figure(figsize=(10,10))
draw_map(obstacles)
plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20)
plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20)
plt.plot( P[:,0], P[:,1], color='blue', linewidth=5, label='RRT path' )
plt.plot(P_short[:,0], P_short[:,1], linewidth=5, color='orange', label='shortened path')
plt.legend()
plt.grid()


# Potential Fields as a global planner
# Combined potential
f = combined_potential(obstacles_grid, xy_goal)
# Plan route
route_field = gradient_planner(f, xy_start, xy_goal, 700)

plt.figure(figsize=(10,10))
draw_gradient(f)
plt.plot(route_field[:,0], route_field[:,1], linewidth=5)
plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=10, label='start')
plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=10, label='goal')
plt.legend()


# Layered Motion Planning: RRT (global) + Potential Field (local)
route = layered_planner(P_short, obstacles_grid)

plt.figure(figsize=(10,10))
draw_map(obstacles)
plt.plot(P_short[:,0], P_short[:,1], linewidth=5, color='orange', label='global planner path')
plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')
plt.grid()    
# draw_gradient(f)
# plt.plot(route[:,0], route[:,1], linewidth=5, color='green', label='path corrected with local planner')
plt.plot(route[:,0], route[:,1], '.', color='green', label='path corrected with local planner')
plt.legend()


plt.draw()
plt.pause(0.5)
raw_input('Hit Enter to close')
plt.close()
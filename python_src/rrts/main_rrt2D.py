#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
from math import *
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path
import time

from PathSmoothing import SmoothPath
from tools import *
from rrt_path_planner import rrt_path



def draw_map(obstacles, params):
    # Draw obstacles
    fig = plt.figure(figsize=(10, 10))
    plt.grid()
    ax = plt.gca()
    ax.set_xlim(params.world_bounds_x)
    ax.set_ylim(params.world_bounds_y)
    for k in range(len(obstacles)):
        ax.add_patch( Polygon(obstacles[k]) )


class RRT_Params:
    def __init__(self):
        self.animate = 0 # show RRT construction, set 0 to reduce time of the RRT algorithm
        self.visualize = 1 # show constructed paths at the end of the RRT and path smoothing algorithms
        self.maxiters = 5000 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.minDistGoal = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.extension = 0.4 # [m], extension parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction

# Initialization
params = RRT_Params()

# Obstacles. An obstacle is represented as a convex hull of a number of points. 
# First row is x, second is y (position of vertices)
w = 0.2
obstacles = [
              np.array([[0, 0], [1, 0], [1, 0.1], [0, w]]),
              np.array([[0, 0], [w, 0.2], [0.1, 2], [0.0, 2.0]]),
              np.array([[0, 2-w], [1, 2], [1, 2+w], [0, 2+w]]),
              np.array([[1-w, 0], [1+w, 0], [1+w, 1], [1, 1]]),
              np.array([[1-w, 2+w], [1+w, 2+w], [1+w, 1.5], [1, 1.5]]),
              np.array([[0.8, 1], [1+w, 1], [1+w, 1+w], [0.8, 1+w]]),
              np.array([[0.8, 1.5], [1+w, 1.5], [1+w, 1.5+w], [0.8, 1.5+w]]),

              np.array([[-0.5, -0.5], [-1.5, -0.5], [-1-w, -1.5-w], [-0.8, -1.5-w]]),
              
              np.array([[0.5, -1.2], [2.0, -1.2], [1+w, -1.5-w], [0.8, -1.5-w]])
            ]

draw_map(obstacles, params)

# Start and goal positions
xy_start = np.array([0.5, 0.5]);   plt.plot(xy_start[0], xy_start[1],'bo',color='red', markersize=20, label='Start')
xy_goal =  np.array([-1.5, 0.8]);  plt.plot(xy_goal[0],  xy_goal[1], 'bo',color='green',markersize=20, label='Goal')
plt.legend()

P = rrt_path(obstacles, xy_start, xy_goal, params)
plt.plot( P[:,0], P[:,1], color='green', linewidth=5, label='Path from RRT' )
P_smooth = SmoothPath(P, obstacles, smoothiters=100)
plt.plot(P_smooth[:,0], P_smooth[:,1], linewidth=5, color='orange', label='Shortened path')

# TODO: setpoints from via-waypoints
V = 0.3
rate = 10; dt = 1./rate
dx = V * dt

traj = np.array([P_smooth[0]])
for i in range(len(P_smooth)-1):
    A = P_smooth[i]
    B = P_smooth[i+1]
    traj = np.vstack([traj, A])
    
    n = (B-A) / norm(B-A)
    delta = n * dx
    N = int( norm(B-A) / norm(delta) )
    sp = A
    for i in range(N):
        sp += delta
        traj = np.vstack([traj, sp])
    traj = np.vstack([traj, B])


# plt.figure(figsize=(10,10))
# plt.plot(traj[:,0], traj[:,1], '.')


if params.visualize:
  plt.legend()
  plt.show()

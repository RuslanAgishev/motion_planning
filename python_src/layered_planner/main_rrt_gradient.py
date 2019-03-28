#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt

from tools import *
from rrt import *
from potential_fields import *


def move_obstacles(obstacles):
    obstacles[-3] += np.array([0.006, 0.0])
    obstacles[-2] += np.array([-0.003, 0.003])
    obstacles[-1] += np.array([0.0, 0.006])
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

              # moving obstacle
              np.array([[-2.3, 2.0], [-2.2, 2.0], [-2.2, 2.1], [-2.3, 2.1]]),
              np.array([[2.3, -2.3], [2.4, -2.3], [2.4, -2.2], [2.3, -2.2]]),
              np.array([[0.0, -2.3], [0.1, -2.3], [0.1, -2.2], [0.0, -2.2]]),
            ]
# obstacles = []

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
P = ShortenPath(P_long, obstacles, smoothiters=30)

# plt.plot( P_long[:,0], P_long[:,1], color='blue', linewidth=5, label='RRT path' )
# plt.plot(P[:,0], P[:,1], linewidth=5, color='orange', label='shortened path')
# plt.legend()
# plt.grid()


# Layered Motion Planning: RRT (global) + Potential Field (local)
plt.figure(figsize=(10,10))

if __name__ == '__main__':
    # P = [[xN, yN], ..., [x1, y1], [x0, y0]]
    route = np.array([P[-1,:]])
    for i in range(len(P)-1, 0, -1): # loop through all the waypoints, nodes of P in inverse direction
        start = route[-1,:]
        waypoint = P[i-1]
        
        # Plan route between 2 consequetive waypoints from P
        V_des = 1.0 # [m/s]
        freq = 100; dt = 1./freq # freq = Vicon rate
        dx = V_des*dt
        maxiters = int( norm(waypoint - start) / dx )

        dist_to_goal_array = []
        for i in range(maxiters):
            current_point = route[-1,:]
            dist_to_goal = norm(current_point-waypoint)
            dist_to_goal_array.append(dist_to_goal)
            if len(dist_to_goal_array)==10 and abs(min(dist_to_goal_array) - max(dist_to_goal_array)) < 0.03:
                print "Robot is stopped, moving to the next waypoint..."
                # print abs(min(dist_to_goal_array) - max(dist_to_goal_array))
                break
            if dist_to_goal < 0.05: # [m]
                print('Waypoint is reached')
                break

            obstacles = move_obstacles(obstacles)            
            obstacles_grid = grid_map(obstacles)
            f = combined_potential(obstacles_grid, waypoint, influence_radius=1.5) # Artificial Potential Field, surface function
            next_point = gradient_planner_next(current_point, f)
            route = np.vstack( [route, next_point] )

            plt.cla()
            draw_map(obstacles)
            draw_gradient(f)
            plt.plot(current_point[0], current_point[1], '^', color='blue', markersize=15, zorder=15)
            plt.plot(waypoint[0], waypoint[1],'bo',color='blue', markersize=10, label='Next waypoint', zorder=5)
            plt.plot(route[:,0], route[:,1], linewidth=5, color='green', label="Robot's path, corrected with local planner", zorder=10)
            plt.plot(P[:,0], P[:,1], linewidth=5, color='orange', label='Global planner path')
            plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
            plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')
            plt.legend()
            plt.draw()
            plt.pause(0.01)

    # close windows if Enter-button is pressed
    plt.draw()
    plt.pause(0.1)
    raw_input('Hit Enter to close')
    plt.close('all')
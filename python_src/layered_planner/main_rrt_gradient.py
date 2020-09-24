#!/usr/bin/env python


import numpy as np
import matplotlib.pyplot as plt

from tools import *
from rrt import *
from potential_fields import *


def move_obstacles(obstacles, params):
    # obstacles[3] += np.array([0.004, 0.005])
    # small cubes movement
    obstacles[-3] += np.array([0.02, 0.0]) * params.drone_vel
    obstacles[-2] += np.array([-0.005, 0.005]) * params.drone_vel
    obstacles[-1] += np.array([0.0, 0.01]) * params.drone_vel
    return obstacles

class Params:
    def __init__(self):
        self.animate = 1 # show RRT construction, set 0 to reduce time of the RRT algorithm
        self.visualize = 1 # show constructed paths at the end of the RRT and path smoothing algorithms
        self.maxiters = 5000 # max number of samples to build the RRT
        self.goal_prob = 0.05 # with probability goal_prob, sample the goal
        self.minDistGoal = 0.25 # [m], min distance os samples from goal to add goal node to the RRT
        self.extension = 0.4 # [m], extension parameter: this controls how far the RRT extends in each step.
        self.world_bounds_x = [-2.5, 2.5] # [m], map size in X-direction
        self.world_bounds_y = [-2.5, 2.5] # [m], map size in Y-direction
        self.drone_vel = 4.0 # [m/s]
        self.ViconRate = 100 # [Hz]
        self.max_sp_dist = 0.3 * self.drone_vel # [m], maximum distance between current robot's pose and the sp from global planner
        self.influence_radius = 1.22 # potential fields radius, defining repulsive area size near the obstacle
        self.goal_tolerance = 0.05 # [m], maximum distance threshold to reach the goal
        self.num_robots = 3
        self.moving_obstacles = 0 # move small cubic obstacles or not


class Robot:
    def __init__(self):
        self.sp = [0, 0]
        self.sp_global = [0,0]
        self.route = np.array([self.sp])
        self.f = 0
        self.leader = False
        self.vel_array = []

    def local_planner(self, obstacles, params):
        obstacles_grid = grid_map(obstacles)
        self.f = combined_potential(obstacles_grid, self.sp_global, params.influence_radius)
        self.sp, self.vel = gradient_planner_next(self.sp, self.f, params)
        self.vel_array.append(norm(self.vel))
        self.route = np.vstack( [self.route, self.sp] )


# Initialization
params = Params()
xy_start = np.array([1.4, 0.9])
xy_goal =  np.array([1.5, -1.4])
# xy_goal = np.array([1.4, 1.2])


""" Obstacles map construction """
# obstacles = [
#               # bugtrap
#               np.array([[0.5, 0], [2.5, 0.], [2.5, 0.3], [0.5, 0.3]]),
#               np.array([[0.5, 0.3], [0.8, 0.3], [0.8, 1.5], [0.5, 1.5]]),
#               # np.array([[0.5, 1.5], [1.5, 1.5], [1.5, 1.8], [0.5, 1.8]]),
#               # angle
#               np.array([[-2, -2], [-0.5, -2], [-0.5, -1.8], [-2, -1.8]]),
#               np.array([[-0.7, -1.8], [-0.5, -1.8], [-0.5, -0.8], [-0.7, -0.8]]),
#               # walls
#               np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.49], [-2.5, -2.49]]),
#               np.array([[-2.5, 2.49], [2.5, 2.49], [2.5, 2.5], [-2.5, 2.5]]),
#               np.array([[-2.5, -2.49], [-2.49, -2.49], [-2.49, 2.49], [-2.5, 2.49]]),
#               np.array([[2.49, -2.49], [2.5, -2.49], [2.5, 2.49], [2.49, 2.49]]),

#               np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]), # my table
#               np.array([[-1.0, 2.0], [0.5, 2.0], [0.5, 2.5], [-1.0, 2.5]]) + np.array([2.0, 0]), # Evgeny's table
#               np.array([[-2.0, -0.5], [-2.0, 1.0], [-2.5, 1.0], [-2.5, -0.5]]), # Roman's table
#               np.array([[-1.2, -1.2], [-1.2, -2.5], [-2.5, -2.5], [-2.5, -1.2]]), # mats
#               np.array([[2.0, 0.8], [2.0, -0.8], [2.5, -0.8], [2.5, 0.8]]), # Mocap table


#               # moving obstacle
#               np.array([[-2.3, 2.0], [-2.2, 2.0], [-2.2, 2.1], [-2.3, 2.1]]),
#               np.array([[2.3, -2.3], [2.4, -2.3], [2.4, -2.2], [2.3, -2.2]]),
#               np.array([[0.0, -2.3], [0.1, -2.3], [0.1, -2.2], [0.0, -2.2]]),
#             ]

passage_width = 0.25
passage_location = 0.0
obstacles = [
            # narrow passage
              np.array([[-2.5, -0.5], [-passage_location-passage_width/2., -0.5], [-passage_location-passage_width/2., 0.5], [-2.5, 0.5]]),
              np.array([[-passage_location+passage_width/2., -0.5], [2.5, -0.5], [2.5, 0.5], [-passage_location+passage_width/2., 0.5]]),
            ]

robots = []
for i in range(params.num_robots):
    robots.append(Robot())
robot1 = robots[0]; robot1.leader=True


# postprocessing variables:
mean_dists_array = []
max_dists_array = []

# Layered Motion Planning: RRT (global) + Potential Field (local)
if __name__ == '__main__':
    plt.figure(figsize=(10,10))
    draw_map(obstacles)
    plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
    plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')

    P_long = rrt_path(obstacles, xy_start, xy_goal, params)
    # plt.plot(P_long[:,0], P_long[:,1], linewidth=3, color='green', label='Global planner path')
    # plt.pause(1.0)
    P = ShortenPath(P_long, obstacles, smoothiters=30) # P = [[xN, yN], ..., [x1, y1], [x0, y0]]

    traj_global = waypts2setpts(P, params); P = np.vstack([P, xy_start])
    plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
    plt.pause(0.1)

    sp_ind = 0
    robot1.route = np.array([traj_global[0,:]])
    robot1.sp = robot1.route[-1,:]

    followers_sp = formation(params.num_robots, leader_des=robot1.sp, v=np.array([0,-1.0]), l=0.3)
    for i in range(len(followers_sp)):
        robots[i+1].sp = followers_sp[i]
        robots[i+1].route = np.array([followers_sp[i]])

    while True: # loop through all the setpoint from global planner trajectory, traj_global
        dist_to_goal = norm(robot1.sp - xy_goal)
        if dist_to_goal < params.goal_tolerance: # [m]
            print('Goal is reached')
            break
        if params.moving_obstacles: obstacles = move_obstacles(obstacles, params) # change poses of some obstacles on the map

        # leader's setpoint from global planner
        robot1.sp_global = traj_global[sp_ind,:]
        # correct leader's pose with local planner
        robot1.local_planner(obstacles, params)

        """ adding following robots in the swarm """
        # formation poses from global planner
        followers_sp_global = formation(params.num_robots, robot1.sp_global, v=normalize(robot1.sp_global-robot1.sp), l=0.3)
        for i in range(len(followers_sp_global)):
            robots[i+1].sp_global = followers_sp_global[i]

        for p in range(len(followers_sp)): # formation poses correction with local planner
            # robots repel from each other inside the formation
            robots_obstacles_sp = [x for i,x in enumerate(followers_sp + [robot1.sp]) if i!=p] # all poses except the robot[p]
            robots_obstacles = poses2polygons( robots_obstacles_sp ) # each drone is defined as a small cube for inter-robots collision avoidance
            obstacles1 = np.array(obstacles + robots_obstacles) # combine exisiting obstacles on the map with other robots[for each i: i!=p] in formation
            # follower robot's position correction with local planner
            robots[p+1].local_planner(obstacles1, params)
            followers_sp[p] = robots[p+1].sp

        # centroid pose:
        centroid = 0
        for robot in robots:
        	centroid += robot.sp / len(robots)
        # dists to robots from the centroid:
        dists = []
        for robot in robots:
        	dists.append( norm(centroid-robot.sp) )
        mean_dists_array.append(np.mean(dists))
        max_dists_array.append(np.max(dists))

        # vizualization
        plt.cla()
        plt.plot(centroid[0], centroid[1], '*', color='blue', markersize=7)
        draw_map(obstacles)
        if params.num_robots == 1:
            draw_gradient(robots[0].f)
        else:
            draw_gradient(robots[1].f)
        for robot in robots[1:]: plt.plot(robot.sp[0], robot.sp[1], '^', color='blue', markersize=10, zorder=15) # robots poses
        plt.plot(robot1.sp[0], robot1.sp[1], '^', color='green', markersize=10, zorder=15) # robots poses
        plt.plot(robot1.route[:,0], robot1.route[:,1], linewidth=2, color='green', label="Robot's path, corrected with local planner", zorder=10)
        # for robot in robots[1:]: plt.plot(robot.route[:,0], robot.route[:,1], '--', linewidth=2, color='green', zorder=10)
        plt.plot(P[:,0], P[:,1], linewidth=3, color='orange', label='Global planner path')
        for robot in robots[:1]: plt.plot(robot.sp_global[0], robot.sp_global[1], 'ro', color='green', markersize=7, label='Global planner setpoint')
        plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20, label='start')
        plt.plot(xy_goal[0], xy_goal[1],'bo',color='green', markersize=20, label='goal')
        plt.legend()
        plt.draw()
        plt.pause(0.01)

        # update loop variable
        if sp_ind < traj_global.shape[0]-1 and norm(robot1.sp_global - robot1.sp) < params.max_sp_dist: sp_ind += 1


print('Postprocessing...')
plt.figure()
plt.title('Robots velocities')
plt.plot(robot1.vel_array, label='leader')
for i in range(1,len(robots)): plt.plot(robots[i].vel_array, '--', label='follower %d' %i)
plt.legend()
plt.grid()


plt.figure()
plt.plot(mean_dists_array, label='mean swarm size')
plt.plot(max_dists_array, label='max swarm size')
plt.legend()
plt.grid()


# close windows if Enter-button is pressed
plt.draw()
plt.pause(0.1)
input('Hit Enter to close')
plt.close('all')
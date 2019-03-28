#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
from scipy.ndimage.morphology import distance_transform_edt as bwdist


# Potential Fields functions

def grid_map(obstacles, nrows=500, ncols=500):
    """ Obstacles dicretized map """
    grid = np.zeros((nrows, ncols));
    # rectangular obstacles
    for obstacle in obstacles:
        x1 = meters2grid(obstacle[0][1]); x2 = meters2grid(obstacle[2][1])
        y1 = meters2grid(obstacle[0][0]); y2 = meters2grid(obstacle[2][0])
        grid[x1:x2, y1:y2] = 1
    return grid

def meters2grid(pose_m, nrows=500, ncols=500):
    # [0, 0](m) -> [250, 250]
    # [1, 0](m) -> [250+100, 250]
    # [0,-1](m) -> [250, 250-100]
    if np.isscalar(pose_m):
        pose_on_grid = int( pose_m*100 + ncols/2 )
    else:
        pose_on_grid = np.array( np.array(pose_m)*100 + np.array([ncols/2, nrows/2]), dtype=int )
    return pose_on_grid
def grid2meters(pose_grid, nrows=500, ncols=500):
    # [250, 250] -> [0, 0](m)
    # [250+100, 250] -> [1, 0](m)
    # [250, 250-100] -> [0,-1](m)
    if np.isscalar(pose_grid):
        pose_meters = (pose_grid - ncols/2) / 100.0
    else:
        pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100.0
    return pose_meters

def combined_potential(obstacles_grid, goal, influence_radius=2, attractive_coef=1./700, repulsive_coef=200, nrows=500, ncols=500):
    """ Repulsive potential """
    goal = meters2grid(goal)
    d = bwdist(obstacles_grid==0)
    d2 = (d/100.) + 1 # Rescale and transform distances
    d0 = influence_radius
    nu = repulsive_coef
    repulsive = nu*((1./d2 - 1./d0)**2)
    repulsive [d2 > d0] = 0
    """ Attractive potential """
    [x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))
    xi = attractive_coef
    attractive = xi * ( (x - goal[0])**2 + (y - goal[1])**2 )
    """ Combine terms """
    f = attractive + repulsive
    return f


def gradient_planner (f, start, goal, max_its):
    # GradientBasedPlanner : This function plans a path through a 2D
    # environment from a start to a destination based on the gradient of the
    # function f which is passed in as a 2D array. The two arguments
    # start_coords and end_coords denote the coordinates of the start and end
    # positions respectively in the array while max_its indicates an upper
    # bound on the number of iterations that the system can use before giving
    # up.
    # The output, route, is an array with 2 columns and n rows where the rows
    # correspond to the coordinates of the robot as it moves along the route.
    # The first column corresponds to the x coordinate and the second to the y coordinate

    [gy, gx] = np.gradient(-f);
    start_coords = meters2grid(start); end_coords = meters2grid(goal)
    route = np.array( [np.array(start_coords)] )
    for i in range(max_its):
        current_point = route[-1,:];
        # print(sum( abs(current_point-end_coords) ))
        if sum( abs(current_point-end_coords) ) < 5.0:
            # print('Reached the goal !');
            break
        ix = int( current_point[1] )
        iy = int( current_point[0] )
        vx = gx[ix, iy]
        vy = gy[ix, iy]
        dt = 1 / np.linalg.norm([vx, vy]);
        next_point = current_point + dt*np.array( [vx, vy] );
        route = np.vstack( [route, next_point] );
    route = grid2meters(route)
    
    return route
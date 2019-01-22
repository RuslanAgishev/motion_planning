# python3 GradientBasedPlanner.py

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.morphology import distance_transform_edt as bwdist
from math import *

from progress.bar import FillingCirclesBar



""" init """
animate = True
max_its = 800
random_map = 0
num_random_obstacles = 6
moving_obstacles = True
progress_bar = FillingCirclesBar('Simulation Progress', max=max_its)
R_obstacles = 0.1 # [m]
R_swarm     = 0.2 # [m]

def draw_map(R_obstacles, nrows=500, ncols=500):
    skip = 10
    [x_m, y_m] = np.meshgrid(np.linspace(-2.5, 2.5, ncols), np.linspace(-2.5, 2.5, nrows))
    Q = plt.quiver(x_m[::skip, ::skip], y_m[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip])
    plt.plot(start[0], start[1], 'ro', markersize=10);
    plt.plot(goal[0], goal[1], 'ro', color='green', markersize=10);
    plt.xlabel('X')
    plt.ylabel('Y')
    ax = plt.gca()
    for pose in obstacles_poses:
        circle = plt.Circle(pose, R_obstacles, color='yellow')
        ax.add_artist(circle)

def meters2grid(pose_m, nrows=500, ncols=500):
    # [0, 0](m) -> [250, 250]
    # [1, 0](m) -> [250+100, 250]
    # [0,-1](m) -> [250, 250-100]
    pose_on_grid = np.array(pose_m)*100 + np.array([ncols/2, nrows/2])
    return np.array( pose_on_grid, dtype=int)

def grid2meters(pose_grid, nrows=500, ncols=500):
    # [250, 250] -> [0, 0](m)
    # [250+100, 250] -> [1, 0](m)
    # [250, 250-100] -> [0,-1](m)
    pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100.0
    return pose_meters

def gradient_planner(f, current_point, end_coords):
    """
    GradientBasedPlanner : This function computes the next_point
    given current location, goal location and potential map, f
	"""
    [gy, gx] = np.gradient(-f);
    # ix = int( current_point[1] ); iy = int( current_point[0] );
    iy, ix = np.array( meters2grid(current_point), dtype=int )
    vx = gx[ix, iy]; vy = gy[ix, iy]
    dt = 0.05 / np.linalg.norm([vx, vy]);
    next_point = current_point + dt*np.array( [vx, vy] );

    return next_point

def combined_potential(obstacles_poses, goal, R_obstacles, nrows=500, ncols=500):
    """ Obstacles map """
    obstacles_map = np.zeros((nrows, ncols));
    [x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))
    for pose in obstacles_poses:
        pose = meters2grid(pose)
        x0 = pose[0]; y0 = pose[1]
        # cylindrical obstacles
        t = ((x - x0)**2 + (y - y0)**2) < (100*R_obstacles)**2
        obstacles_map[t] = True;
    """ Repulsive potential """
    goal = meters2grid(goal)
    d = bwdist(obstacles_map==0);
    d2 = (d/100.) + 1; # Rescale and transform distances
    d0 = 2;
    nu = 200;
    repulsive = nu*((1./d2 - 1/d0)**2);
    repulsive [d2 > d0] = 0;

    """ Attractive potential """
    xi = 1/700.;
    attractive = xi * ( (x - goal[0])**2 + (y - goal[1])**2 );

    """ Combine terms """
    f = attractive + repulsive;
    return f




start = np.array([-1.5, 0.5]); goal = np.array([1.5, -1.0]);

# rectangular obstacles
# obstacle [300:, 100:250] = True;
# obstacle [150:200, 400:500] = True;

if random_map:
    obstacles_poses = np.random.uniform(low=-2.5, high=2.5, size=(num_random_obstacles,2)) # randomly located obstacles 
else:
    obstacles_poses = [[-2, 1], [1.5, 0.5], [0, 0], [-1.8, -1.8]] # 2D - coordinates [m]



""" Plan route: centroid path """
plt.figure(figsize=(10, 10))
route = start
current_point = start
for i in range(max_its):
    if moving_obstacles:
        obstacles_poses[0][0] += 0.03; obstacles_poses[0][1] -= 0.02
        obstacles_poses[1][0] -= 0.03; obstacles_poses[1][1] -= 0.01
        obstacles_poses[2][0] -= 0.01; obstacles_poses[2][1] -= 0.01
        obstacles_poses[3][0] += 0.03; obstacles_poses[3][1] += 0.03
    f = combined_potential(obstacles_poses, goal, R_obstacles)
    [gy, gx] = np.gradient(-f);
    dist_to_goal = np.linalg.norm(current_point - goal)
    if dist_to_goal < 0.1:
        print('\nReached the goal')
        break
    next_point = gradient_planner(f, current_point, goal)
    route = np.vstack([route, next_point])
    current_point = next_point

    progress_bar.next()
    plt.cla()
    draw_map(R_obstacles)
    plt.plot(route[:,0], route[:,1], 'blue', linewidth=2);
    if animate:
        plt.draw()
        plt.pause(0.01)


print('Done')
progress_bar.finish()
plt.show()


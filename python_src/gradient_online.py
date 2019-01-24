# In order to launch execute:
# python3 gradient_online.py

import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import collections
from scipy.ndimage.morphology import distance_transform_edt as bwdist
from math import *
import random

from progress.bar import FillingCirclesBar
from tasks import get_movie_writer
from tasks import get_dummy_context_mgr



def draw_map(obstacles_poses, nrows=500, ncols=500):
    skip = 10
    [x_m, y_m] = np.meshgrid(np.linspace(-2.5, 2.5, ncols), np.linspace(-2.5, 2.5, nrows))
    [gy, gx] = np.gradient(-f);
    Q = plt.quiver(x_m[::skip, ::skip], y_m[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip])
    plt.plot(start[0], start[1], 'ro', color='yellow', markersize=10);
    plt.plot(goal[0], goal[1], 'ro', color='green', markersize=10);
    plt.xlabel('X')
    plt.ylabel('Y')
    ax = plt.gca()
    for pose in obstacles_poses:
        circle = plt.Circle(pose, R_obstacles, color='red')
        ax.add_artist(circle)
    # Create a Rectangle patch
    rect1 = patches.Rectangle((-2.5,-1.15),2.0,0.2,linewidth=1,color='red',fill='True')
    rect2 = patches.Rectangle((-1.2, 1.4), 0.2,1.0,linewidth=1,color='red',fill='True')
    rect3 = patches.Rectangle(( 0.4, 0.8), 2.0,0.5,linewidth=1,color='red',fill='True')
    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(rect3)
    
def draw_robots():
    robots_poses = [next_point1, next_point2, next_point3, next_point4]
    plt.arrow(current_point1[0], current_point1[1], V[0], V[1], width=0.01, head_width=0.05, head_length=0.1, fc='k')
    plt.plot(route1[:,0], route1[:,1], 'green', linewidth=2)
    plt.plot(route2[:,0], route2[:,1], '--', color='blue', linewidth=2)
    plt.plot(route3[:,0], route3[:,1], '--', color='blue', linewidth=2)
    plt.plot(route4[:,0], route4[:,1], '--', color='blue', linewidth=2)
    for pose in robots_poses:
    	plt.plot(pose[0], pose[1], 'ro', color='blue')
    # compute centroid and sort poses by polar angle
    pp = robots_poses
    cent=(sum([p[0] for p in pp])/len(pp),sum([p[1] for p in pp])/len(pp))
    pp.sort(key=lambda p: atan2(p[1]-cent[1],p[0]-cent[0]))
    formation = patches.Polygon(pp, color='blue', fill=False, linewidth=2);
    plt.gca().add_patch(formation)

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

def gradient_planner(f, current_point, ncols=500, nrows=500):
    """
    GradientBasedPlanner : This function computes the next_point
    given current location, goal location and potential map, f.
    It also returns mean velocity, V, of the gradient map in current point.
    """
    [gy, gx] = np.gradient(-f);
    iy, ix = np.array( meters2grid(current_point), dtype=int )
    w = 40 # smoothing window size for gradient-velocity
    vx = np.mean(gx[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
    vy = np.mean(gy[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
    V = np.array([vx, vy])
    dt = 0.05 / norm(V);
    next_point = current_point + dt*np.array( [vx, vy] );

    return next_point, V

def combined_potential(obstacles_poses, goal, nrows=500, ncols=500):
    """ Repulsive potential """
    obstacles_map = map(obstacles_poses)
    goal = meters2grid(goal)
    d = bwdist(obstacles_map==0);
    d2 = (d/100.) + 1; # Rescale and transform distances
    d0 = 2;
    nu = 200;
    repulsive = nu*((1./d2 - 1/d0)**2);
    repulsive [d2 > d0] = 0;
    """ Attractive potential """
    [x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))
    xi = 1/700.;
    attractive = xi * ( (x - goal[0])**2 + (y - goal[1])**2 );
    """ Combine terms """
    f = attractive + repulsive;
    return f

def map(obstacles_poses, nrows=500, ncols=500):
    """ Obstacles map """
    obstacles_map = np.zeros((nrows, ncols));
    [x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))
    for pose in obstacles_poses:
        pose = meters2grid(pose)
        x0 = pose[0]; y0 = pose[1]
        # cylindrical obstacles
        t = ((x - x0)**2 + (y - y0)**2) < (100*R_obstacles)**2
        obstacles_map[t] = 1;
    # rectangular obstacles
    obstacles_map[400:, 130:150] = 1;
    obstacles_map[130:150, :200] = 1;
    obstacles_map[330:380, 300:] = 1;
    return obstacles_map

def move_obstacles(obstacles_poses):
    # dx = 0.01;                   dy = 0.01
    # obstacles_poses[0][0] += dx; obstacles_poses[0][1] -= dy
    # obstacles_poses[1][0] -= dx; obstacles_poses[1][1] -= dy
    # obstacles_poses[2][0] -= dx; obstacles_poses[2][1] -= dy
    # obstacles_poses[3][0] += dx; obstacles_poses[3][1] += dy
    """ obstacles tend to go to the origin, (0,0) - point """
    for pose in obstacles_poses:
    	dx = random.uniform(0, 0.03); dy = random.uniform(0,0.03);
    	pose[0] -= np.sign(pose[0])*dx;      pose[1] -= np.sign(pose[1])*dy;
    return obstacles_poses



""" initialization """
animate = 1
max_its = 200
random_obstacles = 1
num_random_obstacles = 4
moving_obstacles = 1
progress_bar = FillingCirclesBar('Simulation Progress', max=max_its)
R_obstacles = 0.1 # [m]
R_swarm     = 0.3 # [m]
start = np.array([-1.8, 1.8]); goal = np.array([1.7, -1.7])
V0 = (goal - start) / norm(goal-start)   # initial movement direction, |V0| = 1
U0 = np.array([-V0[1], V0[0]]) / norm(V0) # perpendicular to initial movement direction, |U0|=1
should_write_movie = 1; movie_file_name = 'output.avi'
movie_writer = get_movie_writer(should_write_movie, 'Simulation Potential Fields', movie_fps=10., plot_pause_len=0.01)
adaptive_velocity = 1


if random_obstacles:
    obstacles_poses = np.random.uniform(low=-2.5, high=2.5, size=(num_random_obstacles,2)) # randomly located obstacles 
else:
    obstacles_poses = np.array([[-2, 1], [1.5, 0.5], [0, 0], [-1.8, -1.8]]) # 2D - coordinates [m]




""" Plan route: centroid path """

# drones forming equilateral triangle
route1 = start 											# leader
route2 = route1 - V0*(R_swarm*sqrt(3)/2) + U0*R_swarm/2 # follower
route3 = route1 - V0*(R_swarm*sqrt(3)/2) - U0*R_swarm/2 # follower
route4 = route1 - V0 * R_swarm*sqrt(3)                  # follower

current_point1 = start


fig = plt.figure(figsize=(10, 10))
with movie_writer.saving(fig, movie_file_name, max_its) if should_write_movie else get_dummy_context_mgr():
    for i in range(max_its):
        if moving_obstacles: obstacles_poses = move_obstacles(obstacles_poses)


        f = combined_potential(obstacles_poses, goal)
        next_point1, V = gradient_planner(f, current_point1)
        U = np.array([-V[1], V[0]]) # vector, perpendicular to the movement direction

        if adaptive_velocity:
            # scale_min * triangular formation < triangular formation < scale_max * triangular formation
            scale_min = 0.8; scale_max = 1.3
            if norm(V) < scale_min:
                v = scale_min*V / norm(V); u = scale_min*U / norm(V)
            elif norm(V) > scale_max:
                v = scale_max*V / norm(V); u = scale_max*U / norm(V)
            else:
                v = V; u = U
        else:
        	v = V / norm(V); u = U / norm(U)

        # drones formation
        des2 = next_point1 - v*R_swarm*sqrt(3)/2 + u*R_swarm/2 # follower
        des3 = next_point1 - v*R_swarm*sqrt(3)/2 - u*R_swarm/2 # follower
        des4 = next_point1 - v*R_swarm*sqrt(3)				   # follower
        # following drones are attracting to desired points - vertices of the triangle formation
        f2 = combined_potential(obstacles_poses, des2)
        next_point2, _ = gradient_planner(f2, des2)

        f3 = combined_potential(obstacles_poses, des3)
        next_point3, _ = gradient_planner(f3, des3)

        f4 = combined_potential(obstacles_poses, des4)
        next_point4, _ = gradient_planner(f3, des4)

        route1 = np.vstack([route1, next_point1])
        route2 = np.vstack([route2, next_point2])
        route3 = np.vstack([route3, next_point3])
        route4 = np.vstack([route4, next_point4])

        current_point1 = next_point1

        dist_to_goal = norm(current_point1 - goal)
        if dist_to_goal < 0.1:
            print('\nReached the goal')
            break



        progress_bar.next()
        plt.cla()

        draw_map(obstacles_poses)
        draw_robots()
        if animate:
            plt.draw()
            plt.pause(0.01)

        if should_write_movie:
            movie_writer.grab_frame()

    print('\nDone')
    progress_bar.finish()
    plt.show()


# TODO:
# local minimum problem (FM2 - algorithm: https://pythonhosted.org/scikit-fmm/)
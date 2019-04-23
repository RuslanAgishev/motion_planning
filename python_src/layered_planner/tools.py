#!/usr/bin/env python

import numpy as np
from numpy.linalg import norm
from math import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon

# ### Helper functions

def draw_map(obstacles):
    # Obstacles. An obstacle is represented as a convex hull of a number of points. 
    # First row is x, second is y (position of vertices)

    # Bounds on world
    world_bounds_x = [-2.5, 2.5]
    world_bounds_y = [-2.5, 2.5]

    # Draw obstacles
    ax = plt.gca()
    ax.set_xlim(world_bounds_x)
    ax.set_ylim(world_bounds_y)
    for k in range(len(obstacles)):
        ax.add_patch( Polygon(obstacles[k], color='k') )


def draw_gradient(f, nrows=500, ncols=500):
    skip = 10
    [x_m, y_m] = np.meshgrid(np.linspace(-2.5, 2.5, ncols), np.linspace(-2.5, 2.5, nrows))
    [gy, gx] = np.gradient(-f);
    Q = plt.quiver(x_m[::skip, ::skip], y_m[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip])


def waypts2setpts(P, params):
	"""
	construct a long array of setpoints, traj_global, with equal inter-distances, dx,
	from a set of via-waypoints, P = [[x0,y0], [x1,y1], ..., [xn,yn]]
	"""
	V = params.drone_vel * 1.3 # [m/s], the setpoint should travel a bit faster than the robot
	freq = params.ViconRate; dt = 1./freq
	dx = V * dt
	traj_global = np.array(P[-1])
	for i in range(len(P)-1, 0, -1):
		A = P[i]
		B = P[i-1]

		n = (B-A) / norm(B-A)
		delta = n * dx
		N = int( norm(B-A) / norm(delta) )
		sp = A
		traj_global = np.vstack([traj_global, sp])
		for i in range(N):
			sp += delta
			traj_global = np.vstack([traj_global, sp])
		sp = B
		traj_global = np.vstack([traj_global, sp])

	return traj_global


def formation(num_robots, leader_des, v, l):
    """
    geometry of the swarm: following robots desired locations
    relatively to the leader
    """
    u = np.array([-v[1], v[0]])
    """ followers positions """
    des2 = leader_des - v*l*sqrt(3)/2 + u*l/2
    des3 = leader_des - v*l*sqrt(3)/2 - u*l/2
    des4 = leader_des - v*l*sqrt(3)
    des5 = leader_des - v*l*sqrt(3)   + u*l
    des6 = leader_des - v*l*sqrt(3)   - u*l
    des7 = leader_des - v*l*sqrt(3)*3/2 - u*l/2
    des8 = leader_des - v*l*sqrt(3)*3/2 + u*l/2
    des9 = leader_des - v*l*sqrt(3)*2
    if num_robots<=1: return []
    if num_robots==2: return [des4]
    if num_robots==3: return [des2, des3]
    if num_robots==4: return [des2, des3, des4]
    if num_robots==5: return [des2, des3, des4, des5]
    if num_robots==6: return [des2, des3, des4, des5, des6]
    if num_robots==7: return [des2, des3, des4, des5, des6, des7]
    if num_robots==8: return [des2, des3, des4, des5, des6, des7, des8]
    if num_robots==9: return [des2, des3, des4, des5, des6, des7, des8, des9]
    
    return [des2, des3, des4]

def normalize(vector):
	if norm(vector)==0: return vector
	return vector / norm(vector)

def poses2polygons(poses, l=0.1):
    polygons = []
    for pose in poses:
        pose = np.array(pose)
        polygon = np.array([pose + [-l/2,-l/2], pose + [l/2,-l/2], pose + [l/2,l/2], pose + [-l/2,l/2]])
        polygons.append(polygon)
    return polygons
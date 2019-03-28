#!/usr/bin/env python

import numpy as np
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
        ax.add_patch( Polygon(obstacles[k]) )


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
	V = params.drone_vel # [m/s]
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
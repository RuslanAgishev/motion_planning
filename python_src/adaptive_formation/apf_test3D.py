import numpy as np
from numpy.linalg import norm
from math import *
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path
import time
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection


def meters2grid(pose_m, nx=500, ny=500):
    # [0, 0, 0](m) -> [250, 250, 0]
    # [1, 0, 0](m) -> [250+100, 250, 0]
    # [0,-1, 0](m) -> [250, 250-100]
    # [0, 0, 1](m) -> [250, 250, 100]
    pose_on_grid = np.array(pose_m)*100 + np.array([nx/2, ny/2, 0])
    return np.array( pose_on_grid, dtype=int)
def grid2meters(pose_grid, nx=500, ny=500):
    # [250, 250, 0] -> [0, 0, 0](m)
    # [250+100, 250, 0] -> [1, 0, 0](m)
    # [250, 250-100, 100] -> [0,-1, 1](m)
    pose_meters = ( np.array(pose_grid) - np.array([nx/2, ny/2, 0]) ) / 100.0
    return pose_meters

def plot_point3D(p, color='blue'):
    ax.scatter3D(p[0], p[1], p[2], color=color)

# init_fonts()
fig = plt.figure(figsize=(15,15))
ax = plt.axes(projection='3d')
ax.set_xlabel('X, [m]')
ax.set_ylabel('Y, [m]')
ax.set_zlabel('Z, [m]')
ax.set_xlim([-2.5, 2.5])
ax.set_ylim([-2.5, 2.5])
ax.set_zlim([0.0, 3.0])

# Start and goal positions
start = np.array([0.0, 0.0, 0.0]); ax.scatter3D(start[0], start[1], start[2], color='green', s=100)
goal =  np.array([0.0, 1.0, 2.5]);  ax.scatter3D(goal[0], goal[1], goal[2], color='red', s=100)


# while not nearGoal and iters < maxiters:
#     plot_point3D(p, 'red')
#     plot_point3D(new_node.p, color='blue')
    # ax.plot([closest_node.p[0], new_node.p[0]], [closest_node.p[1], new_node.p[1]], [closest_node.p[2], new_node.p[2]],color = 'k', zorder=5)
    # plt.pause(0.01)

plt.show()  


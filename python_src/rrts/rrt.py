#!/usr/bin/env python
# coding: utf-8

import numpy as np
from numpy.linalg import norm
from math import *
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from numpy.random import uniform
from scipy.spatial import ConvexHull
from matplotlib import path
import time


# Helper functions
def isCollisionFreeVertex(obstacles, xy):
	# the function calculates value:
	# collFree = [xy-point is outside obstacles map]
    collFree = True

    for obstacle in obstacles:
        hull = path.Path(obstacle)
        collFree = not hull.contains_points([xy])
        if hull.contains_points([xy]):
            return False

    return collFree

def isCollisionFreeEdge(obstacles, closest_vert, xy):
    closest_vert = np.array(closest_vert); xy = np.array(xy)
    collFree = True
    l = norm(closest_vert - xy)
    map_resolution = 0.01; M = int(l / map_resolution)
    if M <= 2: M = 3
    t = np.linspace(0,1,M)
    for i in range(1,M-1):
        p = (1-t[i])*closest_vert + t[i]*xy # calculate configuration
        collFree = isCollisionFreeVertex(obstacles, p) 
        if collFree == False:
            return False

    return collFree

def closestVertex(rrt_verts, xy):
    distance = np.zeros(len(rrt_verts[0,:]))
    for k in range( len(rrt_verts[0,:]) ):
        distance[k] = sqrt((xy[0] - rrt_verts[0,k])**2 + (xy[1] - rrt_verts[1,k])**2)
    dmin = min(distance)
    ind_min = distance.tolist().index(dmin)
    
    closest_vert = rrt_verts[:,ind_min]

    return closest_vert


animate = 1


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
              np.array([[0.8, 1.5], [1+w, 1.5], [1+w, 1.5+w], [0.8, 1.5+w]])
            ]
# obstacles = []

# Bounds on world
world_bounds_x = [-2.5, 2.5]
world_bounds_y = [-2.5, 2.5]


# Draw obstacles
fig = plt.figure(figsize=(10, 10))
plt.grid()
ax = plt.gca()
ax.set_xlim(world_bounds_x)
ax.set_ylim(world_bounds_y)
for k in range(len(obstacles)):
    ax.add_patch( Polygon(obstacles[k]) )

# Start and goal positions
xy_start = np.array([0.7, 0.4]); plt.plot(xy_start[0],xy_start[1],'bo',color='red', markersize=20)
xy_goal =  np.array([-1.0, 1.0]);  plt.plot(xy_goal[0], xy_goal[1], 'bo',color='green',markersize=20)

# Initialize RRT. The RRT will be represented as a 2 x N list of points. So each column represents a vertex of the tree.
rrt_verts = xy_start[np.newaxis].T
nearGoal = False # This will be set to true if goal has been reached
minDistGoal = 0.1 # This is the convergence criterion. We will declare success when the tree reaches within 0.25 in distance from the goal.
# Extension parameter
d = 0.15 # This controls how far the RRT extends in each step.

# RRT algorithm
start_time = time.time()
while not nearGoal:
# for i in range(1000):
    # Sample point
    rnd = uniform()
    # With probability 0.05, sample the goal. This promotes movement to the goal.
    if rnd < 0.05:
        xy = xy_goal
    else:
        # Sample (uniformly) from space (with probability 0.95). The space is defined
        # with the bounds world_bounds_x and world_bounds_y defined above.
        # So, the x coordinate should be sampled in the interval
        # world_bounds_x=2.5 and the y coordinate from world_bounds_y=2.5.
        xy = np.array([uniform()*5-2.5, uniform()*5-2.5]) # Should be a 2 x 1 vector
    if abs(xy[0])>2.5 or abs(xy[1])>2.5: print "ERROR: not correct sample point"
    # Check if sample is collision free
    collFree = isCollisionFreeVertex(obstacles, xy)
    # If it's not collision free, continue with loop
    if not collFree:
        continue

    # If it is collision free, find closest point in existing tree. 
    closest_vert = closestVertex(rrt_verts, xy)
    
    # Extend tree towards xy from closest_vert. Use the extension parameter
    # d defined above as your step size. In other words, the Euclidean
    # distance between new_vert and closest_vert should be d.
    new_vert = closest_vert + d*(xy - closest_vert)

    # Check if new vertice is in collision
    collFree = isCollisionFreeEdge(obstacles, closest_vert, new_vert)
    # If it's not collision free, continue with loop
    if not collFree:
        continue
        
    if animate:
        plt.plot(xy[0], xy[1], 'ro', color='k')
        plt.plot(new_vert[0], new_vert[1], 'bo',color = 'blue', markersize=5) # VERTICES
        plt.plot([closest_vert[0], new_vert[0]], [closest_vert[1], new_vert[1]], color='blue') # EDGES
        plt.draw()
        plt.pause(0.01)

    # If it is collision free, add it to tree    
    rrt_verts = np.hstack([rrt_verts, new_vert[np.newaxis].T])

    # Check if we have reached the goal
    if norm(np.array(xy_goal) - np.array(new_vert)) < minDistGoal:
    	end_time = time.time()
        print 'Reached the goal after %f seconds:' % (end_time - start_time)
    	nearGoal = True
        break

if not animate:
    plt.plot(rrt_verts[0,:], rrt_verts[1,:], 'ro', color='blue')
    plt.draw()
    plt.pause(0.1)
raw_input('Hit Enter to close')
plt.close('all')
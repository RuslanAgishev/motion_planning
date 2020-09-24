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

from tools import *

class Node:
    def __init__(self):
        self.p     = [0, 0] # node XY-position
        self.i     = 0 # node id
        self.iPrev = 0 # node parent's id

def closestNode(rrt, p):
    distance = []
    for node in rrt:
        distance.append( sqrt((p[0] - node.p[0])**2 + (p[1] - node.p[1])**2) )
    distance = np.array(distance)
    
    dmin = min(distance)
    ind_min = distance.tolist().index(dmin)
    closest_node = rrt[ind_min]

    return closest_node


def rrt_path(obstacles, xy_start, xy_goal, params):
    # Initialize RRT. The RRT will be represented as a list of nodes.
    # So each column represents a vertex of the tree.
    rrt = []
    start_node = Node()
    start_node.p = xy_start
    start_node.i = 0
    start_node.iPrev = 0
    rrt.append(start_node)
    nearGoal = False # This will be set to true if goal has been reached
    minDistGoal = params.minDistGoal # Convergence criterion: success when the tree reaches within 0.25 in distance from the goal.
    d = params.extension # Extension parameter: this controls how far the RRT extends in each step.

    # RRT algorithm
    start_time = time.time()
    iters = 0
    print('Configuration space sampling started ...')
    while not nearGoal: # and iters < maxiters:
        # Sample point
        rnd = random()
        # With probability goal_prob, sample the goal. This promotes movement to the goal.
        if rnd < params.goal_prob:
            xy = xy_goal
        else:
            # Sample (uniformly) from space (with probability 0.95). The space is defined
            # with the bounds world_bounds_x and world_bounds_y defined above.
            # So, the x coordinate should be sampled in the interval
            # world_bounds_x=2.5 and the y coordinate from world_bounds_y=2.5.
            xy = np.array([random()*2*params.world_bounds_x[1]-params.world_bounds_x[1], random()*2*params.world_bounds_x[1]-params.world_bounds_x[1]]) # Should be a 2 x 1 vector
        # Check if sample is collision free
        collFree = isCollisionFreeVertex(obstacles, xy)
        # If it's not collision free, continue with loop
        if not collFree:
            iters += 1
            continue

        # If it is collision free, find closest point in existing tree. 
        closest_node = closestNode(rrt, xy)
        
        # Extend tree towards xy from closest_vert. Use the extension parameter
        # d defined above as your step size. In other words, the Euclidean
        # distance between new_vert and closest_vert should be d.
        new_node = Node()
        new_node.p = closest_node.p + d * (xy - closest_node.p)
        new_node.i = len(rrt)
        new_node.iPrev = closest_node.i

        # Check if new vertice is in collision
        collFree = isCollisionFreeEdge(obstacles, closest_node.p, new_node.p)
        # If it's not collision free, continue with loop
        if not collFree:
            iters += 1
            continue
        
        if params.animate:
            # plt.plot(xy[0], xy[1], 'ro', color='k')
            plt.plot(new_node.p[0], new_node.p[1], 'bo',color = 'blue', markersize=5) # VERTICES
            plt.plot([closest_node.p[0], new_node.p[0]], [closest_node.p[1], new_node.p[1]], color='blue') # EDGES
            plt.draw()
            plt.pause(0.01)


        # If it is collision free, add it to tree    
        rrt.append(new_node)

        # Check if we have reached the goal
        if norm(np.array(xy_goal) - np.array(new_node.p)) < minDistGoal:
            # Add last, goal node
            goal_node = Node()
            goal_node.p = xy_goal
            goal_node.i = len(rrt)
            goal_node.iPrev = new_node.i
            if isCollisionFreeEdge(obstacles, new_node.p, goal_node.p):
                rrt.append(goal_node)
                P = [goal_node.p]
            else: P = []

            end_time = time.time()
            nearGoal = True
            print('Reached the goal after %.2f seconds:' % (end_time - start_time))

        iters += 1

    print('Number of iterations passed: %d / %d' %(iters, params.maxiters))
    print('RRT length: ', len(rrt))

    # Path construction from RRT:
    print('Constructing the path...')
    i = len(rrt) - 1
    while True:
        i = rrt[i].iPrev
        P.append(rrt[i].p)
        if i == 0:
            print('Reached RRT start node')
            break
    P = np.array(P)

    return P
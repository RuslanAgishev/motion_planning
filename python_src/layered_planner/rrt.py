#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm
from math import *
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path
import time

##### RRT algorithm

# Helper functions

def isCollisionFreeVertex(obstacles, xy):
    collFree = True

    for obstacle in obstacles:
        hull = path.Path(obstacle)
        collFree = not hull.contains_points([xy])
        if hull.contains_points([xy]):
            # print 'collision'
            return collFree

    return collFree


def isCollisionFreeEdge(obstacles, closest_vert, xy):
    closest_vert = np.array(closest_vert); xy = np.array(xy)
    collFree = True
    l = norm(closest_vert - xy)
    map_resolution = 0.01; M = int(l / map_resolution)
    if M <= 2: M = 20
    t = np.linspace(0,1,M)
    for i in range(1,M-1):
        p = (1-t[i])*closest_vert + t[i]*xy # calculate configuration
        collFree = isCollisionFreeVertex(obstacles, p) 
        if collFree == False: return False

    return collFree


# RRT algorithm

class Node:
    def __init__(self):
        self.p     = [0, 0]
        self.i     = 0
        self.iPrev = 0


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
            print('RRT is constructed after %.2f seconds:' % (end_time - start_time))

        iters += 1

    # print 'Number of iterations passed: %d / %d' %(iters, params.maxiters)
    # print 'RRT length: ', len(rrt)

    # Path construction from RRT:
    print('Retriving the path from RRT...')
    i = len(rrt) - 1
    while True:
        i = rrt[i].iPrev
        P.append(rrt[i].p)
        if i == 0:
            # print 'Reached RRT start node'
            break
    P = np.array(P)
    # plt.plot( P[:,0], P[:,1], color='green', linewidth=5, label='path from RRT' )

    return P


def ShortenPath(P, obstacles, smoothiters=10):
    # INPUTS
    #   P - path to get smoothed (after RRT algorithm)
    #   obstacles - says where the obstacles are
    #   smoothiters - maximum number of smoothing iterations
    #
    # OUTPUTS
    #   P_smoothed - a path, same format as before:  
    #    P_smoothed = [q1 q2 q3 ... qM]
    #               where q1=qstart and qM=qgoal; in other words, the sequence
    #               of straight-line paths from q1 to q2, q2 to q3, etc., takes
    #               the robot from start to goal without collision
    m = P.shape[0]
    l = np.zeros(m)
    for k in range(1, m):
        l[k] = norm(P[k,:]-P[k-1,:]) + l[k-1] # find all of the straight-line distances
    iters = 0
    while iters < smoothiters:
        s1 = random()*l[m-1] 
        s2 = random()*l[m-1]
        if s2 < s1:
            temps = s1
            s1 = s2
            s2 = temps
        for k in range(1, m):
            if s1 < l[k]:
                i = k - 1
                break
        for k in range(i, m):
            if s2 < l[k]:
                j = k - 1
                break
        if (j <= i):
            iters = iters + 1
            continue
        t1 = (s1 - l[i]) / (l[i+1]-l[i])
        gamma1 = (1 - t1)*P[i,:] + t1*P[i+1,:]
        t2 = (s2 - l[j]) / (l[j+1]-l[j])
        gamma2 = (1 - t2)*P[j,:] + t2*P[j+1,:]
        
        collisionFree = isCollisionFreeEdge(obstacles, gamma1, gamma2)
        if collisionFree == 0:
            iters = iters + 1
            continue
#         print round(l[i],2), round(s1,2), round(l[i+1],2)
#         plt.plot(P[i,0], P[i,1], 'ro', markersize=10, color='red')
#         plt.plot(gamma1[0], gamma1[1], 'ro', markersize=10, color='green')
#         plt.plot(P[i+1,0], P[i+1,1], 'ro', markersize=10, color='blue')
#         plt.plot(P[j,0], P[j,1], 'ro', markersize=10, color='red')
#         plt.plot(gamma2[0], gamma2[1], 'ro', markersize=10, color='green')
#         plt.plot(P[j+1,0], P[j+1,1], 'ro', markersize=10, color='blue')
#         plt.plot([gamma1[0], gamma2[0]], [gamma1[1], gamma2[1]], color='k', linewidth=5)
#         print round(l[j],2), round(s2,2), round(l[j+1],2)
        P = np.vstack([P[:(i+1),:], gamma1, gamma2, P[(j+1):,:]])
        m = P.shape[0]
        l = np.zeros(m)
        for k in range(1, m):
            l[k] = norm( P[k,:] - P[k-1,:] ) + l[k-1]
        iters = iters + 1
#         plt.plot(P[:,0], P[:,1], '--', linewidth=3)
    P_short = P
    
    return P_short
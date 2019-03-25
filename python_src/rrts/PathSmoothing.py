#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm
from matplotlib import pyplot as plt
from random import random
from tools import *

def SmoothPath(P, obstacles, smoothiters=10):
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
        P = np.vstack([P[:(i+1),:], gamma1, gamma2, P[(j+1):m,:]])
        m = P.shape[0]
        l = np.zeros(m)
        for k in range(1, m):
            l[k] = norm( P[k,:] - P[k-1,:] ) + l[k-1]
        iters = iters + 1
#         plt.plot(P[:,0], P[:,1], '--', linewidth=3)
    P_smooth = P
    
    return P_smooth
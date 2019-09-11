# coding: utf-8

# Lidar or multiranger data to 2D grid map
# This script shows how to read LIDAR (range) measurements from a file and convert it to occupancy grid.
# Occupancy grid maps (_Hans Moravec, A.E. Elfes:
# High resolution maps from wide angle sonar, Proc. IEEE Int. Conf. Robotics Autom. (1985)_)
# are a popular, probabilistic approach to represent the environment.
# The grid is basically discrete representation of the environment,
# which shows if a grid cell is occupied or not.
# Here the map is represented as a `numpy array`, and numbers close to 1 means the cell is occupied
# (_marked with red on the next image_), numbers close to 0 means they are free (_marked with green_).
# The grid has the ability to represent unknown (unobserved) areas, which are close to 0.5.
# 
# ![Example](grid_map_example.png)
# 
# In order to construct the grid map from the measurement we need to discretise the values.

import pandas as pd
import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi
from collections import deque
import sys

# The measurement file contains the laser beam coordinates measurements
# in a `csv` (comma separated values) format.
def file_read(filename):
    """
    Reading LIDAR laser beams (x,y and z global coordinates of the scans)
    """
    points = np.array( pd.read_csv(filename) )

    poses = points[:,:3]
    measurements = points[:,3:]

    return poses, measurements



# Handy functions which can used to convert a 2D range measurement to a grid map.
# For example the `bresenham`  gives the a straight line between two points in a grid map.
def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx) # determine how steep the line is
    if is_steep: # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    swapped = False # swap start and end points if necessary and store swap state
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1 # recalculate differentials
    dy = y2 - y1 # recalculate differentials
    error = int(dx / 2.0) # calculate error
    ystep = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)   
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped: # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points


EXTEND_AREA = 3.0
def calc_grid_map_config(ox, oy, xyreso):
    """
    Calculates the size, and the maximum distances according to the the measurement center
    """
    minx = round(min(ox) - EXTEND_AREA / 2.0)
    miny = round(min(oy) - EXTEND_AREA / 2.0)
    maxx = round(max(ox) + EXTEND_AREA / 2.0)
    maxy = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))
    # print("The grid map is ", xw, "x", yw, ".")
    return minx, miny, maxx, maxy, xw, yw

def meters2grid(pose_m, nrows=500, ncols=500):
    # [0, 0](m) -> [250, 250]
    # [1, 0](m) -> [250+100, 250]
    # [0,-1](m) -> [250, 250-100]
    if np.isscalar(pose_m):
        pose_on_grid = int( pose_m*100 + ncols/2 )
    else:
        pose_on_grid = np.array( np.array(pose_m)*100 + np.array([ncols/2, nrows/2]), dtype=int )
    return pose_on_grid

def update_ray_casting_grid_map(pmap, scan_x, scan_y, robot_x, robot_y, params):
    """
    The breshen boolean tells if it's computed with bresenham ray casting (True) or with flood fill (False)
    """
    minx, miny, maxx, maxy = params['minx'], params['miny'], params['maxx'], params['maxy']
    xyreso = params['xyreso']

    robot_ix = int(round( (robot_x - minx) / xyreso))
    robot_iy = int(round( (robot_y - miny) / xyreso))

    # occupancy grid computed with bresenham ray casting
    for (x, y) in zip(scan_x, scan_y):
        ix = int(round((x - minx) / xyreso)) # x coordinate of the the occupied area
        iy = int(round((y - miny) / xyreso)) # y coordinate of the the occupied area

        laser_beams = bresenham((robot_ix, robot_iy), (ix, iy)) # line form the lidar to the cooupied point
        for laser_beam in laser_beams:
            pmap[laser_beam[0]][laser_beam[1]] = 0.0 # free area 0.0
        pmap[ix][iy] = 1.0     # occupied area 1.0
        pmap[ix+1][iy] = 1.0   # extend the occupied area
        pmap[ix][iy+1] = 1.0   # extend the occupied area
        pmap[ix+1][iy+1] = 1.0 # extend the occupied area

    return pmap

def plot_robot(robot_ix, robot_iy, robot_path):
    plt.plot(robot_iy, robot_ix, 'ro', markersize=5)
    path = np.array(robot_path)
    plt.plot(path[:,1], path[:,0], color='k', linewidth=2)

params = {
    'minx': -3.0,
    'miny': -3.0,
    'maxx': 3.0,
    'maxy': 3.0,
    'xyreso': 0.04
}

animate = 0

if __name__ == '__main__':

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'csvs/coordsXYZ1567005444.69.csv'
    poses, measurements = file_read(filename)

    # remove points without movement
    poses = poses[380:1580, :]
    measurements = measurements[380:1580, :]

    fig = plt.figure(figsize=(10,10))
    plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)

    minx, miny, maxx, maxy = params['minx'], params['miny'], params['maxx'], params['maxy']
    xyreso = params['xyreso']
    wx = int(round( (maxx - minx) / xyreso)) # width of the grid map
    wy = int(round( (maxy - miny) / xyreso)) # height of the grid map

    pmap = np.ones((wx, wy))/2

    robot_path = []

    for i in range(poses.shape[0]):
        scan_x = np.array( [measurements[i,0],
                            measurements[i,3],
                            measurements[i,6],
                            measurements[i,9]
                            ] )
        scan_y = np.array( [measurements[i,1],
                            measurements[i,4],
                            measurements[i,7],
                            measurements[i,10]
                            ] )
        # remove nans:
        scan_x = [x for x in scan_x if str(x) != 'nan']
        scan_y = [y for y in scan_y if str(y) != 'nan']

        robot_x = poses[i,0]; robot_y = poses[i,1]
        pmap = update_ray_casting_grid_map(pmap, scan_x, scan_y, robot_x, robot_y, params)
        
        robot_ix = int(round( (robot_x - minx) / xyreso))
        robot_iy = int(round( (robot_y - miny) / xyreso))
        robot_path.append( [robot_ix, robot_iy] )

        xyres = np.array(pmap).shape
        if animate:
            plt.cla()
            plt.imshow(pmap, cmap = "PiYG_r")
            plt.clim(-0.4, 1.4)
            plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor = True)
            plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor = True)
            plot_robot(robot_ix, robot_iy, robot_path)

            plt.pause(0.1)
    
    plt.cla()
    plt.imshow(pmap, cmap = "PiYG_r")
    plt.clim(-0.4, 1.4)
    plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor = True)
    plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor = True)
    plot_robot(robot_ix, robot_iy, robot_path)

    # close windows if Enter-button is pressed
    plt.pause(0.1)
    input('Hit Enter to close')
    plt.close('all')

    # save resulting map as a numpy array
    np.save('gmap.npy', pmap)


import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from grid_map import flood_fill, bresenham

def flood_fill(cpoint, pmap):
    """
    cpoint: starting point (x,y) of fill
    pmap: occupancy map generated from Bresenham ray-tracing
    """
    # Fill empty areas with queue method
    sx, sy = pmap.shape
    fringe = deque()
    fringe.appendleft(cpoint)
    while fringe:
        n = fringe.pop()
        nx, ny = n
        # West
        if nx > 0:
            if pmap[nx - 1, ny] == 1.0:
                pmap[nx - 1, ny] = 0.0
                fringe.appendleft((nx - 1, ny))
        # East
        if nx < sx - 1:
            if pmap[nx + 1, ny] == 1.0:
                pmap[nx + 1, ny] = 0.0
                fringe.appendleft((nx + 1, ny))
        # North
        if ny > 0:
            if pmap[nx, ny - 1] == 1.0:
                pmap[nx, ny - 1] = 0.0
                fringe.appendleft((nx, ny - 1))
        # South
        if ny < sy - 1:
            if pmap[nx, ny + 1] == 1.0:
                pmap[nx, ny + 1] = 0.0
                fringe.appendleft((nx, ny + 1))
    return pmap

class Params:
	def __init__(self):
		self.map_center = np.array([-0.5, 0.5])
		self.map_width_m = 1.0
		self.map_length_m = 1.0
		self.map_resolution_m = 0.01 # [m]
		self.sensor_range_m = 0.1
		self.wall_thickness_m = 1.0*self.sensor_range_m

		self.create_borders_grid_map(points = [[20,20], [30,70], [60,50], [90,10]])

	def create_borders_grid_map(self, points):
		WIDTH = int(100 * (self.map_width_m))
		LENGTH = int(100 * (self.map_length_m))
		border = int(100 * self.wall_thickness_m)
		gmap = np.ones([WIDTH, LENGTH])

		xs = []; ys = []
		points.append(points[0])
		for i in range(len(points)-1):
			xs.append(points[i][0]); ys.append(points[i][1]);
			line = bresenham(points[i], points[i+1])
			for l in line:
				gmap[l[0]][l[1]] = 0

		gmap = flood_fill((int(np.mean(xs)), int(np.mean(ys))), gmap)

		self.gmap = gmap

params = Params()



plt.figure()
gmap = params.gmap

plt.imshow(1-gmap, cmap='gray')

plt.show()
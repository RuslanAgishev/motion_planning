"""
Random walk algorithm implementation for a mobile robot
equipped with 4 ranger sensors (front, back, left and right)
for obstacles detection

author: Ruslan Agishev (agishev_ruslan@mail.ru)
reference: https://ieeexplore.ieee.org/abstract/document/6850799/s
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from collections import deque
from grid_map import GridMap
    

def plot_robot(pose, gridmap_params):
	r = gridmap_params.sensor_range_m
	plt.plot([pose[0]-r*np.cos(pose[2]), pose[0]+r*np.cos(pose[2])],
			 [pose[1]-r*np.sin(pose[2]), pose[1]+r*np.sin(pose[2])], '--', color='b')
	plt.plot([pose[0]-r*np.cos(pose[2]+np.pi/2), pose[0]+r*np.cos(pose[2]+np.pi/2)],
		     [pose[1]-r*np.sin(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)], '--', color='b')
	plt.plot(pose[0], pose[1], 'ro', markersize=5)
	plt.arrow(pose[0], pose[1], 0.05 * np.cos(pose[2]), 0.05 * np.sin(pose[2]),
              head_length=0.05, head_width=0.05)

def obstacle_check(pose, gridmap_params):
	gmap = gridmap_params.gmap

	r = int(100*gridmap_params.sensor_range_m)
	back = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
	front = [pose[0]+r*np.cos(pose[2]), pose[1]+r*np.sin(pose[2])]
	right = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
	left = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]

	pi = np.array(pose[:2], dtype=int)
	backi = np.array(back, dtype=int)
	fronti = np.array(front, dtype=int)
	lefti = np.array(left, dtype=int)
	righti = np.array(right, dtype=int)

	obstacle = {
		'front': 0,
		'back':  0,
		'right': 0,
		'left':  0,
	}

	for i in np.arange(min(pi[0], fronti[0]), max(pi[0], fronti[0])+1):
		for j in np.arange(min(pi[1], fronti[1]), max(pi[1], fronti[1])+1):
			m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
			if gmap[m,n]:
				# print('FRONT collision')
				obstacle['front'] = 1

	for i in np.arange(min(pi[0], backi[0]), max(pi[0], backi[0])+1):
		for j in np.arange(min(pi[1], backi[1]), max(pi[1], backi[1])+1):
			m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
			if gmap[m,n]:
				# print('BACK collision')
				obstacle['back'] = 1

	for i in np.arange(min(pi[0], lefti[0]), max(pi[0], lefti[0])+1):
		for j in np.arange(min(pi[1], lefti[1]), max(pi[1], lefti[1])+1):
			m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
			if gmap[m,n]:
				# print('LEFT collision')
				obstacle['left'] = 1

	for i in np.arange(min(pi[0], righti[0]), max(pi[0], righti[0])+1):
		for j in np.arange(min(pi[1], righti[1]), max(pi[1], righti[1])+1):
			m = min(j, gmap.shape[0]-1); n = min(i, gmap.shape[1]-1)
			if gmap[m,n]:
				# print('RIGHT collision')
				obstacle['right'] = 1

	return obstacle



def left_shift(pose, r):
	left = [pose[0]+r*np.cos(pose[2]+np.pi/2), pose[1]+r*np.sin(pose[2]+np.pi/2)]
	return left
def right_shift(pose, r):
	right = [pose[0]-r*np.cos(pose[2]+np.pi/2), pose[1]-r*np.sin(pose[2]+np.pi/2)]
	return right
def back_shift(pose, r):
	back = pose
	back[:2] = [pose[0]-r*np.cos(pose[2]), pose[1]-r*np.sin(pose[2])]
	return back

def visualize(traj, pose, gridmap_params):
	plt.plot(traj[:,0], traj[:,1], 'g')
	plot_robot(pose, gridmap_params)
	plt.legend()
		

class Params:
	def __init__(self):
		self.simulation_time = 10 # [sec]
		self.numiters = 1000
		self.animate = 1
		self.vel = 0.5 # [m/s]


def main():
	obstacles = [
		np.array([[0.7, -0.9], [1.3, -0.9], [1.3, -0.8], [0.7, -0.8]]) + np.array([-1.0, 0.5]),
		np.array([[0.7, -0.9], [1.3, -0.9], [1.3, -0.8], [0.7, -0.8]]) + np.array([-1.0, 1.0]),
		np.array([[0.7, -0.9], [0.8, -0.9], [0.8, -0.3], [0.7, -0.3]]) + np.array([-1.0, 0.5]),        
	]
	params = Params()
	gridmap = GridMap()
	gridmap.add_obstacles_to_grid_map(obstacles)
	#    x,    y,      yaw
	pose = [0.2, 0.5, -np.pi/3]
	traj = pose[:2]
	plt.figure(figsize=(10,10))
	gridmap.draw_map(obstacles)
	plt.plot(pose[0], pose[1], 'ro', markersize=20, label='Initial position')
	# while True:
	for _ in range(params.numiters):
		dv = 0.1*params.vel
		pose[0] += dv*np.cos(pose[2])
		pose[1] += dv*np.sin(pose[2])

		pose_grid = gridmap.meters2grid(pose[:2])
		boundary = obstacle_check([pose_grid[0], pose_grid[1], pose[2]], gridmap)
		# print(boundary)

		if boundary['right'] or boundary['front']:
			pose = back_shift(pose, 0.03)
			pose[2] -= np.pi/2 * np.random.uniform(0.2, 0.6)
		elif boundary['left']:
			pose = back_shift(pose, 0.03)
			pose[2] += np.pi/2 * np.random.uniform(0.2, 0.6)

		traj = np.vstack([traj, pose[:2]])
		
		if params.animate:
			plt.cla()
			gridmap.draw_map(obstacles)
			visualize(traj, pose, gridmap)
			plt.pause(0.1)

	visualize(traj, pose, gridmap)
	plt.show()

if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
	    pass
		
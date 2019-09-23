"""
Coverage path planning (CPP) algorithm implementation for a mobile robot
equipped with 4 ranger sensors (front, back, left and right)
for obstacles detection.

author: Ruslan Agishev (agishev_ruslan@mail.ru)
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import numpy as np
import math
from grid_map import GridMap
from grid_based_sweep_coverage_path_planner import planning
import time
from tools import define_polygon, polygon_contains_point
from tqdm import tqdm

def plot_robot(ax, pose, params):
	r = params.sensor_range_m
	ax.plot([pose[0]-r*np.cos(pose[3]), pose[0]+r*np.cos(pose[3])],
			 [pose[1]-r*np.sin(pose[3]), pose[1]+r*np.sin(pose[3])],
			 [pose[2], pose[2]], '--', linewidth=1, color='b')
	ax.plot([pose[0]-r*np.cos(pose[3]+np.pi/2), pose[0]+r*np.cos(pose[3]+np.pi/2)],
		     [pose[1]-r*np.sin(pose[3]+np.pi/2), pose[1]+r*np.sin(pose[3]+np.pi/2)],
		     [pose[2], pose[2]], '--', linewidth=1, color='b')
	ax.scatter(pose[0], pose[1], pose[2], marker='^')
	ax.quiver(pose[0], pose[1], pose[2], np.cos(pose[3]), np.sin(pose[3]), 0.0, length=0.2, normalize=True)



def obstacle_check(pose, gridmap, params):
	gmap = gridmap

	r = int(100*params.sensor_range_m)
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
	yaw = pose[3]
	left = [pose[0]+r*np.cos(yaw+np.pi/2), pose[1]+r*np.sin(yaw+np.pi/2)]
	return left
def right_shift(pose, r):
	yaw = pose[3]
	right = [pose[0]-r*np.cos(yaw+np.pi/2), pose[1]-r*np.sin(yaw+np.pi/2)]
	return right
def back_shift(pose, r):
	yaw = pose[3]
	back = pose
	back[:2] = [pose[0]-r*np.cos(yaw), pose[1]-r*np.sin(yaw)]
	return back
def forward_shift(pose, r):
	yaw = pose[3]
	forward = pose
	forward[:2] = [pose[0]+r*np.cos(yaw), pose[1]+r*np.sin(yaw)]
	return forward
def turn_left(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
	pose[3] -= yaw
	return pose
def turn_right(pose, yaw=np.pi/2*np.random.uniform(0.2, 0.6)):
	pose[3] += yaw
	return pose
def slow_down(state, params, dv=0.1):
	if state[4]>params.min_vel:
		state[4] -= dv
	return state
		

def motion(state, goal, params):
	# state = [x(m), y(m), z(m), yaw(rad), v(m/s), omega(rad/s)]
	dx = goal[0] - state[0]
	dy = goal[1] - state[1]
	goal_yaw = math.atan2(dy, dx)
	K_theta = 3
	state[5] = K_theta*math.sin(goal_yaw - state[3]) # omega(rad/s)
	state[3] += params.dt*state[5] # yaw(rad)

	dist_to_goal = np.linalg.norm(goal - state[:3])
	K_v = 0.1
	state[4] += K_v*dist_to_goal
	if state[4] >= params.max_vel: state[4] = params.max_vel
	if state[4] <= params.min_vel: state[4] = params.min_vel

	dv = params.dt*state[4]
	state[0] += dv*np.cos(state[3]) # x(m)
	state[1] += dv*np.sin(state[3]) # y(m)
	dist_to_goalZ = np.linalg.norm(goal[2] - state[2])
	K_z = 0.1
	state[2] -= K_z * dist_to_goalZ

	return state

def collision_avoidance(state, gridmap, params):
	pose_grid = gridmap.meters2grid(state[:2])
	yaw = state[3]
	boundary = obstacle_check([pose_grid[0], pose_grid[1], yaw], gridmap.gmap, params)
	# print(boundary)

	if boundary['right'] or boundary['front']:
		# state = back_shift(state, 0.03)
		state = slow_down(state, params)
		state = turn_left(state, np.radians(30))
		# state = forward_shift(state, 0.02)
	elif boundary['left']:
		# state = back_shift(state, 0.03)
		state = slow_down(state, params)
		state = turn_right(state, np.radians(30))
		# state = forward_shift(state, 0.02)
	return state

def define_flight_area(initial_pose):
	plt.grid()
	while True:
		try:
			num_pts = int( input('Enter number of polygonal vertixes: ') )
			break
		except:
			print('\nPlease, enter an integer number.')
	while True:
		flight_area_vertices = define_polygon(num_pts)
		if polygon_contains_point(initial_pose, flight_area_vertices):
			break
		plt.clf()
		plt.grid()
		print('The robot is not inside the flight area. Define again.')
	return flight_area_vertices

# def get_3D_waypoints(goal_x, goal_y, params):
# 	h_min = params.min_height
# 	h_max = params.max_height
# 	dh = params.sweep_resolution
def get_3D_waypoints(goal_x, goal_y, h_min, h_max, dh):
	height_levels = np.linspace( h_max, h_min, int((h_max-h_min)/dh) )
	waypoints = np.vstack([goal_x, goal_y, height_levels[0]*np.ones_like(goal_x)])
	for i in range(1, len(height_levels)):
		level_x = np.copy(goal_x)
		level_y = np.copy(goal_y)
		if i%2 == 1:
			level_x = np.flip(goal_x)
			level_y = np.flip(goal_y)
		level_z = height_levels[i]*np.ones_like(goal_x)
		waypoints1 = np.vstack([level_x, level_y, level_z])
		waypoints = np.hstack([waypoints, waypoints1])
	return waypoints.T # waypoints = [goal_x.T, goal_y.T, height_levels.T]

class Params:
	def __init__(self):
		self.numiters = 5000
		self.animate = 1
		self.dt = 0.1
		self.goal_tol = 0.15
		self.max_vel = 0.5 # m/s
		self.min_vel = 0.1 # m/s
		self.sensor_range_m = 0.3 # m
		self.time_to_switch_goal = 5.0 # sec
		self.sweep_resolution = 0.25 # m
		self.min_height = 0.5 # m
		self.max_height = 1.5 # m


def main():
	params = Params()

	# initial state = [x(m), y(m), z(m), yaw(rad), v(m/s), omega(rad/s)]
	state = np.array([0, 0.2, params.max_height, np.pi/2, 0.0, 0.0])
	traj = state[:3]
	
	plt.figure(figsize=(10,10))
	flight_area_vertices = define_flight_area(state[:2])
	# flight_area_vertices = np.array([[-1, -1], [-0.3, -1], [-0.3, -0.4], [0.3, -0.4], [0.3, -1], [1,-1], [1,1], [-1,1]])
	gridmap = GridMap(flight_area_vertices, state[:2])

	ox = flight_area_vertices[:,0].tolist() + [flight_area_vertices[0,0]]
	oy = flight_area_vertices[:,1].tolist() + [flight_area_vertices[0,1]]
	reso = params.sweep_resolution
	goal_x2D, goal_y2D = planning(ox, oy, reso)

	waypoints = get_3D_waypoints(goal_x2D, goal_y2D, params.min_height, params.max_height, params.sweep_resolution/2.)
	goal_x = waypoints[:,0]
	goal_y = waypoints[:,1]
	goal_z = waypoints[:,2]

	# goal = [x, y], m
	goali = 0
	goal = [goal_x[goali], goal_y[goali], goal_z[goali]]
	t_prev_goal = time.time()

	fig = plt.figure(figsize=(10,10))
	ax = plt.axes(projection='3d')
	ax.set_xlabel('X, [m]')
	ax.set_ylabel('Y, [m]')
	ax.set_zlabel('Z, [m]')
	ax.set_xlim([-2.5, 2.5])
	ax.set_ylim([-2.5, 2.5])
	ax.set_zlim([0.0, 3.0])
	# while True:
	for _ in tqdm( range(params.numiters) ):
		state = motion(state, goal, params)

		state = collision_avoidance(state, gridmap, params)

		goal_dist = np.linalg.norm(goal - state[:3])
		# print('Distance to goal %.2f [m]:' %goal_dist)
		t_current = time.time()
		if goal_dist < params.goal_tol or (t_current - t_prev_goal) > params.time_to_switch_goal: # goal is reached
		    # print('Switching to the next goal.')
		    # print('Time from the previous reached goal:', t_current - t_prev_goal)
		    if goali < len(goal_x) - 1:
		    	goali += 1
		    else:
		    	break
		    t_prev_goal = time.time()
		    goal = [goal_x[goali], goal_y[goali], goal_z[goali]]


		traj = np.vstack([traj, state[:3]])
		
		if params.animate:
			plt.cla()
			ax.plot(goal_x, goal_y, goal_z, ':')
			ax.scatter(goal[0], goal[1], goal[2], label='Goal position', zorder=20)
			ax.plot(traj[:,0], traj[:,1], traj[:,2], linewidth=3, color='g')
			plot_robot(ax, state, params)
			plt.legend()
			plt.pause(0.1)

	print('Mission is complete!')
	plt.cla()
	ax.plot(goal_x, goal_y, goal_z, ':')
	ax.scatter(goal[0], goal[1], goal[2], label='Goal position', zorder=20)
	ax.plot(traj[:,0], traj[:,1], traj[:,2], linewidth=3, color='g')
	plot_robot(ax, state, params)
	plt.legend()
	plt.pause(0.1)
	plt.draw()
	input('Hit Enter to close all figures')
	plt.close('all')


if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
	    pass
		
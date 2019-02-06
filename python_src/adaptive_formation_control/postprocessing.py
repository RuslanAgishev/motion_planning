#!/usr/bin/env python
from __future__ import division
import csv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import os
from sklearn import datasets, linear_model
from sklearn.metrics import mean_squared_error, r2_score
import math
from tf.transformations import *
from scipy.spatial.distance import cdist
import xlwt

plt.rcParams.update({'font.size': 18})

PATH = os.getcwd()+'/'
visualize = 1


class Data:
    def __init__(self):
    	self.name = ''
    	self.folder_to_save = '~/Desktop/'
        self.S_mean_er = 0
        self.S_std_er = 0
        self.S_max_er = 0

        self.vel_mean = 0
        self.acc_mean = 0
        self.jerk_mean = 0

        self.path_length = 0
        self.min_dist_obstacles = 0
        self.mean_dist_obstacles = 0


class Metrics:
	def __init__(self):
    	self.names = np.array([])
    	self.times = np.array([])

	def area(self, drone1,drone2,drone3, simulation=False):
		area_array = np.array([])
		if simulation:
			drone1_time = drone1.timesp; drone1_pose = drone1.sp; drone2_pose = drone2.sp; drone3_pose = drone3.sp;
		else:
			drone1_time = drone1.time; drone1_pose = drone1.pose; drone2_pose = drone2.pose; drone3_pose = drone3.pose;
		for i in range(min(len(drone1_pose),len(drone2_pose),len(drone3_pose))):
			x = np.array([drone1_pose[i][0], drone2_pose[i][0], drone3_pose[i][0]])
			y = np.array([drone1_pose[i][1], drone2_pose[i][1], drone3_pose[i][1]])
			area = 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))
			if len(area_array)==0:
				area_array = np.array([drone1_time[i][0], area])
			else:
				area_array = np.vstack((area_array, np.array([drone1_time[i][0], area]) ))
		return area_array

	def centroid_path(self, drone1,drone2,drone3, simulation=False):
		centroid_array = np.array([])
		if simulation:
			drone1_time = drone1.timesp; drone1_pose = drone1.sp; drone2_pose = drone2.sp; drone3_pose = drone3.sp;
		else:
			drone1_time = drone1.time; drone1_pose = drone1.pose; drone2_pose = drone2.pose; drone3_pose = drone3.pose;
		for i in range(min(len(drone1_pose),len(drone2_pose),len(drone3_pose))):
			x_aver = np.array([drone1_pose[i][0], drone2_pose[i][0], drone3_pose[i][0]])
			y_aver = np.array([drone1_pose[i][1], drone2_pose[i][1], drone3_pose[i][1]])
			z_aver = np.array([drone1_pose[i][2], drone2_pose[i][2], drone3_pose[i][2]])
			centr = np.array([drone1_time[i][0], np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
			if len(centroid_array)==0:
				centroid_array = centr
			else:
				centroid_array = np.vstack((centroid_array, centr ))
		return centroid_array

	def path_length(self, poses):
		length = 0
		for i in range( 1,len(poses) ):
			length += np.linalg.norm(poses[i,:]-poses[i-1,:])
		return length


class Tools:
	def __init__(self):
    	self.names = np.array([])
    	self.times = np.array([])

	def plot(self, data, style='-'):
		plt.plot(data[:,1], -data[:,0], style)


	def trajplot(self, centroid_array, drone1, drone2, drone3, title, obstacles=None, simulation=False, patterns=None):
		fig = plt.figure()
		ax = fig.gca()
		centroid_path = centroid_array[:,1:3]
		plot(centroid_path)
		plot(drone1.pose, '--')
		plot(drone2.pose, '--')
		plot(drone3.pose, '--')
		legend_list = ['centroid','drone1', 'drone2', 'drone3']
		
		if obstacles is not None:
			for obstacle in obstacles:
				circle = plt.Circle((obstacle.pose[1], -obstacle.pose[0]),0.27, color='yellow')
				ax.add_artist(circle)
				plt.plot(obstacle.pose[1], -obstacle.pose[0],'ro')
				legend_list.append(obstacle.name)
		#plt.legend(legend_list)
		plt.xlabel('Y, meters')
		plt.ylabel('X, meters')
		ax.set_aspect('equal')
		plt.grid()
		plt.title(title)


	def savedata(self, data):
		#style0 = xlwt.easyxf('font: name Times New Roman, color-index red, bold on', num_format_str='#,##0.00')
		#style1 = xlwt.easyxf(num_format_str='D-MMM-YY')

		wb = xlwt.Workbook()
		ws = wb.add_sheet(data.glove_status)

		ws.write(0, 0, 'S_mean_er'); ws.write(0, 1, data.S_mean_er)
		ws.write(1, 0, 'S_std_er'); ws.write(1, 1, data.S_std_er)
		ws.write(2, 0, 'S_max_er'); ws.write(2, 1, data.S_max_er)

		wb.save(data.folder_to_save+'output.xls')



#!/usr/bin/env 

from __future__ import division
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from tf import TransformListener

from scipy.integrate import odeint
from scipy.spatial.distance import cdist
from math import *
import math

import time
from time import sleep

import message_filters
import sys
import numpy as np

from multiprocessing import Process
import os

from crazyflie_driver.msg import FullState
from crazyflie_driver.msg import Position

import serial
import matplotlib.pyplot as plt




class Drone:
	def __init__(self, name, leader = False):
		self.name = name
		self.tf = '/vicon/'+name+'/'+name
		self.leader = leader
		self.tl = TransformListener()
		self.pose = self.position()
		self.orient = np.array([0,0,0])
		self.sp = self.position()
		self.path = Path()
		self.dist_to_drones = np.zeros(3)
		self.drone_time_array = np.ones(10)
		self.drone_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
		self.rate = rospy.Rate(100)
		self.traj = np.array([0,0,0])
		self.start = self.position()
		self.goal  = self.sp

	def position(self):
	    self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
	    position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
	    self.pose = position
	    return np.array(position)

	def orientation(self):
	    self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
	    position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
	    self.orient = get_angles(np.array(quaternion))
	    return get_angles(np.array(quaternion))

	def publish_sp(self):
	    publish_pose(self.sp, np.array([0,0,0]), self.name+"_sp")

	def publish_position(self):
	    publish_pose(self.pose, self.orient, self.name+"_pose")

	def publish_path(self, limit=1000):
		publish_path(self.path, self.sp, self.orient, self.name+"_path", limit)
		#for i in range( 1,len(self.path.poses) ):
		#	print self.name +": "+ str(self.path.poses[i].pose)

	def fly(self):
		# if self.leader:
		# 	limits = np.array([ 2, 2, 2.5 ])
		# 	np.putmask(self.sp, self.sp >= limits, limits)
		# 	np.putmask(self.sp, self.sp <= -limits, -limits)
		publish_goal_pos(self.sp, 0, "/"+self.name)

	def calculate_dist_to_drones(self, drones_poses):
		for i in range(len(drones_poses)):
			self.dist_to_drones[i] = np.linalg.norm(drones_poses[i] - self.sp)

	def omegaZ(self, drone_pose, dist_from_obstacle):
		R = dist_from_obstacle # 3D-vector
		drone_vel = self.velocity(drone_pose)
		# drone_vel_n = np.dot(drone_vel, R)/(np.linalg.norm(R)**2) * R
		# drone_vel_t = drone_vel - drone_vel_n
		drone_w = np.cross(R, drone_vel) # / ( np.linalg.norm(R)**2 )

		return drone_w[2]

	def velocity(self, drone_pose):
		for i in range(len(self.drone_time_array)-1):
			self.drone_time_array[i] = self.drone_time_array[i+1]
		self.drone_time_array[-1] = time.time()

		for i in range(len(self.drone_pose_array[0])-1):
			self.drone_pose_array[0][i] = self.drone_pose_array[0][i+1]
			self.drone_pose_array[1][i] = self.drone_pose_array[1][i+1]
			self.drone_pose_array[2][i] = self.drone_pose_array[2][i+1]
		self.drone_pose_array[0][-1] = drone_pose[0]
		self.drone_pose_array[1][-1] = drone_pose[1]
		self.drone_pose_array[2][-1] = drone_pose[2]

		vel_x = (self.drone_pose_array[0][-1]-self.drone_pose_array[0][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])
		vel_y = (self.drone_pose_array[1][-1]-self.drone_pose_array[1][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])
		vel_z = (self.drone_pose_array[2][-1]-self.drone_pose_array[2][0]) / (self.drone_time_array[-1]-self.drone_time_array[0])

		drone_vel = np.array( [vel_x, vel_y, vel_z] )

		return drone_vel

	def landing(self, sim=False):
		drone_landing_pose = self.position() if sim==False else self.sp
		while not rospy.is_shutdown():
			self.sp = drone_landing_pose
			drone_landing_pose[2] = drone_landing_pose[2]-0.007
			self.publish_sp()
			self.publish_path(limit=1000)
			if sim==False:
				self.fly()
			if self.sp[2]<-1.0:
				sleep(1)
				if sim==False:
					cf1.stop()
				print 'reached the floor, shutdown'
				# rospy.signal_shutdown('landed')
			self.rate.sleep()


class Mocap_object:
    def __init__(self, name):
        self.name = name
        self.tf = '/vicon/'+name+'/'+name
        self.tl = TransformListener()
        self.pose = np.array([0,0,0])
        self.orient = np.array([0,0,0])
        self.angles_to_drones = None

    def position(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.pose = position
        return np.array(position)

    def orientation(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.orient = get_angles(np.array(quaternion))
        return get_angles(np.array(quaternion))

    def publish_position(self):
        publish_pose(self.pose, self.orient, self.name+"_pose")


class Obstacle:
    def __init__(self, name, R_obstacle):
        self.name = name
        self.R = R_obstacle
        self.tf = '/vicon/'+name+'/'+name
        self.tl = TransformListener()
        self.pose = self.position()
        self.orient = np.array([0,0,0])
        self.dist_to_drones = np.zeros(3)

    def position(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.pose = position
        return np.array(position)

    def orientation(self):
        self.tl.waitForTransform("/world", self.tf, rospy.Time(0), rospy.Duration(1))
        position, quaternion = self.tl.lookupTransform("/world", self.tf, rospy.Time(0))
        self.orient = get_angles(np.array(quaternion))
        return get_angles(np.array(quaternion))

    def publish_position(self):
        # publish_pose(self.pose, self.orient, self.name+"_pose")
        publish_cylinder(self.pose, self.orient, self.R, self.name+"_cylinder")

    def calculate_dist(self, drones_poses):
    	for i in range(len(drones_poses)):
        	self.dist_to_drones[i] = np.linalg.norm(drones_poses[i]-self.pose)

    def circle_points(self, R, N=1000):
    	C = np.zeros((N,2))
    	# C[0,:] = self.position()[0] + self.R*np.cos(np.linspace(-pi,pi,N))
    	# C[1,:] = self.position()[1] + self.R*np.sin(np.linspace(-pi,pi,N))
    	C[:,0] = self.pose[0] + R*np.cos(np.linspace(-pi,pi,N))
    	C[:,1] = self.pose[1] + R*np.sin(np.linspace(-pi,pi,N))
    	return C


def msg_def_crazyflie(pose, yaw):
	worldFrame = rospy.get_param("~worldFrame", "/world")
	msg = Position()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.x = pose[0]
	msg.y = pose[1]
	msg.z = pose[2]
	msg.yaw = yaw
	now = rospy.get_time()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	return msg

def msg_def_PoseStamped(pose, orient):
	worldFrame = "world"
	msg = PoseStamped()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.header.seq += 1
	msg.header.stamp = rospy.Time.now()
	return msg

def msg_def_Cylinder(pose, orient, shape, R):
	worldFrame = "world"
	msg = Marker()
	msg.header.seq = 0
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = worldFrame
	msg.type = shape
	msg.pose.position.x = pose[0]
	msg.pose.position.y = pose[1]
	msg.pose.position.z = pose[2] * 0.5
	# quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2])
	quaternion = tf.transformations.quaternion_from_euler(0,0,0)
	msg.pose.orientation.x = quaternion[0]
	msg.pose.orientation.y = quaternion[1]
	msg.pose.orientation.z = quaternion[2]
	msg.pose.orientation.w = quaternion[3]
	msg.scale.x = R
	msg.scale.y = R
	msg.scale.z = 2.0
	msg.color.r = 0.0
	msg.color.g = 1.0
	msg.color.b = 0.0
	msg.color.a = 1.0
	msg.header.seq += 1
	msg.header.stamp = rospy.Time.now()
	return msg

def msg_def_Arrow(pose, orient, length):
	arrow = Marker()
	arrow.header.frame_id = "world"
	arrow.header.stamp = rospy.Time.now()
	arrow.type = arrow.ARROW
	arrow.pose.position.x = pose[0]
	arrow.pose.position.y = pose[1]
	arrow.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
	arrow.pose.orientation.x = quaternion[0]
	arrow.pose.orientation.y = quaternion[1]
	arrow.pose.orientation.z = quaternion[2]
	arrow.pose.orientation.w = quaternion[3]
	arrow.scale.x = length
	arrow.scale.y = 0.02
	arrow.scale.z = 0.02
	arrow.color.a = 1.0
	arrow.color.r = 1.0
	arrow.color.g = 0.5 
	arrow.color.b = 0.5
	arrow.header.seq += 1
	arrow.header.stamp = rospy.Time.now()
	return arrow

def publish_goal_pos(cf_goal_pos, cf_goal_yaw, cf_name):
	name = cf_name + "/cmd_position"
	msg = msg_def_crazyflie(cf_goal_pos, cf_goal_yaw)
	pub = rospy.Publisher(name, Position, queue_size=1)
	pub.publish(msg)

def get_coord(PoseStamped_message):
	x = PoseStamped_message.transform.translation.x
	y = PoseStamped_message.transform.translation.y
	z = PoseStamped_message.transform.translation.z
	coord_array = np.array([x, y, z])
	return coord_array

def get_angles(message):
	quat = ( message[0], message[1], message[2], message[3] )
	euler = tf.transformations.euler_from_quaternion(quat)
	return euler

def publish_pose(pose, orient, topic_name):
	msg = msg_def_PoseStamped(pose, orient)
	pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	pub.publish(msg)

def publish_path(path, pose, orient, topic_name, limit=1000):
	msg = msg_def_PoseStamped(pose, orient)
	path.header = msg.header
	path.poses.append(msg)
	if limit>0:
		path.poses = path.poses[-limit:]
	pub = rospy.Publisher(topic_name, Path, queue_size=1)
	pub.publish(path)

def publish_cylinder(pose, orient, R, topic_name):
	shape = Marker.CYLINDER
	msg = msg_def_Cylinder(pose, orient, shape, R=R)
	pub = rospy.Publisher(topic_name, Marker, queue_size=1)
	pub.publish(msg)

def publish_arrow(pose, orient, length, topic_name):
	shape = Marker.ARROW
	msg = msg_def_Arrow(pose, orient, length)
	pub = rospy.Publisher(topic_name, Marker, queue_size=1)
	pub.publish(msg)


# HUMAN VELOCITY CALCULATION
hum_time_array = np.ones(10)
hum_pose_array = np.array([ np.ones(10), np.ones(10), np.ones(10) ])
def hum_vel(human_pose):

	for i in range(len(hum_time_array)-1):
		hum_time_array[i] = hum_time_array[i+1]
	hum_time_array[-1] = time.time()

	for i in range(len(hum_pose_array[0])-1):
		hum_pose_array[0][i] = hum_pose_array[0][i+1]
		hum_pose_array[1][i] = hum_pose_array[1][i+1]
		hum_pose_array[2][i] = hum_pose_array[2][i+1]
	hum_pose_array[0][-1] = human_pose[0]
	hum_pose_array[1][-1] = human_pose[1]
	hum_pose_array[2][-1] = human_pose[2]

	vel_x = (hum_pose_array[0][-1]-hum_pose_array[0][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_y = (hum_pose_array[1][-1]-hum_pose_array[1][0])/(hum_time_array[-1]-hum_time_array[0])
	vel_z = (hum_pose_array[2][-1]-hum_pose_array[2][0])/(hum_time_array[-1]-hum_time_array[0])

	hum_vel = np.array( [vel_x, vel_y, vel_z] )

	return hum_vel

def pub_circle_traj(x0,y0,z0,r,i):
	# i=0
	# while time_delay<delay:
		
	x1 = x0 + r*sin(i*1.75*pi/360) # 1
	y1 = y0 + r*cos(i*1.75*pi/360) # 1
	z1 = z0

	drone10_pose_goal = np.array([ x1,y1,z1 ])

	x2 = x0 + r*sin(i*1.75*pi/360+pi) # 2
	y2 = y0 + r*cos(i*1.75*pi/360+pi) # 2
	z2 = z0
	
	drone11_pose_goal = np.array([ x2,y2,z2 ])

	i = i+1
	
	# publish_goal_pos(drone10_pose_goal, 0, "/crazyflie10")
	# publish_goal_pos(drone11_pose_goal, 0, "/crazyflie11")

	publish_pose(drone10_pose_goal, 0, "drone10_pose_goal")
	publish_pose(drone11_pose_goal, 0, "drone11_pose_goal")

	return i, drone10_pose_goal, drone11_pose_goal

def rotate(origin, drone, human):
	"""
	Rotate a point counterclockwise by a given angle around a given origin.
	The angle should be given in radians.
	"""
	ox, oy = origin[0], origin[1]
	px, py = drone.sp[0], drone.sp[1]

	qx = ox + math.cos(human.orientation()[2]) * (px - ox) - math.sin(human.orientation()[2]) * (py - oy)
	qy = oy + math.sin(human.orientation()[2]) * (px - ox) + math.cos(human.orientation()[2]) * (py - oy)
	
	return np.array([qx, qy, drone.sp[2]])

def centroid_calc(drone1, drone2, drone3):
	x_aver = np.array([drone1.sp[0], drone2.sp[0], drone3.sp[0]])
	y_aver = np.array([drone1.sp[1], drone2.sp[1], drone3.sp[1]])
	z_aver = np.array([drone1.sp[2], drone2.sp[2], drone3.sp[2]])
	centroid = np.array([ np.mean(x_aver), np.mean(y_aver), np.mean(z_aver) ])
	return centroid





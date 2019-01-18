#!/usr/bin/env python

# python3 GradientBasedPlanner.py

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.morphology import distance_transform_edt as bwdist

""" ROS """
import time
import rospy
from geometry_msgs.msg import TransformStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty
import crazyflie


""" initialization """
visualize     = 1
toFly         = 1
direction     = 'forward'
TakeoffTime   = 3.0 # [s]
TakeoffHeight = 1.0 # [m]

cf_names         = np.array(['cf1',
                             'cf2',
                             'cf3'
                             ])


def GradientBasedPlanner (f, start_coords, end_coords, max_its):
    """
    GradientBasedPlanner : This function plans a path through a 2D
    environment from a start to a destination based on the gradient of the
    function f which is passed in as a 2D array. The two arguments
    start_coords and end_coords denote the coordinates of the start and end
    positions respectively in the array while max_its indicates an upper
    bound on the number of iterations that the system can use before giving
    up.
    The output, route, is an array with 2 columns and n rows where the rows
    correspond to the coordinates of the robot as it moves along the route.
    The first column corresponds to the x coordinate and the second to the y coordinate
	"""
    [gy, gx] = np.gradient(-f);

    route = np.vstack( [np.array(start_coords), np.array(start_coords)] )
    for i in range(max_its):
        current_point = route[-1,:];
        to_goal = sum( abs(current_point-end_coords) )
        # print(to_goal)
        if to_goal < 10.0: # cm
            print('Reached the goal !')
            print('Distance to goal [cm]: '+str(round(to_goal,2)))
            route = route[1:,:]
            return route
        ix = int(round( current_point[1] ));
        iy = int(round( current_point[0] ));
        vx = gx[ix, iy]
        vy = gy[ix, iy]
        dt = 1 / np.linalg.norm([vx, vy]);
        next_point = current_point + dt*np.array( [vx, vy] );
        route = np.vstack( [route, next_point] );
    route = route[1:,:]
    print('The goal is not reached after '+str(max_its)+' iterations...')
    print('Distance to goal [cm]: '+str(round(to_goal,2)))
    return route

def CombinedPotential(obstacle, goal):
	""" Repulsive potential """
	d = bwdist(obstacle==0);
	d2 = (d/100.) + 1; # Rescale and transform distances
	d0 = 2;
	nu = 200; # repulsive strngth
	repulsive = nu*((1./d2 - 1/d0)**2);
	repulsive [d2 > d0] = 0;

	""" Attractive potential """
	xi = 1/200. # attractive strength
	attractive = xi * ( (x - goal[0])**2 + (y - goal[1])**2 );

	""" Combine terms """
	f = attractive + repulsive;
	return f


def meters2grid(pose_m, nrows=500, ncols=500):
	# [0, 0](m) -> [250, 250]
	# [1, 0](m) -> [250+100, 250]
	# [0,-1](m) -> [250, 250-100]
	pose_on_grid = np.array(pose_m)*100 + np.array([ncols/2, nrows/2])
	return np.array( pose_on_grid, dtype=int)

def grid2meters(pose_grid, nrows=500, ncols=500):
	# [250, 250] -> [0, 0](m)
	# [250+100, 250] -> [1, 0](m)
	# [250, 250-100] -> [0,-1](m)
	pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100
	return pose_meters


""" Generate obstacles map """
nrows = 500;
ncols = 500;
obstacle = np.zeros((nrows, ncols));
[x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))

start = meters2grid([-1.5, 0.5]); goal = meters2grid([1.5,-1.5]);

# rectangular obstacles
# obstacle [300:, 100:250] = True;
# obstacle [150:200, 400:500] = True;

def pos_callback(data):
    global cf_pose
    cf_pose = data

def callback0(data):
    global pose0
    pose0 = [data.transform.translation.x, data.transform.translation.y]

def callback3(data):
    global pose3
    pose3 = [data.transform.translation.x, data.transform.translation.y]

def callback4(data):
    global pose4
    pose4 = [data.transform.translation.x, data.transform.translation.y]

rospy.init_node('motion_planning', anonymous=True)
rospy.Subscriber("/vicon/obstacle0/obstacle0", TransformStamped, callback0)
rospy.Subscriber("/vicon/obstacle3/obstacle3", TransformStamped, callback3)
rospy.Subscriber("/vicon/obstacle4/obstacle4", TransformStamped, callback4)


time.sleep(1) # wait for ROS-Vicon driver to start recieving data

# obstacles_poses = [[-2, 1], [1.5, 0.5], [0, 0], [-1.8, -1.8]] # 2D - coordinates [m]
obstacles_poses = [pose0, pose3, pose4] # 2D - coordinates [m]
R_obstacles = 0.2; # [m]

for pose in obstacles_poses:
	pose = meters2grid(pose)
	x0 = pose[0]; y0 = pose[1]
	# cylindrical obstacles
	t = ((x - x0)**2 + (y - y0)**2) < (100*R_obstacles)**2
	obstacle[t] = True;


""" Plan route """
f = CombinedPotential(obstacle, goal)
route = GradientBasedPlanner(f, start, goal, max_its=1000);
route = grid2meters(route)

""" Visualization """
if visualize:
    [gy, gx] = np.gradient(-f);

    # Velocities map and a Path from start to goal
    skip = 10;
    [x_m, y_m] = np.meshgrid(np.linspace(-2.5, 2.5, ncols), np.linspace(-2.5, 2.5, nrows))
    start_m = grid2meters(start); goal_m = grid2meters(goal)
    plt.figure(figsize=(nrows/50, ncols/50))
    ax = plt.gca()
    Q = plt.quiver(x_m[::skip, ::skip], y_m[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip])
    plt.plot(start_m[0], start_m[1], 'ro', markersize=10);
    plt.plot(goal_m[0], goal_m[1], 'ro', color='green', markersize=10);
    plt.plot(route[:,0], route[:,1], linewidth=3);
    plt.xlabel('X')
    plt.ylabel('Y')
    # obstacles
    for pose in obstacles_poses:
        circle = plt.Circle(pose, R_obstacles, color='yellow')
        ax.add_artist(circle)

    plt.show()

# TODO: use computed Path (route variable) to guide real drone
# (takeoff)
# (trajectory following)
# (landing)

if toFly:
    print("takeoff")
    cf1 = crazyflie.Crazyflie(cf_names[0], '/vicon/'+cf_names[0]+'/'+cf_names[0])
    cf1.setParam("commander/enHighLevel", 1)
    cf1.setParam("stabilizer/estimator", 2) # Use EKF
    cf1.setParam("stabilizer/controller", 2) # Use Mellinger controller
    cf1.takeoff(targetHeight = TakeoffHeight, duration = TakeoffTime)
    time.sleep(TakeoffTime+1)

    # cf1.land(targetHeight = 0.0, duration = 2.0)
    # time.sleep(3.0)

    rate = rospy.Rate(40)

    msg = Position()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/world"

    cfname = cf_names[0]
    pub = rospy.Publisher(cfname+"/cmd_position", Position, queue_size=1)
    stop_pub = rospy.Publisher(cfname+"/cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()
    pos_sub = rospy.Subscriber('/vicon/'+cf_names[0]+'/'+cf_names[0], TransformStamped, pos_callback)

    start_time = rospy.Time.now()
    traj = route

    if direction=='inverse':
        RANGE = reversed(range(len(traj)))
    else:
        RANGE = range(len(traj))
    for i in RANGE:
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - start_time).to_sec()
        
        msg.x    = traj[i,0]
        msg.y    = traj[i,1]
        msg.z    = TakeoffHeight
        msg.yaw  = 0

        pub.publish(msg)
        print(msg)
        rate.sleep()

    rospy.loginfo('Landing...')
    while msg.z > -1.0:
        msg.z -= 0.02
        # if ( cf_pos.transform.translation.z < 0.11 ):
        #     break
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
    stop_pub.publish(stop_msg)



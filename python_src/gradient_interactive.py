# In order to launch execute:
# python3 gradient_interactive.py

import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import collections
from scipy.ndimage.morphology import distance_transform_edt as bwdist
from math import *
import random
from impedance import *
import time

from progress.bar import FillingCirclesBar
from tasks import *
from threading import Thread

""" ROS """
import rospy
from geometry_msgs.msg import TransformStamped


def draw_map(obstacles_poses, f=None, draw_gradients=True, nrows=500, ncols=500):
    if draw_gradients and f is not None:
        skip = 10
        [x_m, y_m] = np.meshgrid(np.linspace(-2.5, 2.5, ncols), np.linspace(-2.5, 2.5, nrows))
        [gy, gx] = np.gradient(-f);
        Q = plt.quiver(x_m[::skip, ::skip], y_m[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip])
    else:
        plt.grid()
    plt.plot(start[0], start[1], 'ro', color='yellow', markersize=10);
    plt.plot(goal[0], goal[1], 'ro', color='green', markersize=10);
    plt.xlabel('X')
    plt.ylabel('Y')
    ax = plt.gca()
    for pose in obstacles_poses:
        circle = plt.Circle(pose, R_obstacles, color='red')
        ax.add_artist(circle)
    # Create a Rectangle patch
    rect1 = patches.Rectangle((-2.5,-1.15),2.0,0.2,linewidth=1,color='red',fill='True')
    rect2 = patches.Rectangle((-1.2, 1.4), 0.2,1.0,linewidth=1,color='red',fill='True')
    rect3 = patches.Rectangle(( 0.4, 0.8), 2.0,0.5,linewidth=1,color='red',fill='True')
    ax.add_patch(rect1)
    ax.add_patch(rect2)
    ax.add_patch(rect3)
    
def draw_robots(current_point1, routes=None, num_robots=None, robots_poses=None, centroid=None, vel1=None):
    if vel1 is not None: plt.arrow(current_point1[0], current_point1[1], vel1[0], vel1[1], width=0.01, head_width=0.05, head_length=0.1, fc='k')
    plt.plot(routes[0][:,0], routes[0][:,1], 'green', linewidth=2)
    for r in range(1,num_robots):
        plt.plot(routes[r][:,0], routes[r][:,1], '--', color='blue', linewidth=2)

    for pose in robots_poses:
    	plt.plot(pose[0], pose[1], 'ro', color='blue')
    # compute centroid and sort poses by polar angle
    pp = robots_poses
    pp.sort(key=lambda p: atan2(p[1]-centroid[1],p[0]-centroid[0]))
    formation = patches.Polygon(pp, color='blue', fill=False, linewidth=2);
    plt.gca().add_patch(formation)
    plt.plot(centroid[0], centroid[1], '*', color='blue')

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
    pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100.0
    return pose_meters

def gradient_planner(f, current_point, ncols=500, nrows=500):
    """
    GradientBasedPlanner : This function computes the next_point
    given current location, goal location and potential map, f.
    It also returns mean velocity, V, of the gradient map in current point.
    """
    [gy, gx] = np.gradient(-f);
    iy, ix = np.array( meters2grid(current_point), dtype=int )
    w = 40 # smoothing window size for gradient-velocity
    vx = np.mean(gx[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
    vy = np.mean(gy[ix-int(w/2) : ix+int(w/2), iy-int(w/2) : iy+int(w/2)])
    V = np.array([vx, vy])
    dt = 0.1 / norm(V);
    next_point = current_point + dt*V;

    return next_point, V

def combined_potential(obstacles_poses, goal, nrows=500, ncols=500):
    """ Repulsive potential """
    obstacles_map = map(obstacles_poses)
    goal = meters2grid(goal)
    d = bwdist(obstacles_map==0);
    d2 = (d/100.) + 1; # Rescale and transform distances
    d0 = 2;
    nu = 300;
    repulsive = nu*((1./d2 - 1/d0)**2);
    repulsive [d2 > d0] = 0;
    """ Attractive potential """
    [x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))
    xi = 1/700.;
    attractive = xi * ( (x - goal[0])**2 + (y - goal[1])**2 );
    """ Combine terms """
    f = attractive + repulsive;
    return f

def map(obstacles_poses, nrows=500, ncols=500):
    """ Obstacles map """
    obstacles_map = np.zeros((nrows, ncols));
    [x, y] = np.meshgrid(np.arange(ncols), np.arange(nrows))
    for pose in obstacles_poses:
        pose = meters2grid(pose)
        x0 = pose[0]; y0 = pose[1]
        # cylindrical obstacles
        t = ((x - x0)**2 + (y - y0)**2) < (100*R_obstacles)**2
        obstacles_map[t] = 1;
    # rectangular obstacles
    obstacles_map[400:, 130:150] = 1;
    obstacles_map[130:150, :200] = 1;
    obstacles_map[330:380, 300:] = 1;
    return obstacles_map

def move_obstacles(obstacles_poses):
    # dx = 0.01;                   dy = 0.01
    # obstacles_poses[0][0] += dx; obstacles_poses[0][1] -= dy
    # obstacles_poses[1][0] -= dx; obstacles_poses[1][1] -= dy
    # obstacles_poses[2][0] -= dx; obstacles_poses[2][1] -= dy
    # obstacles_poses[3][0] += dx; obstacles_poses[3][1] += dy
    """ obstacles tend to go to the origin, (0,0) - point """
    for pose in obstacles_poses:
    	dx = random.uniform(0, 0.03); dy = random.uniform(0,0.03);
    	pose[0] -= np.sign(pose[0])*dx;      pose[1] -= np.sign(pose[1])*dy;

    return obstacles_poses


def formation(num_robots, leader_des, v, R_swarm):
    if num_robots<=1: return []
    u = np.array([-v[1], v[0]])
    des4 = leader_des - v*R_swarm*sqrt(3)                 # follower
    if num_robots==2: return [des4]
    des2 = leader_des - v*R_swarm*sqrt(3)/2 + u*R_swarm/2 # follower
    des3 = leader_des - v*R_swarm*sqrt(3)/2 - u*R_swarm/2 # follower
    if num_robots==3: return [des2, des3]
    
    return [des2, des3, des4]

""" initialization """
animate              = 1   # show 1-each frame or 0-just final configuration
random_obstacles     = 1   # randomly distributed obstacles on the map
num_random_obstacles = 8   # number of random circular obstacles on the map
num_robots           = 4   # number of drones in formation
moving_obstacles     = 1   # 0-static or 1-dynamic obstacles
impedance            = 1   # impedance links between the leader and followers (leader's velocity)
formation_gradient   = 1   # followers are attracting to their formation position and repelling from obstacles
draw_gradients       = 1   # 1-gradients plot, 0-grid
""" human guided swarm params """
interactive          = 1      # 1-human guided swarm, 0-potential fields as a planner to goal pose
human_name           = 'palm' # vicon mocap object
pos_coef             = 3.0    # scale of the leader's movement relatively to the human operator
initialized          = False  # is always inits with False: for relative position control
max_its              = 1000 if interactive else 300 # max number of allowed iters for formation to reach the goal


progress_bar = FillingCirclesBar('Number of Iterations', max=max_its)
should_write_movie = 0; movie_file_name = 'videos/output.avi'
movie_writer = get_movie_writer(should_write_movie, 'Simulation Potential Fields', movie_fps=10., plot_pause_len=0.01)

R_obstacles = 0.1 # [m]
R_swarm     = 0.3 # [m]
start = np.array([-1.8, 1.8]); goal = np.array([1.7, -1.7])
V0 = (goal - start) / norm(goal-start)   # initial movement direction, |V0| = 1
U0 = np.array([-V0[1], V0[0]]) / norm(V0) # perpendicular to initial movement direction, |U0|=1
imp_pose_prev = np.array( [0,0] )
imp_vel_prev  = np.array( [0,0] )
imp_time_prev = time.time()

if random_obstacles:
    obstacles_poses = np.random.uniform(low=-2.5, high=2.5, size=(num_random_obstacles,2)) # randomly located obstacles 
else:
    obstacles_poses = np.array([[-2, 1], [1.5, 0.5], [-1.0, 1.5], [0, 0], [1, -2], [-1.8, -1.8]]) # 2D - coordinates [m]


def human_pos_callback(data):
    global human_pose
    global human_yaw
    human_pose = np.array( [data.transform.translation.x, data.transform.translation.y, data.transform.translation.z] )
    # human_yaw  = np.array( [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w] )

rospy.init_node('gradient_interactive', anonymous=True)
pos_sub = rospy.Subscriber('/vicon/' + human_name + '/' + human_name, TransformStamped, human_pos_callback)
time.sleep(1)


""" Main loop """

# drones polygonal formation
route1 = start # leader
current_point1 = start
robots_poses = [start] + formation(num_robots, start, V0, R_swarm)
routes = [route1] + robots_poses[1:]
centroid_route = [ sum([p[0] for p in robots_poses])/len(robots_poses), sum([p[1] for p in robots_poses])/len(robots_poses) ]
des_poses = robots_poses

fig = plt.figure(figsize=(10, 10))
with movie_writer.saving(fig, movie_file_name, max_its) if should_write_movie else get_dummy_context_mgr():
    for i in range(max_its):
        # TODO: make random start and goal points for obstales, not (0,0) for all
        if moving_obstacles: obstacles_poses = move_obstacles(obstacles_poses)

        """ Leader's pose update """
        if interactive:
            f1 = combined_potential(obstacles_poses, current_point1)
            # human palm pose and velocity using Vicon motion capture
            if not initialized:
                human_pose_init = human_pose[:2]
                drone1_pose_init = start
                initialized = True
            dx, dy = human_pose[:2] - human_pose_init
            des_poses[0] = np.array([  drone1_pose_init[0] + pos_coef*dx, drone1_pose_init[1] + pos_coef*dy ])
            vel1 = hum_vel(human_pose)
            # TODO: implemete rotation of the swarm relatively to human orientation: change direction variable
            # for instance: direction=[cos(hum_yaw), sin(hum_yaw)]
            # direction = np.array( des_poses[0] - human_pose_init ) / norm(des_poses[0] - human_pose_init)
        else:
            f1 = combined_potential(obstacles_poses, goal)
            des_poses[0], vel1 = gradient_planner(f1, current_point1)            

        # drones polygonal formation
        direction = ( goal - des_poses[0] ) / norm(goal - des_poses[0])
        des_poses[1:] = formation(num_robots, des_poses[0], direction, R_swarm)
        v = direction; u = np.array([-v[1], v[0]])

        if impedance:
            # drones positions are corrected according to the impedance model
            # based on leader's velocity
            imp_pose, imp_vel, imp_time_prev = velocity_imp(vel1, imp_pose_prev, imp_vel_prev, imp_time_prev)
            imp_pose_prev = imp_pose
            imp_vel_prev = imp_vel

            imp_scale = 1.0 if interactive else 0.05
            des_poses[0] += imp_scale * imp_pose
            if num_robots==2:
                des_poses[1] += imp_scale * imp_pose @ u/norm(u) *u/norm(u) # impedance correction term is projected in u-vector direction
            if num_robots==3:
                des_poses[2] += imp_scale * imp_pose @ u/norm(u) *u/norm(u) # u-vector direction
            if num_robots==4:
                des_poses[3] -= imp_scale * imp_pose @ v/norm(v) *v/norm(v) # v-vector direction

        if formation_gradient:
            # following drones are attracting to desired points - vertices of the polygonal formation
            for p in range(1,num_robots):
                f = combined_potential(obstacles_poses, des_poses[p])
                des_poses[p], _ = gradient_planner(f, des_poses[p])

        for r in range(num_robots):
            routes[r] = np.vstack([routes[r], des_poses[r]])

        current_point1 = des_poses[0] # update current point of the leader

        pp = des_poses
        centroid = [ sum([p[0] for p in pp])/len(pp), sum([p[1] for p in pp])/len(pp) ]
        centroid_route = np.vstack([centroid_route, centroid])
        dist_to_goal = norm(centroid - goal)
        if dist_to_goal < 1.2*R_swarm:
            print('\nReached the goal')
            break

        progress_bar.next()
        plt.cla()

        draw_map(obstacles_poses, f1, draw_gradients=not interactive)
        draw_robots(current_point1, routes, num_robots, robots_poses, centroid)
        if animate:
            plt.draw()
            plt.pause(0.01)

        if should_write_movie:
            movie_writer.grab_frame()

    print('\nDone')
    progress_bar.finish()
    plt.show()

""" postprocessing """
# plt.figure()
# plt.title("Centroid's trajectory")
# plt.plot(centroid_route[:,0], centroid_route[:,1])
# plt.grid()
# plt.show()

# TODO:
# local minimum problem (FM2 - algorithm: https://pythonhosted.org/scikit-fmm/)
# impedance controlled shape of the formation: area(velocity)


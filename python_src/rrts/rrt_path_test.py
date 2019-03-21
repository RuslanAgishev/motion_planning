{
 "cells": [
  {
   "cell_type": "code",
   "execution_count": 5,
   "metadata": {},
   "outputs": [],
   "source": [
    "import numpy as np\n",
    "from numpy.linalg import norm\n",
    "from matplotlib import pyplot as plt\n",
    "from random import random"
   ]
  },
  {
   "cell_type": "markdown",
   "metadata": {},
   "source": [
    "Helper functions"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 4,
   "metadata": {},
   "outputs": [],
   "source": [
    "def meters2grid(pose_m, nrows=1000, ncols=1000):\n",
    "    # [0, 0](m) -> [250, 250]\n",
    "    # [1, 0](m) -> [250+100, 250]\n",
    "    # [0,-1](m) -> [250, 250-100]\n",
    "    pose_on_grid = np.array(pose_m)*100 + np.array([ncols/2, nrows/2])\n",
    "    return np.array( pose_on_grid, dtype=int)\n",
    "\n",
    "def grid2meters(pose_grid, nrows=1000, ncols=1000):\n",
    "    # [250, 250] -> [0, 0](m)\n",
    "    # [250+100, 250] -> [1, 0](m)\n",
    "    # [250, 250-100] -> [0,-1](m)\n",
    "    pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) / 100.0\n",
    "    return pose_meters\n",
    "\n",
    "def normalize(vector):\n",
    "    if np.linalg.norm(vector)==0: return np.zeros_like(vector)\n",
    "    return vector / np.linalg.norm(vector)\n",
    "\n",
    "def draw_map(obstacles_poses, R_obstacles, nrows=1000, ncols=1000):\n",
    "    plt.grid()\n",
    "    plt.xlabel('X')\n",
    "    plt.ylabel('Y')\n",
    "    ax = plt.gca()\n",
    "    for pose in obstacles_poses:\n",
    "        circle = plt.Circle(pose, R_obstacles, color='black')\n",
    "        ax.add_artist(circle)\n",
    "\n",
    "def draw_robot(robot):\n",
    "    plt.plot(robot.pose[0], robot.pose[1], 'ro', color='blue')"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 17,
   "metadata": {},
   "outputs": [
    {
     "data": {
      "text/plain": [
       "<matplotlib.legend.Legend at 0x7fb5ffd56358>"
      ]
     },
     "execution_count": 17,
     "metadata": {},
     "output_type": "execute_result"
    },
    {
     "data": {
      "image/png": "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\n",
      "text/plain": [
       "<Figure size 720x720 with 1 Axes>"
      ]
     },
     "metadata": {
      "needs_background": "light"
     },
     "output_type": "display_data"
    }
   ],
   "source": [
    "obstacles_poses = [[0,0], [1,1]]\n",
    "R_obstacles = 0.2\n",
    "start = [2, 0]\n",
    "goal = [0, 2]\n",
    "\n",
    "\n",
    "plt.figure(figsize=(10,10))\n",
    "draw_map(obstacles_poses, R_obstacles)\n",
    "plt.plot(start[0],start[1], 'ro', color='red', markersize=20, label='start')\n",
    "plt.plot(goal[0],goal[1], 'ro', color='green', markersize=20, label='goal')\n",
    "plt.legend()"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": null,
   "metadata": {},
   "outputs": [],
   "source": []
  },
  {
   "cell_type": "code",
   "execution_count": 37,
   "metadata": {},
   "outputs": [],
   "source": [
    "class Params:\n",
    "    def __init__(self):\n",
    "        self.grid_resolution = 0.5\n",
    "        self.threshold = 5\n",
    "        self.maxiters = 1000\n",
    "        self.smoothiters = 150"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 31,
   "metadata": {},
   "outputs": [],
   "source": [
    "class Robot:\n",
    "    def __init__(self):\n",
    "        self.p = [0, 0]\n",
    "        self.ballradius = 0.5"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 66,
   "metadata": {},
   "outputs": [],
   "source": [
    "class Node:\n",
    "    def __init__(self):\n",
    "        self.p     = [0, 0]\n",
    "        self.iPrev = 0"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 81,
   "metadata": {},
   "outputs": [],
   "source": [
    "# Helper functions\n",
    "from matplotlib import path\n",
    "\n",
    "def isCollisionFreeVertex(robot, obstacles):\n",
    "    # the function calculates value:\n",
    "    # collFree = [xy-point is outside obstacles map]\n",
    "    collFree = 1\n",
    "    xy = robot.p\n",
    "    for obstacle in obstacles:\n",
    "        hull = path.Path(obstacle)\n",
    "        collFree = not hull.contains_points([xy])\n",
    "        if hull.contains_points([xy]):\n",
    "            print('collision Vertex')\n",
    "            return 0\n",
    "\n",
    "    return collFree"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 82,
   "metadata": {},
   "outputs": [],
   "source": [
    "# def isCollisionFreeEdge(obstacles, closest_vert, xy):\n",
    "def isCollisionFreeEdge(robot, obstacles, p1, p2, map_resolution=0.5):\n",
    "    p1 = np.array(p1); p2 = np.array(p2)\n",
    "    collFree = 1\n",
    "    l = norm(p1 - p2)\n",
    "    M = int(l / map_resolution)\n",
    "    if M <= 2: M = 3\n",
    "    t = np.linspace(0,1,M)\n",
    "    for i in range(1,M-1):\n",
    "        p = (1-t[i])*p1 + t[i]*p2 # calculate configuration\n",
    "        robot.p = p\n",
    "        collFree = isCollisionFreeVertex(robot, obstacles) \n",
    "        if collFree == 0:\n",
    "            print('collision Edge')\n",
    "            return 0\n",
    "\n",
    "    return collFree"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 51,
   "metadata": {},
   "outputs": [
    {
     "data": {
      "text/plain": [
       "False"
      ]
     },
     "execution_count": 51,
     "metadata": {},
     "output_type": "execute_result"
    }
   ],
   "source": [
    "r.p = [0.5, 2.0]\n",
    "isCollisionFreeVertex(r, obstacles)"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 3,
   "metadata": {},
   "outputs": [],
   "source": [
    "def AddNode(rrt, p, iPrev):\n",
    "    node.p = p\n",
    "    node.iPrev = iPrev\n",
    "    rrt.append( node )\n",
    "    return rrt"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 133,
   "metadata": {},
   "outputs": [],
   "source": [
    "def PlanPathRRT(robot, obstacles, param, p_start, p_goal):\n",
    "    global iterations\n",
    "    P = []\n",
    "    rrt = []\n",
    "    rrt = AddNode(rrt, p_start, 0)\n",
    "    iteration = 1\n",
    "    while iteration <= param.maxiters:\n",
    "        if iteration % 50 == 0:\n",
    "            blah = 0\n",
    "\n",
    "        p = np.random.rand(2)\n",
    "        p[0] = p[0]*5-2.5\n",
    "        p[1] = p[1]*5-2.5\n",
    "        robot.p = p\n",
    "        collision = isCollisionFreeVertex(robot, obstacles)\n",
    "        if collision == 0: # skip to next iteration\n",
    "            iteration += 1\n",
    "            continue\n",
    "        \n",
    "        # do something if valid coordinate\n",
    "        for i in range(len(rrt)):\n",
    "            dist = norm(rrt[i].p - p)\n",
    "            if (i==0) or (dist < mindist):\n",
    "                mindist = dist\n",
    "                imin = i\n",
    "                l = rrt[i].p\n",
    "\n",
    "        collision = isCollisionFreeEdge(robot, obstacles, p, l, param.grid_resolution) # check for valid edge\n",
    "        if collision == 0: # skip to next iteration if not valid edge\n",
    "            iteration = iteration + 1\n",
    "            continue\n",
    "        rrt = AddNode(rrt, p, imin) # add p to T with parent l\n",
    "        dist = norm(p-p_goal)\n",
    "\n",
    "        \n",
    "        plt.plot(rrt[iteration].p[0], rrt[iteration].p[1], '.', color='k')\n",
    "        if (dist < param.threshold):\n",
    "#             print('near the goal')\n",
    "            collision = isCollisionFreeEdge(robot, obstacles, p, p_goal, param.grid_resolution) # check for valid edge\n",
    "            if collision == 1: # skip to next iteration if not valid edge\n",
    "                iteration = iteration + 1\n",
    "                continue \n",
    "                \n",
    "            iterations = iteration\n",
    "            # add qgoal to T with parent q and exit with success\n",
    "            rrt = AddNode(rrt, p_goal, len(rrt))\n",
    "            # construct P here:\n",
    "            i = len(rrt)\n",
    "            P = np.array( rrt[i].p )\n",
    "            while 1:\n",
    "                i = rrt[i].iPrev\n",
    "                print(i)\n",
    "                if i == 0:\n",
    "                    return\n",
    "    \n",
    "                P = np.append([P, rrt[i].p])\n",
    "\n",
    "\n",
    "        iteration = iteration + 1\n",
    "    \n",
    "    iterations = iteration - 1\n",
    "\n",
    "    return P"
   ]
  },
  {
   "cell_type": "code",
   "execution_count": 132,
   "metadata": {},
   "outputs": [
    {
     "data": {
      "image/png": "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\n",
      "text/plain": [
       "<Figure size 720x720 with 1 Axes>"
      ]
     },
     "metadata": {
      "needs_background": "light"
     },
     "output_type": "display_data"
    }
   ],
   "source": [
    "# Obstacle Locations\n",
    "w = 0.2\n",
    "# obstacles = [\n",
    "#               np.array([[0, 0], [1, 0], [1, 0.1], [0, w]]),\n",
    "#               np.array([[0, 0], [w, 0.2], [0.1, 2], [0.0, 2.0]]),\n",
    "#               np.array([[0, 2-w], [1, 2], [1, 2+w], [0, 2+w]]),\n",
    "#               np.array([[1-w, 0], [1+w, 0], [1+w, 1], [1, 1]]),\n",
    "#               np.array([[1-w, 2+w], [1+w, 2+w], [1+w, 1.5], [1, 1.5]]),\n",
    "#               np.array([[0.8, 1], [1+w, 1], [1+w, 1+w], [0.8, 1+w]]),\n",
    "#               np.array([[0.8, 1.5], [1+w, 1.5], [1+w, 1.5+w], [0.8, 1.5+w]])\n",
    "#             ]\n",
    "obstacles = []\n",
    "\n",
    "\n",
    "# Set Start and Goal locations\n",
    "p_start = [0.5, 0.5]\n",
    "p_goal = [0, -1]\n",
    "\n",
    "# initialization\n",
    "robot = Robot()\n",
    "robot.ballradius = 0.5\n",
    "robot.p = p_start\n",
    "\n",
    "node = Node()\n",
    "param = Params()\n",
    "\n",
    "\n",
    "from matplotlib.patches import Polygon\n",
    "\n",
    "# Bounds on world\n",
    "world_bounds_x = [-2.5, 2.5]\n",
    "world_bounds_y = [-2.5, 2.5]\n",
    "\n",
    "\n",
    "# Draw obstacles\n",
    "fig = plt.figure(figsize=(10, 10))\n",
    "plt.grid()\n",
    "ax = plt.gca()\n",
    "ax.set_xlim(world_bounds_x)\n",
    "ax.set_ylim(world_bounds_y)\n",
    "for k in range(len(obstacles)):\n",
    "    ax.add_patch( Polygon(obstacles[k]) )\n",
    "    \n",
    "# Plan the path\n",
    "P = PlanPathRRT(robot, obstacles, param, p_start, p_goal)\n",
    "\n",
    "\n",
    "\n",
    "# Plot the path from RRT\n",
    "# for i in range(1, len(P)):\n",
    "#     plt.plot([P(1,i);P(1,i-1)],[P(2,i);P(2,i-1)],'r','LineWidth',3)"
   ]
  }
 ],
 "metadata": {
  "kernelspec": {
   "display_name": "Python 2",
   "language": "python",
   "name": "python2"
  },
  "language_info": {
   "codemirror_mode": {
    "name": "ipython",
    "version": 3
   },
   "file_extension": ".py",
   "mimetype": "text/x-python",
   "name": "python",
   "nbconvert_exporter": "python",
   "pygments_lexer": "ipython3",
   "version": "3.5.2"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 2
}

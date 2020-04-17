# Motion Planning

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/rrt/rrt3D.png" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/apf/repulsive_potential.jpg" width="400"/>

## MATLAB

[Computational Motion Planning](https://www.coursera.org/learn/robotics-motion-planning) course from Penn.
Matlab implementation of the tasks can be found in
[matlab_src](https://github.com/RuslanAgishev/motion_planning/tree/master/matlab_src) folder.
Each of the subfolder includes `run.m` script for simulation launching and helper functions.
In order to launch the algorithms simply execute from your Matlab command prompt:
```matlab
run.m
```

## Python

Python code for several path planning algorithms is located inside
[python_src](https://github.com/RuslanAgishev/motion_planning/tree/master/python_src) folder.
Let's go through a couple of examples.

### [APF](https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1986_IJRR.pdf)

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/apf/gradient_route.jpg" width="400"/>
In order to get familiar with the Artificial Potential Filds
(APF) algorithm:

```bash
jupyter-notebook python_src/adaptive_formation/GradientBasedPlanning.ipynb
```

- Real time potential fields-based obstacle avoidance method for robots formations with moving or static obstacles.
```bash
python python_src/adaptive_formation/gradient_interactive.py
```

### [RRT](http://lavalle.pl/rrt/)

- Road map and path construction with Rapidly exploring Random Tree (RRT) algorithm:
```bash
python python_src/rrts/main_rrt2D.py
```
in 3D environment:
```bash
python python_src/rrts/3D/rrt3D.py
```

### Layered planner (RRT+APF)
An example of layered planner with RRT as a global path constructor and APF is responsible for local trajectory creation.
The algorithm is provided not only for an ego-vechicle but also for a group of robots.

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/layered_planner/layered_planner1_traj.png" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/layered_planner/layered_planner4_traj.png" width="400"/>

- Multi-layered planner for formation of robots navigation based on RRT+APF algorithms. Take a look on the [package](https://github.com/RuslanAgishev/adaptive_swarm "RRT+APF layered planner")
 for more details: 
```bash
python python_src/layered_planner/main_rrt_gradient.py
```

### Mapping and Exploration

- Exploration of the environment with inknown obstacles location. Random walk algorithm implementation for a mobile robot
equipped with 4 ranger sensors (front, back, left and right) for obstacles detection.
```bash
python python_src/exploration/random_goals_following/main.py
```
```bash
python python_src/exploration/random_walk/main.py
```
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/exploration/autonomous_exploration.gif" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/exploration/random_walk.png" width="400"/>

- Coverage path planning for unknown map exploration. Robot's kinematics is taken into account in velocity motion model.
```bash
python3 python_src/exploration/coverage_path_planning/main.py
```

- Mapping of the unknown environment using one or swarm of robots equipped with 4 ranger sensors. Localization data and scans are given in csv-files.
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/mapping/2_drones_multiranger_map.png" width="400"/>

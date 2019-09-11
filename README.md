# Motion Planning

[Computational Motion Planning](https://www.coursera.org/learn/robotics-motion-planning) course from Penn. Matlab implementation of the tasks can be found in matlab_src folder.
Each of the subfolder includes run.m script for simulation launching and subfunctions.
In order to launch the algorithms simply execute:
```matlab
run.m
```
in your Matlab command line.

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/rrt/rrt3D.png" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/apf/repulsive_potential.jpg" width="400"/>
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/layered_planner/layered_planner4_traj.png" width="500"/>

Python code that is aimed to apply path planning algorithms for real mobile robots is located in pthon_src folder.
In order to get familiar with the Artificial Potential Filds (APF) algorithm:
```bash
jupyter-notebook GradientBasedPlanning.ipynb
```
- Real time potential fields-based obstacle avoidance method for robots formations with moving or static obstacles.
```bash
python python_src/adaptive_formation/gradient_interactive.py
```
- Road map and path construction with Rapidly exploring Random Tree (RRT) algorithm:
```bash
python python_src/rrts/main_rrt2D.py
```
in 3D environment:
```bash
python python_src/rrts/3D/rrt3D.py
```
- Multi-layered planner for formation of robots navigation based on RRT+APF algorithms. Take a look on the [package](https://github.com/RuslanAgishev/adaptive_swarm "RRT+APF layered planner")
 for more details: 
```bash
python python_src/layered_planner/main_rrt_gradient.py
```
- Exploration of the environment with inknown obstacles location. Random walk algorithm implementation for a mobile robot
equipped with 4 ranger sensors (front, back, left and right) for obstacles detection.
```bash
python python_src/random_goals_following/main.py
```
```bash
python python_src/random_walk/main.py
```
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/exploration/random_goals_following.png" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/exploration/random_walk.png" width="400"/>

- Mapping of the unknown environment using a robot with 4 ranger sensors. Localization data and scans are given in csv-files.
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/mapping/2_drones_multiranger_map.png" width="400"/>

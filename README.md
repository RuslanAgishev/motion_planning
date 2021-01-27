# Motion Planning

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/layered_planner/bugtrap_4drones.gif" width="600"/>

<!-- <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/rrt/rrt_path.png" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/apf/repulsive_potential.jpg" width="400"/> -->

## Python

Python code for several path planning algorithms is located inside
[python_src](https://github.com/RuslanAgishev/motion_planning/tree/master/python_src) folder.
Let's go through a couple of examples.


### [APF](https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1986_IJRR.pdf)

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/apf/gradient_route.jpg" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/apf/repulsive_potential.jpg" width="400"/>

In order to get familiar with the Artificial Potential Filds
(APF) algorithm:

```bash
jupyter-notebook python_src/adaptive_formation/GradientBasedPlanning.ipynb
```

Real time potential fields-based obstacle avoidance method for robots formations with moving or static obstacles.
```bash
python python_src/adaptive_formation/gradient_interactive.py
```


### [RRT](http://lavalle.pl/rrt/)

Road map and path construction with Rapidly exploring Random Tree (RRT) algorithm:
```bash
python python_src/rrts/main_rrt2D.py
```
in 3D environment:
```bash
python python_src/rrts/3D/rrt3D.py
```
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/rrt/rrt_short_path1.png" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/rrt/rrt3D.png" width="400"/>

Here the RRT nodes and edges are depicted in blue, the retrieved path out of the tree is green, while the orange curve is a shortened trajectory .


### Layered planner (RRT+APF)
An example of layered planner with RRT as a global path constructor and APF is responsible for local trajectory creation.
The algorithm is provided not only for an ego-vechicle but also for a group of robots.

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/layered_planner/layered_planner1_traj.png" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/layered_planner/layered_planner4_traj.png" width="400"/>

Multi-layered planner for formation of robots navigation based on RRT+APF algorithms: 
```bash
python python_src/layered_planner/main_rrt_gradient.py
```
Take a look at the [adaptive_swarm](https://github.com/RuslanAgishev/adaptive_swarm "RRT+APF layered planner") package
for implementation details. There you will find how to apply a layered planner algorithm for a swarm of
nano quadrotors.
 
 
### Coverage Path Planning
Exploration of the environment with unknown obstacles location. Random walk algorithm implementation for a mobile robot
equipped with 4 ranger sensors (front, back, left and right) for obstacles detection. An example of a robot with similar sensors setup could a Crazyflie drone with a [multiranger](https://www.bitcraze.io/products/multi-ranger-deck/) deck mounted.
```bash
python python_src/exploration/random_goals_following/main.py
```
```bash
python python_src/exploration/random_walk/main.py
```
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/exploration/autonomous_exploration.gif" width="400"/> <img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/exploration/random_walk.png" width="400"/>

Coverage path planning for unknown map exploration. Robot's kinematics is taken into account in velocity motion model.
```bash
python python_src/exploration/coverage_path_planning/main3D.py
```
<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/coverage_path_planning/cpp_3D_view.png" width="500"/>


### Mapping with a group of robots

Mapping of an unknown environment using one or swarm of robots equipped with 4 ranger sensors.
Here we assume that robots localization data is provided
(prerecorded in csv files [here](https://github.com/RuslanAgishev/motion_planning/tree/master/python_src/mapping/csvs)).
The occupancy grid is constructed from multiranger pointcloud data
using [Bresenham](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
raytracing algorithm.

```python
python python_src/mapping/2robots_occupancy_grid.py
```

<img src="https://github.com/RuslanAgishev/motion_planning/blob/master/figures/mapping/2_drones_multiranger_map.png" width="400"/>

## MATLAB

[Computational Motion Planning](https://www.coursera.org/learn/robotics-motion-planning) course from Penn.
Matlab implementation of the tasks can be found in
[matlab_src](https://github.com/RuslanAgishev/motion_planning/tree/master/matlab_src) folder.
Each of the subfolder includes `run.m` script for simulation launching and helper functions.
In order to launch the algorithms simply execute from your Matlab command prompt:
```matlab
run.m
```

## Reference
- [Robotics: Computational Motion Planning by Penn](https://www.coursera.org/learn/robotics-motion-planning)
- [ColumbiaX Robotics](https://www.edx.org/course/robotics-2)
- [AtsushiSakai/PythonRobotic](https://github.com/AtsushiSakai/PythonRobotics)

## License <a name="license"></a>
Project is distributed under [MIT License](https://github.com/RuslanAgishev/motion_planning/blob/master/LICENSE)

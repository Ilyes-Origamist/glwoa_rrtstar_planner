
# GLWOA-RRT* Global Planner
A ROS implementation (C++) of the path finding algorithm GLWOA-RRT* as global planner, which uses RRT* to find an initial path, then employ optimization using the Whale Optimization Algorithm (WOA). The approach is detailed in our article [here](https://doi.org/10.1016/j.swevo.2025.102062).

This implementation could be easily modified to change the optimizer, implement an RRT* variant or WOA variant. If you use this software for an approach inspired from our method, please consider citing our work.

<img src="tests/demo.gif" width="720" />

## Features
- Run multiple tests a time with stats displayed (runtime, path cost),
- Visualize paths and RRT* tree in Rviz, 
- Enable/disable the optimizer (WOA),
- Change parameters in real-time using `rqt_reconfigure`.
- Testing framework allowing to quickly run other global planners and local planners with ready-to-use launch and config files (see section [1.3](#13-parameters)), and test muliple environments (provided Gazebo worlds with their corresponding maps, section [1.4](#14-environment-setup)).

## Notes
- Since ROS EOL has been reached, I would like to migrate this project to ROS2. Any collaboration is appreciated.
- I have used and improved the RRT* implementation of Rafael Barreto found [here](https://github.com/rafaelbarretorb/rrt_star_global_planner). RRT* is now faster and more efficient. Improvements and fixes are listed in the last section (this is not a fork or patch of the original code).
- A spatial grid search with a fallback to brute force is used to find the nearest neighbors and the nearest node faster than brute force.
- You can easily use OMPL's implementation found [here](https://ompl.kavrakilab.org/classompl_1_1NearestNeighbors.html) for more efficient nearest neighbors and nearest node queries.


## Summary
- 1\. [Installation and Usage](#1-installation-and-usage)
- 2\. [Architecture and Implementation](#2-architecture-and-implementation)
- 3\. [Performance Improvements and Bug Fixes](#3-performance-improvements-and-bug-fixes)
- 4\. [Development and Debugging](#4-development-and-debugging)


## 1. Installation and Usage
### 1.1. Requirements and Installation
ROS Noetic on Ubuntu 20.04 installed, with C++ 17.<br>
Dependencies include ROS navigation stack, AMCL, dynamic reconfigure, etc. You can install them using `rosdep` or `apt`.

Clone the repository inside your ROS workspace:
```bash
cd ~/ros_ws/src
git clone https://github.com/ilyesorigamist/glwoa_rrtstar_planner.git
```
Install dependencies:
```bash
sudo rosdep init
rosdep update && cd ros_ws/src/glwoa_rrtstar_planner
rosdep install --from-paths src --ignore-src -r -y
```
Compile the package:
```bash
cd ../.. && catkin_make --pkg glwoa_rrtstar_planner
```
### 1.2. Run the Planner with Gazebo Simulation
Launch files are provided to launch Gazebo simulation with the ROS navigation stack and necessary components. The package also contains Gazebo worlds and their respective maps.

Start a ROS master:
```bash
roscore
```
Open a new terminal and start Gazebo world first (before spawning the robot to avoid errors):
```bash
roslaunch glwoa_rrtstar_planner world.launch
```
Open a new temrinal and run everything else using:
```bash
roslaunch glwoa_rrtstar_planner turtlebot_planner.launch
```
**Send a Goal** <br>
You can either use Rviz to publish a goal, or run the script `send_goal` after setting the goal parameter:
```bash
rosrun glwoa_rrtstar_planner send_goal
```

### 1.3. Parameters
\- All parameters are specified in `.yaml` files inside the `\config` directory.<br>
\- GLWOA-RRT* parameters are provided in `glwoa_rrtstar_planner.yaml`.<br>
\- Parameter configuration files are also provided for AMCL, local planners (DWA, TEB, TrajectoryPlannerROS), global planners (GLWOA-RRT*, NavFn, A*/Dijkstra), local and global costmaps, and rviz file.
- **Initial position**

The initial position should be set in both the spawn and AMCL configs:<br>
\- For spawn, edit XYZ and RPY in `/launch/turtlebot.launch`.<br>
\- For AMCL, update the position in `turtlebot_amcl.launch`.

- **Select GLWOA-RRT\* or A\*/Dijkstra global planner**

In the file `/launch/turtlebot_move_base.launch`, you can comment/uncomment lines under `<!-- Global Planner -->` to select the planner and under `<!-- Global planner parameters -->` to select the parameters file (yaml). Similarly, you can also specify the local planner in this launch file (comment/uncomment).

- **GLWOA-RRT\* Parameters**

Go to the file `/config/glwoa_rrtstar_planner.yaml` to specify RRT* and WOA parameters:
```yaml
GlWoaRrtStarPlanner:
  # General Parameters
  use_optimizer: true
  number_of_tests: 1
  visualize_tree: true
  goal_tolerance: 0.2
  # RRT* Parameters
  rewiring_radius: 1.6
  epsilon: 0.15
  max_num_nodes: 10000
  # WOA Parameters
  sampling_radius: 0.1
  num_iterations: 100
  num_agents: 50
  spiral_shape: 0.85

SendGoal:
  goal_x: 0.4
  goal_y: -3.1
```
Additionally, you can change them when the planner is running (for tuning) with dynamic reconfigure using the command:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```


### 1.4. Environment Setup
I have provided multiple Gazebo worlds with their models and maps inside the directory `/gazebo`.
```bash
gazebo/
├── models/
├── worlds/
└── maps/
```

Don't forget to source the `models` directory by adding it to Gazebo path:
```bash
echo 'export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$HOME/ros_ws/src/glwoa_rrtstar/gazebo/models:$HOME/.gazebo/models"' >> ~/.bashrc
```

To change the environment (Gazebo World and map):<br>

\- Change world file: change the argument `world_name` inside `world.launch`,<br>
\- Change the map: use the same world name for the map name. Update the argument `map_file` inside `turtlebot_amcl.launch`.<br>
\- Change the scenario (start and goal positions). Examples are gven inside `config\scenarios.md`.<br>
For `small_house` and `bookstore` (by ASW Robomaker), install them from [small_house](https://github.com/aws-robotics/aws-robomaker-small-house-world) and [bookstore](https://github.com/aws-robotics/aws-robomaker-bookstore-world).

To do this, put all the contents of their `models` directory inside `~/.gazebo/models`, or inside a folder then source it (as done previously).

Then open a new terminal (important to apply changes). You can check the path by running:
```bash
echo $GAZEBO_MODEL_PATH 
```
And you should see the two paths.



## 2. Architecture and Implementation
### 2.1. GlWoaRrtStarPlanner (`glwoa_rrtstar_planner.cpp`)
It is the main program which runs RRT* and WOA, publishes their paths and publish markers to visualize the tree with paths in Rviz.

The `makePlan` function runs everything. It is called by `move_base` automatically when a custom planner is registered and used, which is the case for `glwoa_rrtstar_planner/GlWoaRrtStarPlanner` global planner.

The function `initialPath` runs RRT* via an instance of `RRTStar` and stops when it finds an initial path. The function `woaOptimizePath` runs WOA using an instance of `PathAgent` for each agent.

### 2.2. RRTStar (`rrt_star.cpp`)
Implements all the functions required by RRT*, and the main program `initialPath`. It finds the initial path by calling almost all the class methods. 
```cpp
RRTStar::initialPath(std::list<std::pair<float, float>> &path)
```

Optimized functions that use spatial grid-based search do not contain any prefix, like `createNewNode`, `chooseParent` and `rewire`. If they fail or the number of nodes is small, brute force is used with the prefix `BruteForce` (methods like `chooseParentBruteForce`, `rewireBruteForce`).

The function `computeFinalPath` computes the path from start to goal when the goal is reached using backtracking from the goal to start (moving through parent nodes).

In `GlWoaRrtStarPlanner`, a pointer `planner_` to `RRTStar` is created (for each test) specifying the necessary arguments of the constructor, then we run RRT* using 
```cpp
planner_->initialPath(path);
```
Which directly modifies the variable `path` (as argument) instead of return.

**Spatial Grid-based Search**

\- **Rationale**<br>
  - The map is divided into a grid with square cells of size `grid_cell_size_`.
  - Each cell can contain multiple nodes. When any node is created, it is assigned the corresponding cell using `updateSpatialGrid`.
  - By setting the grid cell size to `2*rewiring_radius_`, we ensure that any point within the radius is contained in one of the 4 neighboring cells, no more.
  - This allows for efficient nearest neighbors search.

\- **Data Structure**
  - `spatial_grid_`: a vector of vectors used to store the indices of nodes in each grid cell.
  - Each element of `spatial_grid_[i]` contains all the nodes' IDs that are in the cell `i`.
  - `updateSpatialData`: assign the new node to the corresponding grid cell (index) in the spatial grid structure according to its position in the map.
  - `near_node_indices`: this vector is used to store the indices of nearby nodes found in the 4 grids surrounding the region of near neighbors (circle). This way, search of neighboring nodes is optimized by only checking these indices (of nodes lying inside the 4 neighboring cells) instead of all nodes in the RRT* tree.

\- **Limitation:**<br>
  - The nearest node from the new sampled point is not guaranteed to be within the rewiring radius, especially in the early iterations.
  - If this is the case, a fallback to brute force search is used.
  - Thus, search for the nearest node is not always optimized. 
  - A better approach may be a greedy algorithm that finds the nearest grid and extend the search to neighboring grids.

### 2.3. PathAgent (`woa_agent.cpp`)
In `woa_agent.cpp`, a single agent is implemented. The class allows manipulating the agent using the necessary methods, such as: `spiralUpdate`, and `circularUpdate`, and stores variables like `A`, `C` and `l` as data members. An agent is respresented as a vector `X` of (x,y) coordinates: `[x1, y1, x2, y2, ..., xn, yn]` (armadillo vector `arma::vec`).

In the main program, we run multiple instances of an agent, which is done in `woaOptimizePath`. The class can be easily used for any other metaheuristic optmization algorithm with some modifications. 

In our case in `glwoa_rrtstar_planner.cpp`, a vector of pointers is used to store agents:
```cpp
agents.emplace_back(std::make_unique<PathAgent>(initial_path, sampling_radius_, i, costmap_, spiral_shape));
```
So, `agents[i]` is a pointer to the i-th agent object (without name). We use it to update its data members and call its methods.

Note that a vector of pointers is used instead of a vector of objects to avoid moving the object `PathAgent` which will make errors since `RandomDoubleGenerator` is non-movable.

The function `randomInitialPath` is used to initialize WOA population based on RRT* initial path (procedure described in the article). It is run in the agent's constructor (when the object is created), i.e. in the line initializing agents with `agents.emplace_back`. 


## 3. Performance Improvements and Bug Fixes
**3.1. `RandomDoubleGenerator`**<br>
In the original code (Parreto's code), I found that the sampling algorithm is taking so much time, about 25% to 70% overall (summing all the time periods whenever `sampleFree()` is called, which uses the random double generator).

The reason is that the random engine object (Mersenne Twister engine) is reinitialized in every call to generate a random number. Instead of this, we can initialize the engine once in the constructor, and reuse it throughout the sampling process. Same goes with the distribution.
```cpp
// Initialize the Mersenne Twister generator `gen` with a random seed from the random device `rd_()`
RandomDoubleGenerator::RandomDoubleGenerator() : gen(rd_()) {}
```
Then to generate a random number, just use:
```cpp
double RandomDoubleGenerator::generateFirst() {
  return dist_first_(gen);
}
```
with `dist_first_` a distribution created in the constructor, or using the function `setRangeFirst`.

This reduced overhead significantly.

The class has also been extended to generate two numbers with different ranges for non-square maps.

**3.2. `CollisionDetector`**

I have corrected a mistake in the function `istThereObstacleBetween`, specifically the angle `theta` between the point and node. The angle should be from node to point instead of the angle from point to node. 

Mistake (from point to node): 
```cpp
  float theta = atan2(node.y - point.second, node.x - point.first);
```
Correction (from node to point):
```cpp
  float theta = atan2(point.second - node.y, point.first - node.x);
```

Then, we increment using `theta`:
```cpp
    p_n.first = node.x + n*resolution_*cos(theta);
    p_n.second = node.y + n*resolution_*sin(theta);
```
The mistake in the original code caused moving in the opposite direction, making the path pass through obstacles sometimes. It took me some time troubleshooting the issue to find that mistake.

**3.3. `RRTStar`**

\- **Finding Neighboring Nodes**<br>
Finding the nearest node and neighbors is the most costy operation in RRT* which takes most of its runtime. Search for neighboring nodes has been improved using spatial grid search: instead of iterating over all the tree's nodes, we only look for the nodes in the 4 cells containing the circle (as explained previously).

This search is implemented in `getNearNodes` to get the indices of the nodes contained in the 4 cells covering the circle of raduis `2 * rewiring_radius`. Then, these indices are used in `createNewNode`, `chooseParent` and `rewire`. In `CreateNewNode`, a fallback to brute force (original functions) is used in case nothing is found, but it will probably never occur.

\- **Finding the Nearest Node**<br>
For finding the nearest node, I have used the same approach (searching for the 4 cells only then fallback to brute force). However, the fallback to brute force in this case may occur often since the nearest node to the new point is not necessarily in the neighbring area, especially in the early iterations or when the environment is large. To improve search efficiency and scalability to large environments, I developing a greedy algorithm that I may implement in the future.


## 4. Development and Debugging
I have provided some debugging logs which are commented (collision rate, out of bound rate, time measurement for specific functions..etc).

You can uncomment them and modify them to evaluate performance or troubleshoot issues you encounter.

Don't forget to build the package after any modification because it's in C++ and needs to be compiled before running (unlike Python). You can just run:
```bash
# move to your ROS workspace
cd ~/ros_ws
# build the package
catkin_make --pkg glwoa_rrtstar_planner 
```


# Frontier Based Autonomous Exploration with ROS2

This project implements an autonomous, frontier-based exploration algorithm using ROS2. The robot navigates toward frontier points—the boundaries between explored and unexplored areas—to efficiently map its environment.

Based on the [original work](https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot) by [abdulkadrtr](https://github.com/abdulkadrtr).

---

## Overview

The algorithm uses the ROS2 Navigation Stack (Nav2) and SLAM to enable the robot to autonomously explore and build a map of its environment by continually identifying and traveling to frontier points. During the exploration process, the total exploration time is appended to a CSV file named `fbae_goal_times.csv`.


<img src="exploration.gif" width=550px>

---

## Requirements

- **ROS2 Humble**
- **Nav2 Navigation Stack**
- **go2_ros2_sdk** (for Unitree Go2)
- **SLAM Toolbox** (for TurtleBot 3)
- **TurtleBot Packages**
  - `turtlebot4` (for TurtleBot 4)
  - `turtlebot3` (for TurtleBot 3)

---

## Installation

1. **Clone the Repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/akifbayram/ros2_frontierbasedexploration.git
   ```

2. **Build and Source the Workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build

   source ~/ros2_ws/install/setup.bash
   ```

---

## Unitree Go2

1. **Launch go2_ros2_sdk**:

   Update the go2_ros2_sdk source directory if needed.

   ```bash
   source ~/ros2_ws/install/setup.bash &&  
   export ROBOT_IP="x.x.x.x" &&
   export CONN_TYPE="webrtc" && 
   ros2 launch go2_robot_sdk robot.launch.py
   ```

2. **Run the Exploration Algorithm**:

   ```bash
   source ~/ros2_ws/install/setup.bash && 
   ros2 run autonomous_exploration control_go2
   ```

---

## TurtleBot 4 Simulation

1. **Launch the Simulation**:
   ```bash
   source /etc/turtlebot4/setup.bash && 
   ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=false world:=maze
   ```
   Parameters:
   - `slam:=true`: Enables SLAM.
   - `nav2:=true`: Starts Nav2.
   - `world:=maze`: Specifies the simulation world. Other options: depot, warehouse.

2. **Launch RViz2**:
   ```bash
   source ~/ros2_ws/install/setup.bash && 
   source /etc/turtlebot4/setup.bash &&
   ros2 run rviz2 rviz2 -d ~/ros2_ws/src/autonomous_exploration/rviz/tb4.rviz
   ```

3. **Run the Exploration Algorithm**:
   ```bash
   source ~/ros2_ws/install/setup.bash && 
   source /etc/turtlebot4/setup.bash &&
   ros2 run autonomous_exploration control_tb4
   ```

**Optional Nav2 Configuration**: To avoid path planning through unknown areas, edit `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`:

```yaml
planner_server:
  ros__parameters:
    GridBased:
      allow_unknown: false
```

---

## TurtleBot 3 Simulation

1. **Launch SLAM Toolbox**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch slam_toolbox online_async_launch.py
   ```

2. **Launch the Simulation**:
   ```bash
   export TURTLEBOT3_MODEL=burger &&
   source ~/ros2_ws/install/setup.bash &&
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

3. **Run the Exploration Algorithm**:
   ```bash
   source ~/ros2_ws/install/setup.bash &&
   ros2 run autonomous_exploration control_tb3
   ```

4. **Launch RViz2** (optional):
   ```bash
   source ~/ros2_ws/install/setup.bash &&
   ros2 run rviz2 rviz2 -d ~/ros2_ws/src/autonomous_exploration/rviz/tb3.rviz
   ```

---

## Acknowledgments

Based heavily on the work by [abdulkadrtr](https://github.com/abdulkadrtr).
# Frontier Based Autonomous Exploration with ROS2 and TurtleBot Simulations

This project implements an autonomous, frontier-based exploration algorithm using ROS2, enabling TurtleBot robots to autonomously map unknown environments.

Based on the [work](https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot) by [abdulkadrtr](https://github.com/abdulkadrtr).

---

## Overview

The algorithm uses ROS2 Navigation Stack (Nav2) and SLAM to allow the robot to explore by navigating to frontiers—the boundaries between explored and unexplored areas—thus building a map of the environment.

---

## Requirements

- **ROS2 Humble**
- **Nav2 Navigation Stack**
- **SLAM Toolbox** (for TurtleBot 3)
- **TurtleBot Packages**
  - `turtlebot4` (for TurtleBot 4)
  - `turtlebot3` (for TurtleBot 3)

---

## Installation

1. **Install ROS2 Humble**: Follow the [official guide](https://docs.ros.org/en/humble/Installation.html).

2. **Install TurtleBot Packages**:

   - For TurtleBot 4:
     ```bash
     sudo apt install ros-humble-turtlebot4*
     ```
   - For TurtleBot 3:
     ```bash
     sudo apt install ros-humble-turtlebot3*
     ```

3. **Install SLAM Toolbox** (for TurtleBot 3):
   ```bash
   sudo apt install ros-humble-slam-toolbox
   ```

4. **Clone the Repository**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/autonomous_exploration.git
   ```
   *Replace `yourusername` with your GitHub username or the correct repository URL.*

5. **Build the Workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

6. **Source the Workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## TurtleBot 4 Simulation

### Setup Steps

1. **Launch the Simulation**:
   ```bash
   ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=false world:=depot
   ```
   Parameters:
   - `slam:=true`: Enables SLAM.
   - `nav2:=true`: Starts Nav2.
   - `world:=depot`: Specifies the simulation world.

2. **Launch RViz2**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run rviz2 rviz2 -d ~/ros2_ws/src/autonomous_exploration/config/autonomous_exploration_tb4.rviz
   ```

3. **Run the Exploration Algorithm**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run autonomous_exploration control_tb4
   ```

### Nav2 Configuration

Modify Nav2 parameters to prevent path planning through unknown areas:

**File**: `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`

**Change**:
```yaml
planner_server:
  ros__parameters:
    GridBased:
      allow_unknown: false  # Set from true to false
```

---

## TurtleBot 3 Simulation

### Setup Steps

1. **Launch SLAM Toolbox**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch slam_toolbox online_async_launch.py
   ```

2. **Launch the Simulation**:
   ```bash
   export TURTLEBOT3_MODEL=burger
   source ~/ros2_ws/install/setup.bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
   *You can set `TURTLEBOT3_MODEL` to `burger`, `waffle`, or `waffle_pi`.*

3. **Run the Exploration Algorithm**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run autonomous_exploration control_tb3
   ```

4. **Launch RViz2** (optional):
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run rviz2 rviz2 -d ~/ros2_ws/src/autonomous_exploration/config/autonomous_exploration_tb3.rviz
   ```

### Nav2 Configuration

Modify Nav2 parameters:

**File**: `/opt/ros/humble/share/turtlebot3_navigation2/param/nav2_params.yaml`

**Change**:
```yaml
planner_server:
  ros__parameters:
    GridBased:
      allow_unknown: false  # Set from true to false
```

---

## Acknowledgments

Based on work by [abdulkadrtr](https://github.com/abdulkadrtr).

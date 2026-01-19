# MicroMouse-Robot-Maze-Navigation-with-ROS2
This project features a sophisticated navigation stack capable of solving a 16x16 maze using both Depth-First Search (DFS) and Breadth-First Search (BFS) algorithms, fully integrated with ROS 2 Humble and the MicroMouse Simulator (MMS).

# 🚀 Overview
The repository contains a multi-package ROS 2 system that implements autonomous maze exploration and pathfinding. The architecture emphasizes strong C++ Core Guidelines, including RAII, thread safety, and clean encapsulation.

## Key Features
* **Dual-Algorithm Support**: Polymorphic implementation of **DFS** for deep exploration and **BFS** for finding the mathematically optimal shortest path.
* **Advanced Navigation Logic**: Features LCA-based (Lowest Common Ancestor) path reconstruction to handle dynamic replanning when new walls are detected.
* **ROS 2 Communication Stack**:
  * **Actions**: Long-running navigation tasks with real-time feedback and cancellation support.
  * **Services**: Instantaneous robot status queries.
  * **Topics**: High-frequency robot position and orientation broadcasting.
* **Thread Safety**: Uses a ```MultiThreadedExecutor``` and ```std::mutex``` to prevent race conditions during asynchronous action callbacks.

# 🛠 System Architecture
* Design Patterns
  * **Composition**: The Robot class owns the Maze (lifecycle bound).
  * **Aggregation**: The Robot uses an Algorithm pointer, making the search logic swappable at runtime.
  * **Encapsulation**: All simulator-specific API calls are abstracted within a Maze wrapper to ensure the logic remains independent of the simulation environment.

* Custom Interfaces (```micromouse_interfaces```): The project defines specialized ROS 2 message types to facilitate communication
  * ```MapsToGoal.action```: Goal ($x, y$), Result (success, steps, time), and Feedback (current $x, y$, direction, elapsed time).
  * ```GetRobotStatus.srv```: Returns position, direction, steps taken, and estimated steps to the goal.

# 📂 Repository Structure
```
├── micromouse_interfaces     # Custom Action and Service definitions
├── micro_mouse_logic         # Core C++ navigation and algorithm implementation
│   ├── src/dfs_algorithm.cpp # Stateful DFS with LCA reconstruction
│   ├── src/bfs_algorithm.cpp # Bonus BFS implementation
│   └── src/micromouse_node.cpp # Main ROS 2 Node with Action Server
└── config/params.yaml        # YAML configuration for goals and colors
```

# ⚙️ Setup and Installation
* **Prerequisites**:
  * ROS 2 Humble
  * MicroMouse Simulator (MMS)
  * C++17 Compiler 

* **Build**:
```
mkdir -p ~/micromouse_ws/src
cd ~/micromouse_ws/src
git clone <repository_url> .
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
source install/setup.bash
```

# 🏃 Usage
**Launch Navigation**: To run the robot in action mode with parameters:

```
ros2 run micro_mouse_logic micromouse_node --ros-args --params-file src/config/params.yaml
```

**Request Robot Status**: To query the robot status from a standalone node:

```
ros2 run micro_mouse_logic service_client
```

**Configuration** ```(params.yaml)```
* You can modify the following in ```config/params.yaml```:
  * ```goal_x / goal_y```: Target coordinates (default: 7, 7).
  * ```path_color```: Color of the path in MMS (default: "c" for cyan).
  * ```standalone_mode```: Set to ```true``` for immediate navigation.

# Outcomes
* Developed a ROS2 MicroMouse navigation system using C++ and custom Action/Service interfaces.
* Implemented a thread-safe control architecture to manage real-time robot positioning and asynchronous goal tracking.

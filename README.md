# Maze-Solving-Bot

# 🧠 Maze-Solving Robot Simulation (ROS2 + Gazebo + OpenCV)

This project simulates a maze-solving robot in a full ROS2 + Gazebo environment with OpenCV-based vision — developed entirely on **Windows using Docker**. It integrates perception, path planning, and control in a single robotic loop.

## 🚀 Features

- Differential drive robot with `gazebo_ros_diff_drive`
- Vision-based background extraction using OpenCV for basic localization
- Pathfinding using A*, Dijkstra, and Depth-First Search (DFS)
- Optimized search via Min Heap-based priority queue
- Go-To-Goal (GTG) behavior with basic obstacle avoidance
- Fully containerized for platform-agnostic development (Windows/Linux)

---

## 🧱 Technologies Used

- **ROS2 Foxy**
- **Gazebo**
- **OpenCV (Python)**
- **Docker** (volume-mounted workspace)
- **Python**

---

## ✅ Running Project

cd /root/ros2_ws
colcon build --packages-select maze_solve
source install/setup.bash

# Launch robot in Gazebo
ros2 launch maze_solve maze_world.launch.py

# Run the path planning node
ros2 run maze_solve maze_solver


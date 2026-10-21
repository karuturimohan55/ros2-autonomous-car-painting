# ROS 2 Autonomous Car Painting Cell

A robotics simulation project that explores autonomous vehicle painting using **ROS 2 Humble** and **Gazebo Classic**.

## Project Overview

This project simulates a robotic painting cell with a mobile robot base, a robotic arm, and a spray-painting tool. It includes motion planning, robot control, waypoint-based movement, and collision/safety checks.

## Technologies Used

- ROS 2 Humble
- Gazebo Classic 11
- Python
- URDF / Xacro
- CMake

## Key Components

- **Robot model:** Mobile base with robotic arm and painting tool
- **Simulation environment:** Vehicle body and painting-cell models
- **Trajectory planning:** Waypoint-based trajectory generation and execution
- **Safety:** Joint-limit validation and collision-checking utilities
- **Testing:** Python scripts for kinematics, robot control, navigation, and safety checks

## Repository Structure

```text
.
├── Screenshots/
├── Simulation_Files/
│   └── src/
│       └── car_painting_cell/
│           ├── config/
│           ├── docs/
│           ├── launch/
│           ├── models/
│           ├── scripts/
│           ├── urdf/
│           └── worlds/
└── .gitignore
```

## Setup

Use Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic installed.

Clone the repository:

```bash
git clone https://github.com/karuturimohan55/ros2-autonomous-car-painting.git
cd ros2-autonomous-car-painting/Simulation_Files/src
```

Build the ROS 2 package:

```bash
source /opt/ros/humble/setup.bash
cd /home/$USER/car_painting_ws
```

Copy the package into your workspace's `src` directory if needed, then run:

```bash
colcon build --packages-select car_painting_cell
source install/setup.bash
```

Refer to the package's launch files and documentation for the available simulation and testing procedures.

## Evidence

Screenshots of the simulation environment, robot poses, painting stages, and test evidence are available in the `Screenshots/` directory.

## Author

**Mohan Karuturi**

## Project Status

This is a simulation and development project. Refer to the documentation for test coverage, known limitations, and the specific scenarios verified during execution.

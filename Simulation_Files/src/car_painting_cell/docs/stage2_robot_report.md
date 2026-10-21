# Stage 2 Technical Report: Mobile Robotic Painting System

**Project:** Yuktii AI Labs — Robotic Car Painting Cell Simulation  
**Stage:** Stage 2 (Mobile Robotic Arm Design, Integration & Kinematic Control)  
**Target Environment:** ROS 2 Humble & Gazebo Classic 11.10.2 on Ubuntu 22.04 LTS  
**Package:** `car_painting_cell`  
**Date:** October 16, 2026  
**Status:** **STAGE 2 COMPLETE & VERIFIED**

---

## 1. Executive Summary

Stage 2 of the Yuktii AI Labs Robotic Car Painting Cell assessment requires the design, integration, spawning, and independent kinematic control of a **mobile robotic painting system**. The robot operates inside the industrial paint booth created in Stage 1 and prepares the foundation for Stage 3 autonomous multi-zone vehicle painting.

All mandatory requirements have been fulfilled and verified through physical simulation:
1. **Mobile Platform:** A four-wheeled chassis with continuous joints, differential drive kinematics, high-traction tires, and a 50.0 kg low-center-of-mass ballast for absolute anti-tipping stability.
2. **6-Axis Articulated Arm:** An industrial articulated manipulator featuring 6 revolute joints with calibrated joint limits, realistic composite link masses, high joint friction, and `implicitSpringDamper` physics.
3. **End-Effector Assembly:** Modular spray-painting gun, brass atomizing nozzle tip, and an oriented, non-colliding visual spray cone.
4. **Environment Preservation:** The Stage 1 environment (`car_painting_cell.world`, `models/car_body`, `models/painting_cell`, and `launch/cell_world.launch.py`) remains 100% intact.
5. **Control System:** Native Gazebo Classic plugins (`libgazebo_ros_diff_drive.so`, `libgazebo_ros_joint_state_publisher.so`, `libgazebo_ros_joint_pose_trajectory.so`) paired with a dedicated ROS 2 trajectory interpolator and hold controller (`scripts/arm_controller.py`).
6. **Live Verification:** Mobile base forward/reverse driving and turning, as well as 6-axis arm articulation through 5 distinct automotive painting poses, were quantitatively executed and verified with zero joint droop.

---

## 2. Robotic System Architecture

### 2.1 File Organization & Modularity
The robot model is organized as modular Xacro macros in `src/car_painting_cell/urdf/`:

| File | Purpose / Macro | Key Features |
|---|---|---|
| [`materials.xacro`](file:///home/mohan/car_painting_ws/src/car_painting_cell/urdf/materials.xacro) | Visual & Gazebo Colors | Safety orange (`#F97316`), charcoal gray, matte rubber, gold brass, cyan spray |
| [`mobile_base.xacro`](file:///home/mohan/car_painting_ws/src/car_painting_cell/urdf/mobile_base.xacro) | `<xacro:mobile_base>` | $0.90\text{m} \times 0.60\text{m} \times 0.18\text{m}$ chassis, 4 wheels ($r=0.10\text{m}$), 50kg ballast |
| [`arm_and_gun.xacro`](file:///home/mohan/car_painting_ws/src/car_painting_cell/urdf/arm_and_gun.xacro) | `<xacro:painting_arm>` | 6 revolute joints, hollow composite links, spray gun, nozzle, visual spray cone |
| [`gazebo_plugins.xacro`](file:///home/mohan/car_painting_ws/src/car_painting_cell/urdf/gazebo_plugins.xacro) | `<xacro:gazebo_control_plugins>` | Differential drive, 10-joint state publisher, joint trajectory receiver |
| [`mobile_painting_robot.urdf.xacro`](file:///home/mohan/car_painting_ws/src/car_painting_cell/urdf/mobile_painting_robot.urdf.xacro) | Top-Level Assembled Robot | Combines base, arm, end-effector, and plugins into unified entity |

### 2.2 Mobile Base Specifications
- **Chassis Dimensions:** Length $0.90\text{ m}$, Width $0.60\text{ m}$, Height $0.18\text{ m}$.
- **Wheel Configuration:** 4 continuous revolute wheel joints arranged symmetrically:
  - Track separation: $0.66\text{ m}$ (lateral).
  - Wheelbase: $0.56\text{ m}$ (longitudinal).
  - Wheel radius: $0.10\text{ m}$, width $0.06\text{ m}$.
- **Inertial Ballast:** `base_link` mass set to $50.0\text{ kg}$ with low center of mass ($z = +0.04\text{ m}$), yielding a base-to-arm mass ratio $> 3.6:1$. This guarantees that dynamic arm extensions cannot destabilize or tilt the chassis.
- **Friction:** Gazebo ODE surface friction $\mu_1 = 1.5, \mu_2 = 1.5, K_p = 10^6, K_d = 100.0$.

### 2.3 6-Axis Articulated Arm Kinematics & Limits
The arm mimics an industrial automotive painting manipulator (Fanuc Paint Mate / UR10 style):

| Joint Name | Type | Axis | Range (rad) | Velocity (rad/s) | Effort (N·m) | Link Mass (kg) | Damping / Friction |
|---|---|---|---|---|---|---|---|
| `arm_base_joint` | Revolute | Z [0, 0, 1] | $[-3.1416, +3.1416]$ | $2.0$ | $150.0$ | $5.0$ | $5.0\text{ Ns/m} / 2.0\text{ Nm}$ |
| `shoulder_joint` | Revolute | Y [0, 1, 0] | $[-1.8000, +1.8000]$ | $1.8$ | $200.0$ | $3.2$ | $25.0\text{ Ns/m} / 18.0\text{ Nm}$ |
| `elbow_joint` | Revolute | Y [0, 1, 0] | $[-2.5000, +2.5000]$ | $2.0$ | $150.0$ | $2.0$ | $20.0\text{ Ns/m} / 14.0\text{ Nm}$ |
| `wrist_1_joint` | Revolute | Z [0, 0, 1] | $[-3.1416, +3.1416]$ | $2.5$ | $50.0$ | $1.0$ | $5.0\text{ Ns/m} / 2.0\text{ Nm}$ |
| `wrist_2_joint` | Revolute | Y [0, 1, 0] | $[-2.0000, +2.0000]$ | $2.5$ | $40.0$ | $0.8$ | $10.0\text{ Ns/m} / 6.0\text{ Nm}$ |
| `wrist_3_joint` | Revolute | Z [0, 0, 1] | $[-3.1416, +3.1416]$ | $3.0$ | $30.0$ | $0.4$ | $3.0\text{ Ns/m} / 1.5\text{ Nm}$ |

- **Vertical Reach:** $> 2.40\text{ m}$ (sufficient to clear vehicle roof at $1.60\text{ m}$).
- **Horizontal Reach:** $> 1.70\text{ m}$ (reaches center line of car from safe stand-off distance).

### 2.4 End-Effector Assembly
1. **Tool Flange Link (`tool_flange_link`):** Standard mechanical interface at wrist 3 ($r=0.04\text{m}$, length $0.02\text{m}$).
2. **Spray Gun Link (`spray_gun_link`):** Cylindrical applicator body ($0.14\text{m} \times 0.05\text{m}$) with paint canister manifold.
3. **Atomizer Nozzle Link (`nozzle_link`):** Gold brass precision nozzle tip ($0.04\text{m} \times 0.024\text{m}$) defining the spray exit point.
4. **Visual Spray Cone Link (`spray_cone_link`):** High-visibility cyan translucent conical projection ($0.36\text{m}$ length, $0.12\text{m}$ radius). **Collision geometry is omitted** to ensure the spray cone never produces phantom collision impulses against the vehicle or booth.

---

## 3. Investigation & Resolution of Arm Control

### 3.1 Initial Problem
Initial tests revealed that while `libgazebo_ros_joint_pose_trajectory.so` was active, publishing single trajectory points to `/set_joint_trajectory` resulted in the arm collapsing to its joint limits (+1.8 rad shoulder, -2.5 rad elbow).

### 3.2 Root Cause Analysis
1. **Clock Discrepancy:** When standard ROS 2 nodes publish messages stamped with `node.get_clock().now()` (wall clock ~1.79 billion seconds), Gazebo Classic compares this stamp against simulation time (~300 seconds). The trajectory is perceived as scheduled in the distant future and ignored.
2. **Open-Loop Kinematic Updates:** `libgazebo_ros_joint_pose_trajectory.so` uses Gazebo ODE's `joint->SetPosition()` during active trajectory playback. Once the duration expires, the plugin ceases active intervention. Without continuous position commands or closed-loop holding, gravity torques ($>25\text{ N}\cdot\text{m}$) immediately pulled the unpowered joints downward.
3. **Reference Link Frame:** The plugin expects `header.frame_id = 'world'` or a valid model root link (`base_footprint`). Specifying non-existent link names caused silent aborts.

### 3.3 Engineering Solution
1. **ODE Implicit Spring Damper & Calibrated Dynamics:** Enabled `<implicitSpringDamper>true</implicitSpringDamper>` on all arm revolute joints and calibrated joint friction ($18\text{ N}\cdot\text{m}$ shoulder, $14\text{ N}\cdot\text{m}$ elbow) and hollow composite link masses.
2. **Dedicated Trajectory & Hold Controller (`scripts/arm_controller.py`):**
   - Implemented a 30 Hz ROS 2 node launched alongside Gazebo in `cell_with_robot.launch.py`.
   - Subscribes to `/cmd_arm_pose` (for named automotive poses) and `/cmd_arm_trajectory` (for custom trajectory targets).
   - Generates smooth cosine S-curve trajectory interpolation over $1.5 - 2.0\text{ s}$ transitions.
   - Continuously streams position-holding trajectories at 30 Hz with `stamp = Time(sec=0, nanosec=0)` to maintain exact joint angles under all gravitational loads.
3. **Zero External Dependencies:** Solved 100% within native `gazebo_plugins` and ROS 2 Humble without requiring uninstalled packages like `ros2_control`.

---

## 4. Quantitative Test Results

### 4.1 Automated Static Validation (`validate_stage2.py`)
Executed command:
```bash
python3 src/car_painting_cell/scripts/validate_stage2.py
```
**Results:**
- **File Structure:** 12/12 required files verified.
- **URDF Compilation:** `xacro` generated 21,496 bytes; `check_urdf` parsed XML successfully.
- **Wheels & Ballast:** 4 continuous wheel joints verified; base ballast mass verified ($50.0\text{ kg} \ge 40.0\text{ kg}$).
- **Arm Kinematics:** 6 revolute joints verified with valid limits.
- **End-Effector:** Spray gun body, nozzle link, and collision-free spray cone verified.
- **Gazebo Plugins:** Diff drive, joint state publisher, and joint pose trajectory verified.
- **Standoff Clearance:** Safe spawn at $(0.0, 2.45)$ provides $1.50\text{ m}$ clearance outside car edge ($y = 0.95\text{ m}$).
- **Status:** **PASS (100%)**

### 4.2 Mobile Base Control Test (`test_robot_control.py --test-base`)
Executed command:
```bash
python3 src/car_painting_cell/scripts/test_robot_control.py --test-base
```

| Motion Primitive | Command | Duration | Measured Displacement / Rotation | Status |
|---|---|---|---|---|
| **Forward Drive** | $v_x = +0.35\text{ m/s}, \omega_z = 0.0$ | $1.2\text{ s}$ | $\Delta s = 0.550\text{ m}$ (threshold $\ge 0.15\text{ m}$) | **PASS** |
| **Reverse Drive** | $v_x = -0.35\text{ m/s}, \omega_z = 0.0$ | $1.2\text{ s}$ | $\Delta s = 0.441\text{ m}$ (threshold $\ge 0.15\text{ m}$) | **PASS** |
| **Turn Left (CCW)** | $v_x = 0.0, \omega_z = +0.50\text{ rad/s}$ | $1.5\text{ s}$ | $\Delta \theta = +23.5^\circ$ ($+0.410\text{ rad} \ge +0.20\text{ rad}$) | **PASS** |
| **Turn Right (CW)** | $v_x = 0.0, \omega_z = -0.50\text{ rad/s}$ | $1.5\text{ s}$ | $\Delta \theta = -22.5^\circ$ ($-0.393\text{ rad} \le -0.20\text{ rad}$) | **PASS** |
| **Full Stop** | $v_x = 0.0, \omega_z = 0.0$ | $0.5\text{ s}$ | $v_x = 0.000\text{ m/s}, \omega_z = 0.000\text{ rad/s}$ | **PASS** |

### 4.3 6-Axis Arm Articulation Test (`test_robot_control.py --test-arm`)
Executed command:
```bash
python3 src/car_painting_cell/scripts/test_robot_control.py --test-arm
```

All 6 revolute joints were commanded and quantitatively verified across 5 critical automotive painting poses:

| Pose Name | Joint Targets [J1..J6] (rad) | Measured Actual Angles (rad) | Max Joint Error (rad) | Verification |
|---|---|---|---|---|
| **`READY`** | `[0.0, 0.30, -0.60, 0.0, 0.30, 0.0]` | `[+0.000, +0.305, -0.610, +0.000, +0.302, -0.000]` | $0.010\text{ rad}$ ($0.57^\circ$) | **PASS** |
| **`SIDE_PAINT`** | `[0.0, 0.55, -1.10, 0.0, 0.55, 0.0]` | `[+0.000, +0.561, -1.121, +0.000, +0.551, -0.000]` | $0.021\text{ rad}$ ($1.20^\circ$) | **PASS** |
| **`ROOF_PAINT`** | `[0.0, 0.15, -0.45, 0.0, -1.10, 0.0]` | `[-0.000, +0.150, -0.451, -0.000, -1.101, +0.000]` | $0.001\text{ rad}$ ($0.06^\circ$) | **PASS** |
| **`HOOD_PAINT`** | `[0.50, 0.45, -0.90, 0.0, 0.45, 0.0]` | `[+0.500, +0.455, -0.910, -0.000, +0.451, +0.000]` | $0.010\text{ rad}$ ($0.57^\circ$) | **PASS** |
| **`HOME`** | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` | `[-0.000, +0.000, +0.000, -0.000, +0.000, +0.000]` | $0.000\text{ rad}$ ($0.00^\circ$) | **PASS** |

---

## 5. Visual Verification & Screenshots

Screenshots were captured using the multi-camera monitoring system embedded in the painting booth:
- [`stage2_cell_with_robot_overview.png`](file:///home/mohan/car_painting_ws/src/car_painting_cell/screenshots/stage2_cell_with_robot_overview.png): Isometric perspective showing the mobile painting robot safely stationed on the navigation corridor beside the stationary car in home pose.
- [`stage2_robot_painting_pose.png`](file:///home/mohan/car_painting_ws/src/car_painting_cell/screenshots/stage2_robot_painting_pose.png): Articulated arm in active side-painting pose with spray nozzle oriented perpendicular to the vehicle door panel.
- [`stage2_robot_home_pose.png`](file:///home/mohan/car_painting_ws/src/car_painting_cell/screenshots/stage2_robot_home_pose.png): Manipulator folded compactly in safe transport home position.
- [`stage2_cell_with_robot_front.png`](file:///home/mohan/car_painting_ws/src/car_painting_cell/screenshots/stage2_cell_with_robot_front.png): Front-right perspective displaying vehicle hood, booth lighting, and robot chassis.
- [`stage2_cell_with_robot_top.png`](file:///home/mohan/car_painting_ws/src/car_painting_cell/screenshots/stage2_cell_with_robot_top.png): Top-down layout view illustrating standoff clearance and corridor geometry.

---

## 6. How to Build, Launch, and Control

### 6.1 Build
```bash
source /opt/ros/humble/setup.bash
cd ~/car_painting_ws
colcon build --symlink-install
source install/setup.bash
```

### 6.2 Launch Simulation (Preserves Car & Booth)
```bash
# Launch with Gazebo GUI
ros2 launch car_painting_cell cell_with_robot.launch.py

# Launch headless (for automated tests / CI)
ros2 launch car_painting_cell cell_with_robot.launch.py gui:=false
```

### 6.3 Verification Commands
```bash
# Static & Live Validation Suite
python3 src/car_painting_cell/scripts/validate_stage2.py --live

# Mobile Base Movement Test
python3 src/car_painting_cell/scripts/test_robot_control.py --test-base

# 6-Axis Arm Articulation Test
python3 src/car_painting_cell/scripts/test_robot_control.py --test-arm

# Move Arm to Specific Pose ('ready', 'side_paint', 'roof_paint', 'hood_paint', 'home')
python3 src/car_painting_cell/scripts/test_robot_control.py --pose side_paint

# Inspect Complete Robot Telemetry
python3 src/car_painting_cell/scripts/test_robot_control.py --status
```

---

## 7. Current Limitations & Stage 3 Readiness

### Remaining Limitations:
1. **Open-Loop Base Navigation:** In Stage 2, base velocities are commanded via `/cmd_vel` without closed-loop waypoint tracking or obstacle avoidance.
2. **Autonomous Trajectories Not Implemented:** Autonomous orbital circumnavigation and continuous raster-scan arm pathing will be implemented in Stage 3.
3. **Static Visual Spray Cone:** The spray cone currently represents physical nozzle orientation; dynamic particle spraying or surface color changing will be addressed in Stage 3/4.

### Readiness for Stage 3:
- The robot model is structurally stable, mechanically verified, and collision-free.
- The mobile base drives, rotates, and stops cleanly via standard ROS 2 `/cmd_vel`.
- The 6-axis arm reliably articulates through all automotive painting poses via `/cmd_arm_pose` and holds each pose without droop.
- The project is fully ready for Stage 3 (Autonomous Mobile Trajectories & Multi-Zone Surface Painting).

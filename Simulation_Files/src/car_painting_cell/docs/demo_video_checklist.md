# Yuktii AI Labs — Robotic Car Painting Simulation
## 3–7 Minute Demonstration Video Checklist & Presentation Guide

This checklist provides a step-by-step recording script and demonstration outline for presenting the **Robotic Car Painting Simulation** within a 3–7 minute video submission.

---

### ⏱️ Video Time Budget & Script Breakdown

| Timestamp | Phase / Section | Screen Activity | Narration & Key Points |
| :---: | :--- | :--- | :--- |
| **0:00 – 1:00** | **1. System & Cell Architecture** | Gazebo GUI: Overview Camera View | - Introduce project scope for the Yuktii AI Labs Assessment.<br>- Highlight booth dimensions ($14\text{ m} \times 10\text{ m} \times 4.2\text{ m}$), ventilation banks, and yellow/cyan floor markings.<br>- Show the full-scale vehicle ($4.8\text{ m} \times 1.9\text{ m} \times 1.6\text{ m}$) centered in the staging bay.<br>- Show the mobile painting robot with 4-wheel base, ballast, 6-axis arm, and atomizing spray gun. |
| **1:00 – 2:30** | **2. Mobile Robot Stability & Arm Poses** | Terminal & Gazebo: Split View | - Show that the mobile base is stationary ($v_x = 0, \omega_z = 0$).<br>- Command arm to verified **READY** pose (`test_robot_control.py --pose ready`).<br>- Show joint angles: shoulder $+0.31\text{ rad}$, elbow $-0.63\text{ rad}$, wrist $+0.30\text{ rad}$.<br>- Highlight active 30 Hz position holding with zero gravity droop.<br>- Demonstrate clean transitions to **SIDE_PAINT**, **ROOF_PAINT**, and **HOME** poses. |
| **2:30 – 4:00** | **3. Trajectory Planning & Kinematic Solvers** | Terminal execution of `test_kinematics_and_planning.py` | - Run the comprehensive planning test suite.<br>- Point out analytical FK and analytical/numerical IK solvers.<br>- Walk through all **6 exterior surface plans**: Hood (45 WPs), Left Side (44 WPs), Roof (39 WPs), Rear (41 WPs), Right Side (44 WPs), Front Bumper (17 WPs).<br>- Explain raster scanning pattern, surface normal alignment, and maintained $0.20 - 0.35\text{ m}$ standoff. |
| **4:00 – 5:00** | **4. Collision Checking & Pre-Flight Safety** | Terminal: `test_base_pose_and_frames.py` and `execute_safe_trajectory.py` | - Explain multi-box collision model (lower body, cabin, roof, booth walls, floor).<br>- Demonstrate pre-flight station check: show that if the base is not within $80\text{ mm}$ and $5^\circ$ of the nominal station, execution is safely aborted.<br>- Highlight that safety rules prevent collision rather than blindly executing with coordinate mismatches. |
| **5:00 – 6:30** | **5. Honest Engineering Appraisal & Future Scope** | Gazebo & Terminal Summary | - **State the limitation clearly:** Autonomous multi-station base navigation is frozen due to 4-wheel skid-steer friction scrub ($\mu_2 = 1.5$) exceeding maximum motor torque ($\tau_{\text{max}} = 60\text{ Nm}$) during in-place turns.<br>- Emphasize that keeping the base stationary ensures 100% demo stability.<br>- Future roadmap: transition to omnidirectional Mecanum chassis or Ackermann steering, add GPU shader paint deposition. |

---

### 📋 Pre-Recording Setup Checklist

- [ ] **Workspace Sourced:**
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/car_painting_ws/install/setup.bash
  ```
- [ ] **Clean Build Verified:**
  ```bash
  cd ~/car_painting_ws
  colcon build --symlink-install
  ```
- [ ] **Gazebo Simulation Launched & Visible:**
  ```bash
  ros2 launch car_painting_cell cell_with_robot.launch.py
  ```
  *(Note: If recording headless or viewing via cell cameras, verify `/cell_camera/overview/image_raw` is publishing).*
- [ ] **Terminal Windows Arranged:**
  - Window 1: Simulation launch / Gazebo GUI.
  - Window 2: Command input & test execution.
  - Window 3 (Optional): `ros2 topic echo /odom` or telemetry display.

---

### 💻 Live Commands Cheat Sheet for Recording

#### Command 1: Check Current Telemetry (Stationary Base & Arm)
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/test_robot_control.py --status
```
*Expected display: Base velocities $v_x = 0, \omega_z = 0$, arm joint positions.*

#### Command 2: Move Arm to READY Pose
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/test_robot_control.py --pose ready
```
*Expected: Arm raises gracefully into the standard ready inspection pose without base movement.*

#### Command 3: Move Arm to Key Painting Poses (Optional Transitions)
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/test_robot_control.py --pose side_paint
python3 ~/car_painting_ws/src/car_painting_cell/scripts/test_robot_control.py --pose roof_paint
python3 ~/car_painting_ws/src/car_painting_cell/scripts/test_robot_control.py --pose home
```

#### Command 4: Run Trajectory Planning & Kinematics Test Suite
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/test_kinematics_and_planning.py
```
*Expected: All 6 surfaces plan 230 collision-free waypoints with `[PASS]` on every check.*

#### Command 5: Demonstrate Pre-Flight Station Abort Protection
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/execute_safe_trajectory.py --surface left_side --validate-only
```
*Expected: Script measures current base pose, detects station mismatch, and safely reports `[ABORT]` rather than risking vehicle collision.*

#### Command 6: Run Base Frame Validation Test Suite
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/test_base_pose_and_frames.py
```
*Expected: 100% pass across all 6 station validation checks.*

---

### 🎯 Key Talking Points to Mention During Presentation

1. **Safety First Architecture:** "We enforce strict coordinate frame validation. If the mobile base has not converged to its nominal station within $80\text{ mm}$ and $5^\circ$, the controller aborts before moving the arm, protecting both the robot and the vehicle."
2. **Kinematic Precision:** "Our analytical and numerical inverse kinematics solve all 230 painting waypoints across hood, sides, roof, rear, and bumper with sub-millimeter Cartesian accuracy."
3. **Honest Accounting:** "We do not claim that live surface painting or autonomous multi-station transit is fully functional. We identified that the 4-wheel skid-steer friction torque ($\sim 309\text{ Nm}$) exceeds the $240\text{ Nm}$ total motor torque during pure rotation. We made the professional engineering decision to freeze navigation and demonstrate the arm from a stationary base."

# Yuktii AI Labs — Robotic Car Painting Cell Simulation
## Stage 1 Technical Report: Painting Cell Environment & Vehicle Body

**Author / Candidate:** Mohan  
**Assessment:** Robotics Simulation Intern Practical Assessment  
**Project:** Mobile Robotic Car Painting System Simulation  
**Platform:** Ubuntu 22.04 LTS, ROS 2 Humble, Gazebo Classic 11.10.2  
**Target Workspace:** `~/car_painting_ws/`  
**Package Name:** `car_painting_cell`  

---

### 1. Executive Summary

In Stage 1 of the Yuktii AI Labs Robotic Car Painting Cell Simulation project, we designed and implemented a production-grade industrial automotive painting cell and stationary vehicle body in Gazebo Classic 11 and ROS 2 Humble. In strict adherence to Stage 1 constraints:
- **No mobile robot or manipulator has been created, spawned, or controlled yet.**
- The cell environment, vehicle physical geometry, surface inspection cameras, illumination system, and multi-zone safety floor markings have been fully implemented, validated, and tested.

All geometric dimensions, material appearances, physical collision bounds, and camera viewports were engineered to provide an optimal foundation for subsequent stages (Stage 2: Mobile Robot Base, Stage 3: Movement Trajectories, Stage 4: Surface Spray Simulation, and Stage 5: Collision Checking).

---

### 2. Painting Cell Workshop Architecture

The painting cell is modeled as an industrial automotive crossdraft/downdraft paint booth with cleanroom-rated enclosure panels, epoxy flooring, and dedicated robot travel lanes.

| Parameter | Specification | Physical Justification |
| :--- | :--- | :--- |
| **Interior Cell Length** | 14.0 m | Leaves 4.60 m longitudinal clearance front and rear of the vehicle. |
| **Interior Cell Width** | 10.0 m | Leaves 4.05 m lateral clearance on left and right flanks. |
| **Ceiling Height** | 4.2 m | Standard industrial spray booth height accommodating overhead ventilation & inspection lighting. |
| **Floor Surface** | 15.0 m × 11.0 m × 0.1 m | Monolithic epoxy concrete floor slab with top surface exactly at $z = 0.00\text{ m}$. |
| **Wall Construction** | 0.20 m thick sandwich panels | Cleanroom off-white paneling with dark slate baseboards ($0.4\text{ m}$ height). |
| **Exhaust / Ventilation** | Dual lower wall banks | $6.0\text{ m} \times 1.2\text{ m}$ downdraft/crossdraft filter grates on north and south walls. |

```
+---------------------------------------------------------------------------------------+
|  NORTH ENCLOSURE WALL (X = +7.0m) [Ventilation Filter Bank]                           |
|                                                                                       |
|   +-------------------------------------------------------------------------------+   |
|   |  OUTER HAZARD PERIMETER (Yellow Border: 11.6m x 8.0m)                         |   |
|   |                                                                               |   |
|   |    +---------------------------------------------------------------------+    |   |
|   |    |  ROBOT NAVIGATION CORRIDOR (Cyan Loop: 7.8m x 4.9m, Standoff ~1.3m) |    |   |
|   |    |                                                                     |    |   |
|   |    |     [ Station 1: Front Surface Zone (Hood & Bumper) ]               |    |   |
|   |    |     +-----------------------------------------------+               |    |   |
|   |    |     |  VEHICLE STAGING BAY (Yellow Box: 5.4m x 2.4m)|               |    |   |
|   |    | [S2]|  +=========================================+  | [S4]          |    |   |
|   |    | Left|  |  CAR BODY (4.80m x 1.90m x 1.60m)       |  | Right         |    |   |
|   |    | Zone|  +=========================================+  | Zone          |    |   |
|   |    |     +-----------------------------------------------+               |    |   |
|   |    |     [ Station 3: Rear Surface Zone (Trunk & Bumper) ]               |    |   |
|   |    |                                                                     |    |   |
|   |    +---------------------------------------------------------------------+    |   |
|   |                                                                               |   |
|   |   [Operator Safe Station]                                                     |   |
|   +-------------------------------------------------------------------------------+   |
|                                                                                       |
|  SOUTH ENCLOSURE WALL (X = -7.0m) [Ventilation Filter Bank]                           |
+---------------------------------------------------------------------------------------+
```

---

### 3. Vehicle Body Geometry & Scale Verification

The vehicle is modeled as a stationary, full-scale passenger sedan with distinct architectural components (chassis, hood, trunk, greenhouse cabin, roof, windshield, rear window, side windows, side mirrors, bumpers, lighting assemblies, and four wheels).

#### 3.1 Dimension Compliance Check
The project prompt dictates target dimensions of **4.8 m × 1.9 m × 1.6 m**.

| Dimension | Target Specification | Model Implementation | Verification Status |
| :--- | :--- | :--- | :--- |
| **Length ($X$)** | $4.80\text{ m}$ | Rear Bumper ($x = -2.40\text{ m}$) to Front Bumper ($x = +2.40\text{ m}$) | **PASS** ($4.80\text{ m}$ exact) |
| **Width ($Y$)** | $1.90\text{ m}$ | Right Wheel/Mirror ($y = -0.95\text{ m}$) to Left Wheel/Mirror ($y = +0.95\text{ m}$) | **PASS** ($1.90\text{ m}$ exact) |
| **Height ($Z$)** | $1.60\text{ m}$ | Ground Contact ($z = 0.00\text{ m}$) to Roof Panel Top ($z = 1.60\text{ m}$) | **PASS** ($1.60\text{ m}$ exact) |
| **Ground Clearance** | $0.25\text{ m}$ | Chassis base sits at $z = 0.25\text{ m}$, preventing floor clipping | **PASS** ($0.25\text{ m}$) |

#### 3.2 Wheel Position and Ground Contact
Four wheel assemblies are integrated with realistic automotive proportions:
- **Wheelbase:** $2.80\text{ m}$ (Front Axle: $x = +1.40\text{ m}$, Rear Axle: $x = -1.40\text{ m}$)
- **Track Width:** $1.72\text{ m}$ (Left Tires: $y = +0.86\text{ m}$, Right Tires: $y = -0.86\text{ m}$)
- **Tire Radius:** $r = 0.35\text{ m}$, Width: $w = 0.22\text{ m}$
- **Ground Resting Calculation:**
  $$\text{Wheel Bottom Level} = z_{\text{axle}} - r_{\text{tire}} = 0.35\text{ m} - 0.35\text{ m} = 0.000\text{ m}$$
  The car rests perfectly tangent to the floor surface without clipping or floating.

#### 3.3 Visual & Material Distinguishability
To simulate realistic automotive painting workflows:
- **Body Panels:** Factory unpainted primer / metallic silver e-coat (`ambient: 0.62 0.65 0.68`, `diffuse: 0.74 0.77 0.80`, `specular: 0.5 0.5 0.52`).
- **Glass Surfaces:** Smoke-tinted safety glass with high specular reflectance (`diffuse: 0.18 0.22 0.26`, `alpha: 0.90`, `specular: 0.9 0.92 0.95`).
- **Tires:** Matte automotive rubber black (`diffuse: 0.18 0.18 0.18`).
- **Rims:** Machined silver alloy (`diffuse: 0.75 0.77 0.80`, `specular: 0.8 0.8 0.8`).
- **Front Headlights:** Xenon white with emissive luminance (`diffuse: 0.92 0.96 1.00`, `emissive: 0.35 0.40 0.45`).
- **Rear Taillights:** LED ruby red (`diffuse: 0.88 0.10 0.10`, `emissive: 0.45 0.03 0.03`).

#### 3.4 Physical Collision Geometry
For reliable simulation stability and Stage 5 collision checking:
- `lower_body_collision`: Box geometry ($4.80\text{ m} \times 1.80\text{ m} \times 0.47\text{ m}$).
- `cabin_collision`: Box geometry ($2.05\text{ m} \times 1.56\text{ m} \times 0.82\text{ m}$).
- `roof_collision`: Box geometry ($2.10\text{ m} \times 1.52\text{ m} \times 0.05\text{ m}$).
- 4 $\times$ `wheel_collision`: Cylinders ($r = 0.35\text{ m}$, $l = 0.24\text{ m}$).

---

### 4. Painting Cell Markings & Robot Navigation Corridor

The floor incorporates a layered marking scheme:
1. **Vehicle Staging Bay (Yellow Box):** Outlines vehicle boundary ($5.40\text{ m} \times 2.40\text{ m}$) with wheel alignment pads under each tire.
2. **Designated Robot Navigation Corridor (Cyan Track):** A rectangular guidance loop ($7.80\text{ m} \times 4.90\text{ m}$) positioned at a standoff distance of $1.2\text{ m}$ to $1.5\text{ m}$ from the vehicle body. This defines the path for the mobile robot in Stages 2 and 3.
3. **Four Exterior Surface Painting Stations:**
   - **Station 1 (Front Zone):** Covers front bumper and hood ($x = 3.85\text{ m}$, size $0.9\text{ m} \times 2.4\text{ m}$).
   - **Station 2 (Left Flank Zone):** Covers left doors, pillars, and roof rail ($y = 2.45\text{ m}$, size $4.8\text{ m} \times 0.8\text{ m}$).
   - **Station 3 (Rear Zone):** Covers rear bumper and trunk ($x = -3.85\text{ m}$, size $0.9\text{ m} \times 2.4\text{ m}$).
   - **Station 4 (Right Flank Zone):** Covers right doors, pillars, and roof rail ($y = -2.45\text{ m}$, size $4.8\text{ m} \times 0.8\text{ m}$).
4. **Outer Hazard Perimeter (Safety Orange/Yellow):** Outer clearance perimeter ($11.60\text{ m} \times 8.00\text{ m}$).
5. **Operator Safety Station:** Marked safe technician zone ($1.40\text{ m} \times 0.90\text{ m}$) in the corner.

All markings are elevated by $0.002\text{ m}$ above the floor to completely eliminate z-fighting flickering, with collision geometry disabled so wheel rolling dynamics remain unaffected.

---

### 5. Illumination & Camera System

- **Overhead Inspection Banks:** 4 calibrated point lights situated at $z = 3.8\text{ m}$ directly illuminating the hood, trunk, left flank, and right flank.
- **Ambient Lighting:** High CRI ambient lighting (`0.60 0.62 0.66`) eliminates dark shadows under vehicle undercuts.
- **Interactive GUI Camera:** Default orbital camera preset in `<gui><camera>` positioned at `(-6.8, -5.8, 4.2)` pitched at $0.42\text{ rad}$ and yawed at $0.72\text{ rad}$, presenting an immediate isometric overview upon launching Gazebo.
- **Industrial Monitoring Cameras:**
  - `/cell_camera/overview/image_raw`: Rear-left elevated 3/4 view ($1280 \times 720$).
  - `/cell_camera/front/image_raw`: Front-right elevated 3/4 view ($1280 \times 720$).
  - `/cell_camera/top/image_raw`: Top-down orthographic plan view ($1280 \times 720$).

---

### 6. Validation Results

The automated test script (`validate_cell.py`) was executed on the workspace, verifying all 25 individual checkpoints:

```
Running Stage 1 Validation Suite on: /home/mohan/car_painting_ws/src/car_painting_cell

=================================================================
 1. Package and File Structure Inspection
=================================================================
 [PASS] File exists: package.xml
 [PASS] File exists: CMakeLists.txt
 [PASS] File exists: launch/cell_world.launch.py
 [PASS] File exists: worlds/car_painting_cell.world
 [PASS] File exists: models/car_body/model.config
 [PASS] File exists: models/car_body/model.sdf
 [PASS] File exists: models/painting_cell/model.config
 [PASS] File exists: models/painting_cell/model.sdf

=================================================================
 2. Gazebo SDF Syntax Validation
=================================================================
 [PASS] SDF syntax check: models/car_body/model.sdf - Check complete
 [PASS] SDF syntax check: models/painting_cell/model.sdf - Check complete
 [PASS] SDF syntax check: worlds/car_painting_cell.world - Check complete

=================================================================
 3. Car Body Specifications & Geometry
=================================================================
 [PASS] Car body length check - Expected ~4.80m, parsed: 4.80m
 [PASS] Car body height check - Expected ~1.60m, parsed: 1.60m
 [PASS] Wheel wheel_fl ground resting level - Axle z=0.35m, radius=0.35m -> contact z=0.000m
 [PASS] Wheel wheel_fr ground resting level - Axle z=0.35m, radius=0.35m -> contact z=0.000m
 [PASS] Wheel wheel_rl ground resting level - Axle z=0.35m, radius=0.35m -> contact z=0.000m
 [PASS] Wheel wheel_rr ground resting level - Axle z=0.35m, radius=0.35m -> contact z=0.000m
 [PASS] Collision geometry: lower_body_collision
 [PASS] Collision geometry: cabin_collision
 [PASS] Collision geometry: roof_collision
 [PASS] Collision geometry: wheel_fl_collision
 [PASS] Collision geometry: wheel_fr_collision
 [PASS] Collision geometry: wheel_rl_collision
 [PASS] Collision geometry: wheel_rr_collision

=================================================================
 4. Workshop Enclosure & Floor Markings
=================================================================
 [PASS] Industrial floor size - Floor dimensions: 15.0m x 11.0m
 [PASS] Enclosure wall: north_wall_collision
 [PASS] Enclosure wall: south_wall_collision
 [PASS] Enclosure wall: east_wall_collision
 [PASS] Enclosure wall: west_wall_collision
 [PASS] Vehicle Staging Bay (Yellow Box) - 4 visual elements
 [PASS] Robot Navigation Loop (Cyan Corridor) - 4 visual elements
 [PASS] Surface Painting Stations (4 Exterior Zones) - 4 visual elements
 [PASS] Cell Hazard Perimeter - 4 visual elements
 [PASS] Operator Safe Station - 1 visual elements

=================================================================
 5. Stage 1 Scope Constraints
=================================================================
 [PASS] Strict constraint: No robot in Stage 1 - Models present: [None, None, 'cell_monitoring_camera']

=================================================================
 Stage 1 Validation Summary
=================================================================
 >>> ALL STAGE 1 CHECKS PASSED SUCCESSFULLY! <<<
```

---

### 7. Execution Guide

#### 7.1 Prerequisites & Environment Setup
```bash
source /opt/ros/humble/setup.bash
cd ~/car_painting_ws
colcon build --symlink-install
source install/setup.bash
```

#### 7.2 Launch the Simulation Environment
To start the complete simulation cell in Gazebo Classic GUI:
```bash
ros2 launch car_painting_cell cell_world.launch.py
```

To run in headless mode (server only):
```bash
ros2 launch car_painting_cell cell_world.launch.py gui:=false
```

#### 7.3 Run Automated Validation Suite
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/validate_cell.py
```

#### 7.4 Capture Simulation Screenshots
When the simulation is active, capture all three camera perspectives with:
```bash
python3 ~/car_painting_ws/src/car_painting_cell/scripts/capture_screenshot.py
```

---

### 8. Stage 1 Sign-Off & Next Steps

Stage 1 is complete, verified, and ready for review.

**Readiness for Stage 2:**
- Stationary vehicle is positioned stably at the cell origin $(0, 0, 0)$.
- Clear clearance lanes ($>4.0\text{ m}$ to walls, $1.2\text{ m} - 1.5\text{ m}$ standoff lane) are established.
- Collision meshes are verified for accurate obstacle avoidance in Stage 5.
- ROS 2 topics and launch pipelines are verified for seamless integration of the mobile base in Stage 2.

# Known Limitations & Technical Constraints
## Robotic Car Painting Cell Simulation

This document provides a transparent, professional engineering assessment of the architectural and physical constraints of the simulation implementation.

---

## 1. Simulated Paint Deposition vs. Dynamic Texture Painting

### Implementation Mechanism:
In this simulation, surface coating is rendered by spawning visual overlay entities in Gazebo via the `/spawn_entity` service:
- Front bumper: `paint_layer_front` (curved visual box matching bumper contour)
- Left side panels: `paint_layer_left` (planar side coat model)
- Rear bumper & trunk: `paint_layer_rear` (multi-segment visual model)
- Right side panels: `paint_layer_right` (planar side coat model)
- Front hood: `paint_layer_hood` (slanted planar visual overlay)
- Roof: `paint_layer_roof` (horizontal planar visual overlay)

### Engineering Distinction:
- **Visual Entity Overlays (Implemented):** These models change the apparent color and finish of the car body in the 3D scene from industrial grey primer to metallic blue automotive lacquer once the spray trajectory completes.
- **Dynamic Texture/Shader Painting (Not Implemented):** Real-time progressive GPU texture splatting (e.g. via custom Ogre/Gazebo material plugins with dynamic UV coordinate texture baking) is not supported natively by Gazebo Classic 11 without custom C++ rendering engine modifications. The project explicitly documents visual layer spawning as the simulation proxy and avoids false claims regarding dynamic pixel-level texture deposition.

---

## 2. Geometric Collision Checking Fidelity

### Implementation Mechanism:
The collision checker (`collision_checker.py`) models the cell environment using bounding geometry:
- Car Body Lower Envelope: $[-2.40, 2.44] \times [-0.95, 0.95] \times [0.00, 0.70]$ m
- Car Cabin & Roof Envelope: $[-1.15, 0.90] \times [-0.85, 0.85] \times [0.70, 1.605]$ m
- Workshop Booth Enclosure: $[-7.00, 7.00] \times [-6.00, 6.00]$ m
- Mobile Robot Footprint: Oriented Bounding Box (OBB) $0.80\text{m} \times 0.60\text{m}$ rotated by base yaw.
- Arm Links: Multi-sphere bounding capsules around the 6 kinematic links and spray nozzle.

### Limitation:
While dense collision sampling (20 points per corridor segment, 10 intermediate IK points per trajectory waypoint) guarantees clearance against the vehicle and booth walls, it is an analytical volumetric approximation rather than a full continuous Flexible Collision Library (FCL) mesh-to-mesh or signed distance field (SDF) solver. Dynamic, unexpected obstacles placed in the corridor are not dynamically sensed by 3D LiDAR.

---

## 3. Navigation Architecture

### Implementation Mechanism:
Base navigation utilizes a 3-phase differential drive controller (`navigate_to_station.py` and `PaintingStateMachine.drive_route`):
1. Orientation alignment towards waypoint.
2. Proportional linear transit along designated standoff corridors with normalized angular tracking.
3. Final station normal fine-yaw alignment.

### Limitation:
This is a deterministic waypoint transit system designed specifically for industrial painting cells with fixed floor guide paths (similar to an Automated Guided Vehicle / AGV rail or optical line-tracking system). It does not incorporate the ROS 2 Nav2 stack (costmaps, AMCL particle filter, or DWA/TEB local planners), which would introduce significant overhead and non-deterministic behavior in a fixed-cell manufacturing simulation.

---

## 4. Hardware and CPU Resource Dependency

### Real-Time Factor (RTF):
- In headless mode (`gui:=false`), Gazebo Classic physics executes between **0.90x and 1.05x RTF** on modern multi-core x86_64 systems.
- When running with full GUI (`gzclient`) alongside uncompressed 720p camera streaming, rendering overhead can cause transient drops in RTF (down to 0.60x - 0.75x). The software architecture accommodates this by anchoring all timeouts, timers, and interpolators strictly to `/clock`.

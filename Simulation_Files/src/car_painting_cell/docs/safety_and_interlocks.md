# Safety Interlocks & Failure Protection Architecture
## Robotic Car Painting Cell Simulation

This document specifies the fail-safe interlocks, transport verification protocols, and emergency halt mechanisms enforced across the simulation architecture.

---

## 1. Safety Interlock Overview

The painting cell operates under strict industrial safety invariants to ensure that robot motion never risks collision with the vehicle body, enclosure walls, or personnel fixtures.

```
       +---------------------------------------------+
       |   Pre-Motion Safety Interlock Evaluation    |
       +---------------------------------------------+
                              |
       +----------------------v----------------------+
       | 1. Is Joint Feedback Present & Fresh?       |
       |    (age <= 0.5s, all 6 arm joints defined)  |
       +----------------------|----------------------+
                              | YES
       +----------------------v----------------------+
       | 2. Is Arm in Verified Transport Pose?       |
       |    (|q_i - q_t| <= 0.04 rad, ||e|| <= 0.08) |
       +----------------------|----------------------+
                              | YES
       +----------------------v----------------------+
       | 3. Is Controller Status Fresh & 'HOLDING'?  |
       |    (age <= 1.0s, status == 'HOLDING')       |
       +----------------------|----------------------+
                              | YES
       +----------------------v----------------------+
       |       ALLOW BASE VELOCITY COMMANDS          |
       |            (Spray FORCED OFF)               |
       +---------------------------------------------+
                              |
            ANY CONDITION FAILS / TIMEOUT
                              |
       +----------------------v----------------------+
       |     EMERGENCY SAFETY STOP & LOCKOUT         |
       |   - Publish zero-velocity (/cmd_vel = 0)    |
       |   - Force Spray OFF (/spray_active = false) |
       |   - Prohibit Navigation & Lock Motion       |
       +---------------------------------------------+
```

---

## 2. The 7 Primary Safety Interlocks

| Interlock ID | Safety Rule Description | Mathematical / Logic Threshold | Safety Response on Violation |
| :---: | :--- | :--- | :--- |
| **INT-01** | **Joint Feedback Freshness** | $\Delta t_{\text{feedback}} = t_{\text{clock}} - t_{\text{joint\_state}} \le 0.50\text{s}$ | Immediate base zero-velocity halt; Motion prohibited; Error logged. |
| **INT-02** | **Full Joint Chain Completeness** | $\forall j \in \{\text{arm\_base, shoulder, elbow, wrist\_1, wrist\_2, wrist\_3}\} \subset \text{joint\_state.name}$ | Refuse motion; Prohibit trajectory dispatch. |
| **INT-03** | **Transport Pose Tolerances** | $\max_i \|q_i - q_{\text{transport}, i}\| \le 0.04\text{ rad}$, $\|\mathbf{e}\|_2 \le 0.08\text{ rad}$ | Base movement strictly locked until arm is safely folded in transport pose. |
| **INT-04** | **Arm Controller Status Check** | $\text{status} \in \{\text{'HOLDING', 'HOLD', 'IDLE', 'TRANSPORT\_HOLD'}\} \land \Delta t_{\text{status}} \le 1.0\text{s}$ | Base movement prohibited if arm is interpolating a trajectory or status is stale. |
| **INT-05** | **Spray Navigation Interlock** | During any base movement ($\|\mathbf{v}\| > 0 \lor \|\omega\| > 0$), `/spray_active` is forced `False`. | Physical spray cone suppressed; Paint deposition blocked. |
| **INT-06** | **Station Positioning Tolerance** | $\Delta p_{\text{station}} \le 0.120\text{m} \land \|\Delta\psi_{\text{station}}\| \le 6.0^\circ$ | Arm painting trajectory is rejected if base fails to dock within tolerance. |
| **INT-07** | **Post-Painting Return Safety** | After painting, arm must return to transport pose and verify holding before return-route transit is permitted. | If arm cannot achieve transport pose, return route is aborted and robot stays stationary at station. |
| **INT-08** | **Arm Velocity Stability Guard** | $\dot{q}_{\text{shoulder}} \le 0.3000\text{ rad/s}, \dot{q}_{\text{elbow}} \le 0.5000\text{ rad/s}, \|\mathbf{\dot{q}}\|_2 \le 0.5500\text{ rad/s}$ | Base movement immediately prohibited if arm links exhibit unverified dynamic oscillations during transit. |

---

## 3. Mathematical Verification of Transport Tolerances

The transport pose tucks the 6-DOF arm over the mobile base chassis to minimize the swept collision radius:
- `transport` configuration: $[0.0, -1.20, 2.20, -1.00, 0.0, 0.0]$ radians.

```python
transport_q = [0.0, -1.20, 2.20, -1.00, 0.0, 0.0]
err_vec = [abs(measured[i] - transport_q[i]) for i in range(6)]
assert all(e <= 0.04 for e in err_vec), "Per-joint tolerance exceeded"
assert np.linalg.norm(err_vec) <= 0.08, "Vector norm tolerance exceeded"
```

This ensures that gravity sag, inertia, or controller drift cannot allow the painting arm to protrude outside the safety envelope during corridor navigation.

---

## 4. Spray Deactivation Guarantee

Spray deactivation is enforced at three distinct layers:
1. **Application Layer (`PaintingStateMachine`):** Explicitly calls `self.set_spray(False)` before any base movement, upon station departure, during arm retractions, and in all exception/abort handlers.
2. **Controller Layer (`arm_controller.py`):** Shuts off `/spray_status` whenever the arm transitions out of a valid painting sweep or when `/spray_active` is False.
3. **Dry-Run Mode Guard:** In dry-run mode, all spray publishers and entity spawning clients are intercepted and bypassed, guaranteeing zero simulated aerosol or entity creation.

---

## 5. Bounded Base Braking & Inertial Pitch Stabilization

To eliminate transient inertia-induced angular velocity spikes across the folded arm links during corridor navigation:
1. **Ramped Deceleration (`ramp_stop_base`):** Whenever the mobile base transitions between waypoints or prepares for yaw rotation, commanded velocities are smoothly ramped down using bounded deceleration ($\le 0.30\text{ m/s}^2$ linear, $\le 0.40\text{ rad/s}^2$ angular).
2. **Chassis Pitch Settle Delay:** Following velocity ramp-down, the base commands zero velocity for an additional $0.35\text{ s}$ settle period. This allows the chassis suspension and tire-ground contact dynamics in Gazebo ODE to stabilize completely, ensuring arm joint velocities remain strictly within the $\le 0.3000\text{ rad/s}$ transport safety threshold before any subsequent motion or trajectory planning.


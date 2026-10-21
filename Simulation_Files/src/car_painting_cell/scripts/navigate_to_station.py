#!/usr/bin/env python3
"""
Controlled Mobile-Base Positioning Node for Stage 4 Cell Stations.
Navigates the differential-drive mobile base to a designated cell station:
- Uses closed-loop feedback from /odom.
- Employs a 3-phase controller (Align to waypoint -> Drive to waypoint -> Align to final yaw).
- Enforces normalized angular error in [-pi, pi].
- Performs pre-navigation route safety check against car envelope and booth boundaries.
- Halts with zero-velocity command upon completion or timeout/failure.
- Verifies position error <= 0.03m (30mm) and yaw error <= 2.0 degrees.
"""

import time
import math
import sys
import os
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from collision_checker import CollisionChecker, point_to_box_distance, CAR_LOWER_MIN, CAR_LOWER_MAX, BOOTH_MIN, BOOTH_MAX
from trajectory_planner import PaintingTrajectoryPlanner, validate_base_station


def normalize_angle(angle):
    """Normalizes angle to [-pi, pi] radians."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(q):
    """Extracts yaw (rad) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StationNavigator(Node):
    def __init__(self):
        super().__init__('station_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_pose = None
        self.current_vel = None
        self.collision_checker = CollisionChecker()

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)
        self.current_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def stop_robot(self):
        """Sends multiple zero-velocity commands to ensure full stop."""
        zero_twist = Twist()
        for _ in range(5):
            self.cmd_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def wait_for_telemetry(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pose is not None:
                return True
        return False

    def verify_route_safety(self, start_pose, target_pose, num_checks=20):
        """Verifies that the linear transit between start and target maintains clearance."""
        sx, sy, syaw = start_pose
        tx, ty, tyaw = target_pose

        for i in range(num_checks + 1):
            alpha = i / float(num_checks)
            x = sx + alpha * (tx - sx)
            y = sy + alpha * (ty - sy)
            yaw = syaw + alpha * normalize_angle(tyaw - syaw)
            ok, msg = self.collision_checker.check_base_pose((x, y, yaw))
            if not ok:
                return False, f"Route collision check failed at ({x:.2f}, {y:.2f}): {msg}"

        return True, "Route clear"

    def navigate_to(self, target_x, target_y, target_yaw, pos_tol=0.03, yaw_tol_deg=2.0, timeout=35.0):
        """
        Executes controlled, closed-loop navigation to (target_x, target_y, target_yaw).
        Uses a 2-stage corridor approach:
        1. Transit to corridor entry waypoint behind the station.
        2. Aligned corridor approach down the station normal line to target pose.
        Halts with zero-velocity on all paths and verifies pos_err <= pos_tol and yaw_err <= yaw_tol.
        Returns: (success: bool, final_pos_err: float, final_yaw_err: float, msg: str)
        """
        yaw_tol_rad = math.radians(yaw_tol_deg)
        t0 = time.time()
        twist = Twist()

        print(f"\nTarget Station: X={target_x:.4f}m, Y={target_y:.4f}m, Yaw={math.degrees(target_yaw):.2f}deg")
        print(f"Tolerances:     pos <= {pos_tol*1000.0:.1f}mm, yaw <= {yaw_tol_deg:.1f}deg (timeout={timeout:.1f}s)")

        # Corridor entry waypoint 0.25m behind station along its approach line
        entry_x = target_x - 0.25 * math.cos(target_yaw)
        entry_y = target_y - 0.25 * math.sin(target_yaw)

        # Determine initial stage
        curr_x, curr_y, curr_yaw = self.current_pose
        dx_line = curr_x - target_x
        dy_line = curr_y - target_y

        # If already near the corridor line and behind station, start in approach mode
        if abs(dx_line) <= 0.04 and dy_line >= 0.02:
            state = "CORRIDOR_APPROACH"
        else:
            state = "TRANSIT_TO_ENTRY"

        print(f"Initial State:  {state} (entry waypoint: X={entry_x:.3f}, Y={entry_y:.3f})")

        settle_counter = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            elapsed = time.time() - t0

            if elapsed > timeout:
                self.stop_robot()
                curr = self.current_pose
                pos_err = math.hypot(target_x - curr[0], target_y - curr[1])
                yaw_err = abs(normalize_angle(target_yaw - curr[2]))
                return False, pos_err, yaw_err, f"Navigation timed out after {elapsed:.1f}s (pos_err={pos_err*1000:.1f}mm, yaw_err={math.degrees(yaw_err):.2f}deg)"

            curr_x, curr_y, curr_yaw = self.current_pose
            pos_err = math.hypot(target_x - curr_x, target_y - curr_y)
            yaw_err = abs(normalize_angle(target_yaw - curr_yaw))

            # Periodic telemetry log (every ~1 sec)
            if int(elapsed * 10) % 20 == 0:
                print(f"  [t={elapsed:4.1f}s] [{state:17s}] pos=({curr_x:6.3f}, {curr_y:6.3f}) pos_err={pos_err*1000:5.1f}mm, yaw={math.degrees(curr_yaw):6.1f}deg (yaw_err={math.degrees(yaw_err):5.1f}deg)")

            if state == "TRANSIT_TO_ENTRY":
                dx_entry = entry_x - curr_x
                dy_entry = entry_y - curr_y
                d_entry = math.hypot(dx_entry, dy_entry)

                # Smoothly transition to corridor approach when intercepting the entry corridor
                if d_entry <= 0.08 or (abs(curr_x - target_x) <= 0.05 and curr_y >= target_y + 0.15):
                    state = "CORRIDOR_APPROACH"
                    print(f"  [t={elapsed:4.1f}s] Corridor entry intercepted. Switching to CORRIDOR_APPROACH.")
                else:
                    phi = math.atan2(dy_entry, dx_entry)
                    e_head = normalize_angle(phi - curr_yaw)

                    if abs(e_head) > math.radians(20.0):
                        # Roll slowly while turning to prevent skid-steer static friction stall
                        v = 0.035
                        w = math.copysign(max(0.15, min(0.35, 1.5 * abs(e_head))), e_head)
                    else:
                        v = max(0.04, min(0.12, 0.6 * d_entry))
                        w = math.copysign(min(0.25, 1.2 * abs(e_head)), e_head)

                    twist.linear.x = float(v)
                    twist.angular.z = float(w)

            elif state == "CORRIDOR_APPROACH":
                dx_target = curr_x - target_x
                dy_target = curr_y - target_y
                e_yaw = normalize_angle(target_yaw - curr_yaw)

                if pos_err <= 0.025:
                    # In fine target zone: hold position and perform micro-alignment of yaw
                    twist.linear.x = 0.0
                    if abs(e_yaw) <= math.radians(1.8):
                        twist.angular.z = 0.0
                        settle_counter += 1
                        if settle_counter >= 15:  # ~300ms stable within tolerances
                            print(f"  Station converged and settled! (pos_err={pos_err*1000:.1f}mm, yaw_err={math.degrees(e_yaw):.2f}deg)")
                            break
                    else:
                        settle_counter = 0
                        w = math.copysign(max(0.08, min(0.22, 1.2 * abs(e_yaw))), e_yaw)
                        twist.angular.z = float(w)
                else:
                    settle_counter = 0
                    # Guide heading toward the approach centerline x = target_x
                    des_th = target_yaw - math.atan2(dx_target, 0.30)
                    des_th = max(target_yaw - math.radians(35.0), min(target_yaw + math.radians(35.0), des_th))
                    e_steer = normalize_angle(des_th - curr_yaw)

                    if dy_target < -0.015:
                        # Overshot south of station: reverse gently
                        v = -0.04
                        w = math.copysign(min(0.18, 1.0 * abs(e_steer)), e_steer)
                    else:
                        # Forward approach down corridor
                        v = max(0.03, min(0.10, 0.5 * pos_err))
                        w = math.copysign(min(0.25, 1.2 * abs(e_steer)), e_steer)

                    twist.linear.x = float(v)
                    twist.angular.z = float(w)

            self.cmd_pub.publish(twist)
            time.sleep(0.02)

        # Full stop on exit
        self.stop_robot()
        time.sleep(0.3)
        rclpy.spin_once(self, timeout_sec=0.1)

        # Measure settled final pose and verify
        final_x, final_y, final_yaw = self.current_pose
        final_pos_err = math.hypot(target_x - final_x, target_y - final_y)
        final_yaw_err = abs(normalize_angle(target_yaw - final_yaw))

        pos_ok = final_pos_err <= pos_tol
        yaw_ok = final_yaw_err <= yaw_tol_rad
        success = pos_ok and yaw_ok

        msg = f"Final Pose: X={final_x:.4f}m, Y={final_y:.4f}m, Yaw={math.degrees(final_yaw):.2f}deg | Err: pos={final_pos_err*1000.0:.1f}mm, yaw={math.degrees(final_yaw_err):.2f}deg"
        return success, final_pos_err, final_yaw_err, msg



def main():
    parser = argparse.ArgumentParser(description="Controlled Mobile-Base Station Navigator")
    parser.add_argument('--target-x', type=float, default=0.0,
                        help='Target X coordinate (meters, default 0.0)')
    parser.add_argument('--target-y', type=float, default=2.00,
                        help='Target Y coordinate (meters, default 2.00)')
    parser.add_argument('--target-yaw', type=float, default=-90.0,
                        help='Target Yaw angle (degrees, default -90.0)')
    parser.add_argument('--station', type=str, default='left_side',
                        help='Predefined station name: left_side, hood, roof, rear, right_side, front_bumper')
    parser.add_argument('--pos-tol', type=float, default=0.03,
                        help='Position error tolerance in meters (default 0.03m = 30mm)')
    parser.add_argument('--yaw-tol', type=float, default=2.0,
                        help='Yaw error tolerance in degrees (default 2.0 deg)')
    parser.add_argument('--timeout', type=float, default=35.0,
                        help='Max navigation timeout in seconds (default 35.0s)')
    args = parser.parse_args()

    rclpy.init()
    node = StationNavigator()

    print("=" * 70)
    print(" CONTROLLED MOBILE BASE STATION POSITIONING")
    print("=" * 70)

    # Resolve target from station name if provided
    planner = PaintingTrajectoryPlanner()
    target_station = None
    if args.station:
        try:
            plan = planner.generate_surface_plan(args.station)
            target_station = plan.base_station
            target_x, target_y, target_yaw = target_station
            print(f"Station '{args.station}' resolved to: ({target_x:.3f}, {target_y:.3f}, {math.degrees(target_yaw):.1f} deg)")
        except ValueError:
            target_x = args.target_x
            target_y = args.target_y
            target_yaw = math.radians(args.target_yaw)
    else:
        target_x = args.target_x
        target_y = args.target_y
        target_yaw = math.radians(args.target_yaw)

    # 1. Wait for telemetry
    print("Waiting for /odom telemetry...")
    if not node.wait_for_telemetry():
        print("[FAIL] Timeout waiting for /odom telemetry. Ensure Gazebo is running!")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    start_pose = node.current_pose
    print(f"Initial Base Pose: X={start_pose[0]:.4f}m, Y={start_pose[1]:.4f}m, Yaw={math.degrees(start_pose[2]):.2f}deg")

    # 2. Check route safety before moving
    safe, reason = node.verify_route_safety(start_pose, (target_x, target_y, target_yaw))
    print(f"Route Safety Check: {'PASSED (' + reason + ')' if safe else 'FAILED: ' + reason}")
    if not safe:
        print("[ABORT] Cannot proceed: route violates cell safety clearance!")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 3. Check if already at station
    is_at_station, curr_pos_err, curr_yaw_err, _ = validate_base_station(
        start_pose, (target_x, target_y, target_yaw), pos_tol=args.pos_tol, yaw_tol=math.radians(args.yaw_tol)
    )
    if is_at_station:
        print(f"\n[INFO] Robot is already at target station! (pos_err={curr_pos_err*1000:.1f}mm, yaw_err={math.degrees(curr_yaw_err):.2f}deg)")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # 4. Execute controlled closed-loop navigation
    print("\n--- Initiating Closed-Loop Motion ---")
    try:
        success, final_pos_err, final_yaw_err, report = node.navigate_to(
            target_x, target_y, target_yaw,
            pos_tol=args.pos_tol, yaw_tol_deg=args.yaw_tol, timeout=args.timeout
        )
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Stopping robot immediately...")
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 5. Report results
    print("\n" + "=" * 70)
    print(" STATION NAVIGATION RESULT")
    print("=" * 70)
    print(f" Status:         {'SUCCESS' if success else 'FAILURE'}")
    print(f" Target Station: X={target_x:.4f}m, Y={target_y:.4f}m, Yaw={math.degrees(target_yaw):.2f}deg")
    print(f" Reached Pose:   X={node.current_pose[0]:.4f}m, Y={node.current_pose[1]:.4f}m, Yaw={math.degrees(node.current_pose[2]):.2f}deg")
    print(f" Position Error: {final_pos_err*1000.0:.2f} mm (limit: <= {args.pos_tol*1000.0:.1f} mm)")
    print(f" Yaw Error:      {math.degrees(final_yaw_err):.2f} deg (limit: <= {args.yaw_tol:.1f} deg)")
    print(f" Final Velocity: vx={node.current_vel[0]:.4f}m/s, wz={node.current_vel[1]:.4f}rad/s")
    print("=" * 70)

    node.destroy_node()
    rclpy.shutdown()

    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()

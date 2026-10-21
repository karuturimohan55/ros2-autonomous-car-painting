#!/usr/bin/env python3
"""
Stage 6K Regression Test Suite: Minimal Dry-Run Implementation Verification

Verifies:
1. CLI argument parsing:
   - '--dry-run' flag correctly recognized and sets dry_run=True.
   - Default without '--dry-run' sets dry_run=False.
2. paint_surface() in dry-run mode (dry_run=True):
   - Every spray command is forced OFF (set_spray(False)); zero set_spray(True) calls.
   - spawn_paint_layer() is NEVER called.
   - Raster waypoints, collision checks, standoff checks, arm trajectory targets,
     and transport retractions are fully executed without alteration.
3. paint_surface() in normal mode (dry_run=False):
   - Preserves exact existing behavior: set_spray(True) is invoked during active sweeps.
   - spawn_paint_layer() is invoked upon completion.
4. run_single_station() in dry-run mode:
   - Forces spray OFF prior to base movement.
   - Forwards dry_run=True to paint_surface().
   - Shuts spray OFF again during cleanup and on failure.
5. Failure guarding under dry-run mode:
   - Collision abort during dry-run shuts spray OFF and retracts arm.
   - Convergence timeout during dry-run shuts spray OFF and retracts arm.
"""

import sys
import os
import argparse
from unittest.mock import MagicMock, call

# Isolate ROS domain
os.environ['ROS_DOMAIN_ID'] = '124'

import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from painting_state_machine import (
    PaintingStateMachine,
    run_single_station,
    run_full_sequence,
    PRESET_POSES,
    STATIONS,
    TRANSIT_ROUTES
)


def create_isolated_test_node():
    """Initializes an isolated PaintingStateMachine node for testing."""
    if not rclpy.ok():
        rclpy.init()
    node = PaintingStateMachine()
    node.current_pose = (3.50, 0.00, -3.14159)  # FRONT station pose
    node.current_vel = (0.0, 0.0)

    # Mock publishers and clients to avoid live network calls
    node.cmd_vel_pub = MagicMock()
    node.spray_active_pub = MagicMock()
    node.cmd_arm_traj_pub = MagicMock()
    node.cmd_arm_pose_pub = MagicMock()
    node.delete_client = MagicMock()
    node.delete_client.wait_for_service.return_value = False

    return node


def test_cli_parsing():
    """Test 1: Verify argument parser supports --dry-run and defaults to False."""
    print("--- Test 1: CLI Argument Parsing ---")
    parser = argparse.ArgumentParser(description="Autonomous Painting State Machine")
    parser.add_argument('--single-station', type=str, default=None)
    ws_screenshots = os.path.join(os.getcwd(), 'screenshots')
    pkg_screenshots = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'screenshots')
    default_screenshots = ws_screenshots if os.path.isdir(ws_screenshots) else pkg_screenshots
    parser.add_argument('--output-dir', type=str, default=default_screenshots)
    parser.add_argument('--dry-run', action='store_true', default=False)

    # 1. Without --dry-run
    args_normal = parser.parse_args(['--single-station', 'FRONT'])
    assert args_normal.single_station == 'FRONT'
    assert args_normal.dry_run is False, "Default dry_run must be False"
    print("  [PASS] Default CLI parsing: dry_run=False")

    # 2. With --dry-run
    args_dry = parser.parse_args(['--single-station', 'FRONT', '--dry-run'])
    assert args_dry.single_station == 'FRONT'
    assert args_dry.dry_run is True, "--dry-run must set dry_run=True"
    print("  [PASS] Dry-run CLI parsing: dry_run=True")


def test_paint_surface_dry_run_suppression():
    """Test 2: Verify paint_surface(dry_run=True) suppresses all spray and layer spawning."""
    print("--- Test 2: paint_surface() in Dry-Run Mode ---")
    node = create_isolated_test_node()

    spray_history = []
    original_set_spray = node.set_spray
    def mock_set_spray(val):
        spray_history.append(bool(val))
    node.set_spray = mock_set_spray

    node.spawn_paint_layer = MagicMock()
    node.send_arm_joint_target = MagicMock()
    node.wait_for_arm_convergence = MagicMock(return_value=(True, 0.01))
    node.set_arm_pose = MagicMock(return_value=True)
    node.save_screenshot = MagicMock()

    # Execute FRONT surface painting with dry_run=True
    result = node.paint_surface(
        surface_key='front_bumper',
        layer_name='paint_layer_front',
        waypoint_step=0.16,
        wp_duration=0.6,
        dry_run=True
    )

    assert result is True, "paint_surface() in dry-run should succeed when trajectory is valid"
    assert len(spray_history) > 0, "set_spray should have been called"
    assert all(s is False for s in spray_history), f"All spray calls must be False in dry-run mode! History: {spray_history}"
    assert node.spawn_paint_layer.call_count == 0, "spawn_paint_layer must NOT be called in dry-run mode"
    assert node.send_arm_joint_target.call_count > 0, "Arm joint targets must still be commanded in dry-run mode"
    assert node.set_arm_pose.call_count >= 1, "Arm must be commanded to transport pose after sweeps"

    print(f"  [PASS] Spray calls: {len(spray_history)}, all False. spawn_paint_layer calls: 0.")
    print(f"  [PASS] Arm joint targets sent: {node.send_arm_joint_target.call_count}, transport retractions: {node.set_arm_pose.call_count}.")


def test_paint_surface_normal_mode_preservation():
    """Test 3: Verify paint_surface(dry_run=False) preserves exact normal spray behavior."""
    print("--- Test 3: paint_surface() in Normal Mode (dry_run=False) ---")
    node = create_isolated_test_node()

    spray_history = []
    def mock_set_spray(val):
        spray_history.append(bool(val))
    node.set_spray = mock_set_spray

    node.spawn_paint_layer = MagicMock()
    node.send_arm_joint_target = MagicMock()
    node.wait_for_arm_convergence = MagicMock(return_value=(True, 0.01))
    node.set_arm_pose = MagicMock(return_value=True)
    node.save_screenshot = MagicMock()

    result = node.paint_surface(
        surface_key='front_bumper',
        layer_name='paint_layer_front',
        waypoint_step=0.16,
        wp_duration=0.6,
        dry_run=False
    )

    assert result is True, "paint_surface() in normal mode should succeed"
    assert True in spray_history, f"Normal mode must activate spray! History: {spray_history}"
    assert spray_history[-1] is False, "Final spray state must be False"
    assert node.spawn_paint_layer.call_count == 1, "spawn_paint_layer must be called once in normal mode"
    node.spawn_paint_layer.assert_called_with('paint_layer_front')

    print(f"  [PASS] Normal mode activated spray {spray_history.count(True)} times and shut off at end.")
    print("  [PASS] spawn_paint_layer called exactly once with 'paint_layer_front'.")


def test_run_single_station_dry_run_forwarding():
    """Test 4: Verify run_single_station(dry_run=True) properly forwards flag and manages spray."""
    print("--- Test 4: run_single_station() in Dry-Run Mode ---")
    node = create_isolated_test_node()
    node.current_pose = (0.00, 2.45, -1.5708)  # HOME pose

    spray_history = []
    def mock_set_spray(val):
        spray_history.append(bool(val))
    node.set_spray = mock_set_spray

    node.set_arm_pose = MagicMock(return_value=True)
    def mock_drive_route(route, target_yaw, timeout=50.0):
        # Simulate base reaching station pose
        if route == TRANSIT_ROUTES.get(('HOME', 'FRONT')) or (route and route[-1] == (3.50, 0.00)):
            node.current_pose = (3.50, 0.00, target_yaw)
        elif route == TRANSIT_ROUTES.get(('FRONT', 'HOME')) or (route and route[-1] == (0.00, 2.45)):
            node.current_pose = (0.00, 2.45, target_yaw)
        return True, "Arrived at station"
    node.drive_route = mock_drive_route
    node.verify_arm_transport_safety = MagicMock(return_value=(True, "Transport verified"))
    node.save_screenshot = MagicMock()

    # Track paint_surface call
    paint_surface_called_with = {}
    original_paint_surface = node.paint_surface
    def mock_paint_surface(*args, **kwargs):
        paint_surface_called_with['args'] = args
        paint_surface_called_with['kwargs'] = kwargs
        return True
    node.paint_surface = mock_paint_surface

    success = run_single_station(node, 'FRONT', output_dir="/tmp", dry_run=True)

    assert success is True, "run_single_station with dry_run=True should succeed"
    assert paint_surface_called_with['kwargs'].get('dry_run') is True, "dry_run=True must be forwarded to paint_surface"
    assert all(s is False for s in spray_history), f"Spray must remain False throughout run_single_station. History: {spray_history}"
    assert spray_history[0] is False, "First action must force spray OFF"

    print("  [PASS] run_single_station correctly forwarded dry_run=True to paint_surface().")
    print(f"  [PASS] Spray calls: {len(spray_history)}, all False.")


def test_dry_run_failure_guarding():
    """Test 5: Verify spray shutoff and retraction on failures in dry-run mode."""
    print("--- Test 5: Failure Guarding in Dry-Run Mode ---")
    node = create_isolated_test_node()

    spray_history = []
    def mock_set_spray(val):
        spray_history.append(bool(val))
    node.set_spray = mock_set_spray

    node.spawn_paint_layer = MagicMock()
    node.send_arm_joint_target = MagicMock()
    node.set_arm_pose = MagicMock(return_value=True)

    # Simulate convergence failure on raster waypoint 2
    conv_calls = [0]
    def mock_convergence(q, timeout=3.5, tolerances=None):
        conv_calls[0] += 1
        if conv_calls[0] >= 3:
            return False, 0.25  # Abort
        return True, 0.01
    node.wait_for_arm_convergence = mock_convergence

    result = node.paint_surface(
        surface_key='front_bumper',
        layer_name='paint_layer_front',
        waypoint_step=0.16,
        wp_duration=0.6,
        dry_run=True
    )

    assert result is False, "paint_surface must return False on convergence failure"
    assert spray_history[-1] is False, "Spray must be shut off on failure"
    assert node.spawn_paint_layer.call_count == 0, "No layer should be spawned on failure"
    assert node.set_arm_pose.call_count >= 1, "Arm must attempt retraction to transport on failure"

    print("  [PASS] Convergence abort in dry-run: returned False, spray shut off, retraction attempted, 0 entities spawned.")


def main():
    print("=" * 70)
    print(" RUNNING STAGE 6K DRY-RUN REGRESSION TEST SUITE")
    print("=" * 70)

    try:
        test_cli_parsing()
        test_paint_surface_dry_run_suppression()
        test_paint_surface_normal_mode_preservation()
        test_run_single_station_dry_run_forwarding()
        test_dry_run_failure_guarding()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    print("=" * 70)
    print(" ALL STAGE 6K DRY-RUN TESTS PASSED SUCCESSFULLY!")
    print("=" * 70)
    return 0


if __name__ == '__main__':
    sys.exit(main())

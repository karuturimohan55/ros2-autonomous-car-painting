#!/usr/bin/env python3
"""
Master Automated Test Runner for Yuktii AI Labs Robotic Car Painting Simulation.
Executes the complete suite of static, unit, integration, and regression tests.
Generates a structured execution report with PASS / FAIL metrics.
"""

import os
import sys
import time
import subprocess

TEST_SCRIPTS = [
    # Static & Specification Validation
    ("VAL-01", "Stage 1 Cell World & SDF Validation", "validate_cell.py"),
    ("VAL-02", "Stage 2 Robot URDF & Control Plugins", "validate_stage2.py"),

    # Kinematics & Motion Planning Unit Tests
    ("KIN-01", "Forward & Inverse Kinematics and Planning", "test_kinematics_and_planning.py"),
    ("NAV-01", "Corridor Navigation & Path Clearance", "test_base_navigation_static.py"),
    ("NAV-02", "Base Station Poses & Coordinate Frames", "test_base_pose_and_frames.py"),
    ("PLN-01", "Rear Trajectory Continuity & Joint Limits", "test_stage6c_rear_plan.py"),
    ("PLN-02", "Rear Surface Runtime Resolution Alignment", "test_stage6e_rear_runtime_alignment.py"),

    # Safety Interlocks & Controller Stability Tests
    ("SAF-01", "7-Point Safety Interlock Regression Suite", "test_stage6g_safety_interlocks.py"),
    ("DRY-01", "Dry-Run Spray & Deposition Suppression", "test_stage6k_dry_run.py"),
    ("CTL-01", "Arm Gravity Deflection Trim & Tolerances", "test_stage6l_gravity_deflection.py"),
    ("CTL-02", "Controller Anti-Windup & Stability Hardening", "test_stage6m_controller_stability.py"),
    ("REG-01", "Feedback Staleness Root-Cause & Queue Drain", "test_feedback_staleness_regression.py"),
]


def run_master_test_suite():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.abspath(os.path.join(script_dir, '../../..'))

    print("\n" + "=" * 80)
    print(" ROBOTIC CAR PAINTING CELL — MASTER AUTOMATED TEST SUITE")
    print(f" Target Directory: {script_dir}")
    print(f" Execution Date:   {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80 + "\n")

    results = []
    total_start = time.time()

    for test_id, description, filename in TEST_SCRIPTS:
        filepath = os.path.join(script_dir, filename)
        if not os.path.exists(filepath):
            results.append((test_id, description, filename, "MISSING", 0.0, "File not found"))
            continue

        print(f"Running [{test_id}] {description} ({filename})...")
        t0 = time.time()
        try:
            cmd = [sys.executable, filepath]
            proc = subprocess.run(
                cmd,
                cwd=ws_dir,
                capture_output=True,
                text=True,
                timeout=60.0
            )
            elapsed = time.time() - t0
            if proc.returncode == 0:
                print(f"  --> PASS ({elapsed:.2f}s)")
                results.append((test_id, description, filename, "PASS", elapsed, ""))
            else:
                print(f"  --> FAIL (exit {proc.returncode}) ({elapsed:.2f}s)")
                err_snippet = proc.stderr.strip() or proc.stdout.strip()
                last_line = err_snippet.splitlines()[-1] if err_snippet else "Unknown error"
                results.append((test_id, description, filename, "FAIL", elapsed, last_line))
        except subprocess.TimeoutExpired:
            elapsed = time.time() - t0
            print(f"  --> TIMEOUT ({elapsed:.2f}s)")
            results.append((test_id, description, filename, "TIMEOUT", elapsed, "Exceeded 60s timeout"))
        except Exception as e:
            elapsed = time.time() - t0
            print(f"  --> ERROR: {e}")
            results.append((test_id, description, filename, "ERROR", elapsed, str(e)))

    total_elapsed = time.time() - total_start

    # Summary Table
    print("\n" + "=" * 80)
    print(" TEST EXECUTION SUMMARY TABLE")
    print("=" * 80)
    print(f"{'ID':<8} | {'Test Suite':<40} | {'Status':<8} | {'Time (s)':<8} | {'Notes'}")
    print("-" * 80)

    passed_count = 0
    for test_id, description, filename, status, elapsed, notes in results:
        status_str = f"\033[92m{status}\033[0m" if status == "PASS" else f"\033[91m{status}\033[0m"
        print(f"{test_id:<8} | {description:<40} | {status_str:<17} | {elapsed:>7.2f}s | {notes}")
        if status == "PASS":
            passed_count += 1

    total_tests = len(results)
    print("-" * 80)
    print(f"TOTAL: {passed_count}/{total_tests} suites passed ({passed_count/total_tests*100:.1f}%) in {total_elapsed:.2f}s.")
    print("=" * 80 + "\n")

    return 0 if passed_count == total_tests else 1


if __name__ == '__main__':
    sys.exit(run_master_test_suite())

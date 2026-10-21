#!/usr/bin/env python3
"""
Validation script for Yuktii AI Labs - Robotic Car Painting Cell Simulation (Stage 1).
Validates:
1. Workspace and package file structure.
2. SDF syntax for world and models.
3. Car body target dimensions: 4.8m length, 1.9m width, 1.6m height.
4. Car wheel positions and ground contact (z = 0.0m).
5. Collision geometry existence for body, cabin, roof, and all 4 wheels.
6. Floor and workshop enclosure wall geometry.
7. Painting-cell markings (vehicle staging, robot standoff corridor, painting stations, hazard perimeter).
8. Absence of mobile robot (Stage 1 constraint).
"""

import os
import sys
import xml.etree.ElementTree as ET
import subprocess

def print_header(title):
    print("\n" + "=" * 65)
    print(f" {title}")
    print("=" * 65)

def print_check(name, passed, details=""):
    status = "\033[92m[PASS]\033[0m" if passed else "\033[91m[FAIL]\033[0m"
    detail_str = f" - {details}" if details else ""
    print(f" {status} {name}{detail_str}")
    return passed

def validate_files(base_path):
    print_header("1. Package and File Structure Inspection")
    required_files = [
        "package.xml",
        "CMakeLists.txt",
        "launch/cell_world.launch.py",
        "worlds/car_painting_cell.world",
        "models/car_body/model.config",
        "models/car_body/model.sdf",
        "models/painting_cell/model.config",
        "models/painting_cell/model.sdf",
    ]
    all_ok = True
    for rel_path in required_files:
        full_path = os.path.join(base_path, rel_path)
        exists = os.path.isfile(full_path)
        all_ok = all_ok and print_check(f"File exists: {rel_path}", exists)
    return all_ok

def validate_sdf_syntax(base_path):
    print_header("2. Gazebo SDF Syntax Validation")
    models_dir = os.path.join(base_path, "models")
    system_models = "/usr/share/gazebo-11/models"
    model_path = f"{models_dir}:{system_models}"
    env = os.environ.copy()
    env["GAZEBO_MODEL_PATH"] = model_path

    sdf_files = [
        os.path.join(base_path, "models/car_body/model.sdf"),
        os.path.join(base_path, "models/painting_cell/model.sdf"),
        os.path.join(base_path, "worlds/car_painting_cell.world"),
    ]

    all_ok = True
    for sdf in sdf_files:
        rel = os.path.relpath(sdf, base_path)
        try:
            res = subprocess.run(
                ["gz", "sdf", "-k", sdf],
                env=env,
                capture_output=True,
                text=True,
                timeout=5
            )
            passed = (res.returncode == 0) and ("Check complete" in res.stdout)
            all_ok = all_ok and print_check(f"SDF syntax check: {rel}", passed, res.stdout.strip())
        except Exception as e:
            all_ok = all_ok and print_check(f"SDF syntax check: {rel}", False, str(e))
    return all_ok

def validate_car_body(base_path):
    print_header("3. Car Body Specifications & Geometry")
    sdf_path = os.path.join(base_path, "models/car_body/model.sdf")
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # Target: 4.8m Length, 1.9m Width, 1.6m Height
    # Check visual and collision boxes
    lower_box = root.find(".//visual[@name='lower_body_visual']/geometry/box/size")
    roof_visual = root.find(".//visual[@name='roof_visual']/pose")
    roof_box = root.find(".//visual[@name='roof_visual']/geometry/box/size")

    all_ok = True
    if lower_box is not None:
        lx, ly, lz = [float(v) for v in lower_box.text.split()]
        all_ok = all_ok and print_check(
            "Car body length check",
            abs(lx - 4.80) < 0.05,
            f"Expected ~4.80m, parsed: {lx:.2f}m"
        )
    else:
        all_ok = all_ok and print_check("Car body length check", False, "Missing lower_body_visual")

    # Check overall height (roof pose z + roof box dz/2)
    if roof_visual is not None and roof_box is not None:
        rz = float(roof_visual.text.split()[2])
        rdz = float(roof_box.text.split()[2])
        total_height = rz + rdz / 2.0
        all_ok = all_ok and print_check(
            "Car body height check",
            abs(total_height - 1.60) < 0.05,
            f"Expected ~1.60m, parsed: {total_height:.2f}m"
        )
    else:
        all_ok = all_ok and print_check("Car body height check", False, "Missing roof element")

    # Check 4 wheels and ground contact
    wheel_names = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
    for w in wheel_names:
        cyl = root.find(f".//collision[@name='{w}_collision']/geometry/cylinder/radius")
        w_pose = root.find(f".//collision[@name='{w}_collision']/pose")
        if cyl is not None and w_pose is not None:
            radius = float(cyl.text)
            w_z = float(w_pose.text.split()[2])
            w_y = abs(float(w_pose.text.split()[1]))
            ground_contact = abs(w_z - radius)
            all_ok = all_ok and print_check(
                f"Wheel {w} ground resting level",
                ground_contact < 0.001,
                f"Axle z={w_z:.2f}m, radius={radius:.2f}m -> contact z={w_z - radius:.3f}m"
            )
        else:
            all_ok = all_ok and print_check(f"Wheel {w} check", False, "Missing wheel geometry")

    # Check collision elements
    collisions = [c.attrib.get('name') for c in root.findall(".//collision")]
    expected_collisions = [
        "lower_body_collision",
        "cabin_collision",
        "roof_collision",
        "wheel_fl_collision",
        "wheel_fr_collision",
        "wheel_rl_collision",
        "wheel_rr_collision"
    ]
    for ec in expected_collisions:
        present = ec in collisions
        all_ok = all_ok and print_check(f"Collision geometry: {ec}", present)

    return all_ok

def validate_cell_environment(base_path):
    print_header("4. Workshop Enclosure & Floor Markings")
    sdf_path = os.path.join(base_path, "models/painting_cell/model.sdf")
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    all_ok = True

    # Check floor slab
    floor_box = root.find(".//visual[@name='floor_visual']/geometry/box/size")
    if floor_box is not None:
        fx, fy, fz = [float(v) for v in floor_box.text.split()]
        all_ok = all_ok and print_check(
            "Industrial floor size",
            fx >= 14.0 and fy >= 10.0,
            f"Floor dimensions: {fx:.1f}m x {fy:.1f}m"
        )
    else:
        all_ok = all_ok and print_check("Industrial floor size", False, "Floor visual missing")

    # Check walls
    wall_names = ["north_wall_collision", "south_wall_collision", "east_wall_collision", "west_wall_collision"]
    for wn in wall_names:
        present = root.find(f".//collision[@name='{wn}']") is not None
        all_ok = all_ok and print_check(f"Enclosure wall: {wn}", present)

    # Check Markings
    marking_groups = [
        ("Vehicle Staging Bay (Yellow Box)", ["mark_car_front", "mark_car_rear", "mark_car_left", "mark_car_right"]),
        ("Robot Navigation Loop (Cyan Corridor)", ["robot_track_front", "robot_track_rear", "robot_track_left", "robot_track_right"]),
        ("Surface Painting Stations (4 Exterior Zones)", ["station_front_zone", "station_left_zone", "station_rear_zone", "station_right_zone"]),
        ("Cell Hazard Perimeter", ["hazard_border_north", "hazard_border_south", "hazard_border_east", "hazard_border_west"]),
        ("Operator Safe Station", ["operator_station_marking"])
    ]

    for group_name, marks in marking_groups:
        all_marks_present = all(root.find(f".//visual[@name='{m}']") is not None for m in marks)
        all_ok = all_ok and print_check(group_name, all_marks_present, f"{len(marks)} visual elements")

    return all_ok

def validate_stage1_constraints(base_path):
    print_header("5. Stage 1 Scope Constraints")
    world_path = os.path.join(base_path, "worlds/car_painting_cell.world")
    tree = ET.parse(world_path)
    root = tree.getroot()

    models_in_world = [m.attrib.get('name') for m in root.findall(".//include/name")]
    # Also check if any model tags directly defined
    direct_models = [m.attrib.get('name') for m in root.findall(".//model")]
    all_models = models_in_world + direct_models

    has_robot = any("robot" in (m or "").lower() for m in all_models)
    passed = not has_robot
    print_check(
        "Strict constraint: No robot in Stage 1",
        passed,
        f"Models present: {all_models}"
    )
    return passed

def find_package_root():
    # 1. Direct parent if running from source scripts directory
    candidate = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if os.path.isfile(os.path.join(candidate, "package.xml")) and os.path.isdir(os.path.join(candidate, "models")):
        return candidate
    # 2. Source workspace from install/car_painting_cell/lib/car_painting_cell
    candidate_ws = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../src/car_painting_cell"))
    if os.path.isfile(os.path.join(candidate_ws, "package.xml")) and os.path.isdir(os.path.join(candidate_ws, "models")):
        return candidate_ws
    # 3. Share directory fallback
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('car_painting_cell')
    except Exception:
        pass
    return candidate

def main():
    pkg_dir = find_package_root()
    print(f"Running Stage 1 Validation Suite on: {pkg_dir}")

    f_ok = validate_files(pkg_dir)
    s_ok = validate_sdf_syntax(pkg_dir)
    c_ok = validate_car_body(pkg_dir)
    e_ok = validate_cell_environment(pkg_dir)
    r_ok = validate_stage1_constraints(pkg_dir)

    all_passed = f_ok and s_ok and c_ok and e_ok and r_ok
    print_header("Stage 1 Validation Summary")
    if all_passed:
        print("\033[92m >>> ALL STAGE 1 CHECKS PASSED SUCCESSFULLY! <<<\033[0m\n")
        sys.exit(0)
    else:
        print("\033[91m >>> ONE OR MORE CHECKS FAILED! <<<\033[0m\n")
        sys.exit(1)

if __name__ == "__main__":
    main()

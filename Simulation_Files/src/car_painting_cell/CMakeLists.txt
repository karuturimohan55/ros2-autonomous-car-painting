cmake_minimum_required(VERSION 3.8)
project(car_painting_cell)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY
  launch
  worlds
  models
  urdf
  config
  scripts
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/arm_controller.py
  scripts/painting_state_machine.py
  scripts/execute_safe_trajectory.py
  scripts/navigate_to_station.py
  scripts/capture_evidence.py
  scripts/capture_screenshot.py
  scripts/validate_cell.py
  scripts/validate_stage2.py
  scripts/test_robot_control.py
  scripts/test_kinematics_and_planning.py
  scripts/test_base_navigation_static.py
  scripts/test_base_pose_and_frames.py
  scripts/test_repositioned_waypoint.py
  scripts/test_stage6c_rear_plan.py
  scripts/test_stage6e_rear_runtime_alignment.py
  scripts/test_stage6g_safety_interlocks.py
  scripts/test_stage6k_dry_run.py
  scripts/test_stage6l_gravity_deflection.py
  scripts/test_stage6m_controller_stability.py
  scripts/test_feedback_staleness_regression.py
  scripts/run_all_tests.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

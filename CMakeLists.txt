cmake_minimum_required(VERSION 3.8)
project(twbot_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)

# --- Testing / linting ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Install robot description (URDF/XACRO) ---
install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME}/
)

# --- Install launch files ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# --- Install config files ---
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)
#-- Install script ---
install(PROGRAMS
  scripts/torque_driver.py
  DESTINATION lib/${PROJECT_NAME}
)
#--install -balancing par PID ---
install(PROGRAMS
  scripts/balancing.py
  DESTINATION lib/${PROJECT_NAME}
)
#install --- balancing par ADRC
install(PROGRAMS
  scripts/ADRC_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install worlds files ---
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# --- Install worlds files ---
# --- Install Python scripts if any ---
# install(PROGRAMS scripts/my_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
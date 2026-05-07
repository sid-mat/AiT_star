cmake_minimum_required(VERSION 3.8)
project(ros2_ait_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── OMPL ─────────────────────────────────────────────────────────────────────
# sudo apt install ros-jazzy-ompl
find_package(ompl REQUIRED)

# ── Planner node ──────────────────────────────────────────────────────────────
add_executable(planner_node src/planner_node.cpp)

ament_target_dependencies(planner_node
  rclcpp geometry_msgs nav_msgs visualization_msgs std_msgs
  tf2_ros tf2_geometry_msgs
)

target_include_directories(planner_node PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(planner_node ${OMPL_LIBRARIES})

install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})

# ── Python scripts ─────────────────────────────────────────────────────────────
install(PROGRAMS
  scripts/drone_sim.py
  scripts/webots_drone_driver.py
  scripts/webots_obstacle_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Worlds, config, launch ────────────────────────────────────────────────────
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()

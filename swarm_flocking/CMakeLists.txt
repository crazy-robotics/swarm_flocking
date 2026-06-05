cmake_minimum_required(VERSION 3.8)
project(swarm_flocking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ─── Dependencies ─────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# PX4 autopilot message definitions — required for VehicleOdometry,
# TrajectorySetpoint, and OffboardControlMode
find_package(px4_msgs REQUIRED)

# ─── Message / Service generation ─────────────────────────────────────────────

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentState.msg"
  "msg/SwarmCommand.msg"
  "msg/FlockingDiagnostics.msg"
  "msg/ArenaConfig.msg"
  "srv/SetFormation.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
)

# ─── Core library (algorithm + subsystems, no ROS dependency) ─────────────────

add_library(swarm_flocking_core SHARED
  src/flocking_algorithm.cpp
  src/formation_controller.cpp
  src/communication_manager.cpp
  src/safety_monitor.cpp
)
target_include_directories(swarm_flocking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Core library has no ROS dependencies — pure C++17

# ─── ROS2 node executable ─────────────────────────────────────────────────────

add_executable(swarm_node
  src/swarm_controller.cpp
  src/swarm_node.cpp
)
target_include_directories(swarm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(swarm_node swarm_flocking_core)
ament_target_dependencies(swarm_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  px4_msgs
)

# Link generated swarm_flocking messages to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(swarm_node "${cpp_typesupport_target}")

# ─── Install ──────────────────────────────────────────────────────────────────

install(TARGETS swarm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS swarm_flocking_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ─── Export ────────────────────────────────────────────────────────────────────

ament_export_include_directories(include)
ament_export_libraries(swarm_flocking_core)

# ─── Tests ────────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_flocking_algorithm test/test_flocking_algorithm.cpp)
  target_link_libraries(test_flocking_algorithm swarm_flocking_core)
  target_include_directories(test_flocking_algorithm PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils swarm_flocking_core)
  target_include_directories(test_math_utils PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(ommpc_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find Eigen3 & OSQP
find_package(Eigen3 REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/home/dinesh/acados")
find_package(osqp REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolyTraj.msg"
  "msg/ReferenceState.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# Set include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  /home/dinesh/acados/include
)

# --- ommpc_node target ---
add_executable(ommpc_node src/ommpc_node.cpp)
ament_target_dependencies(ommpc_node
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ament_index_cpp
)
target_link_libraries(ommpc_node
  osqp::osqp
)
# Ensure custom messages are generated first and link them
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
add_dependencies(ommpc_node ${cpp_typesupport_target})
target_link_libraries(ommpc_node ${cpp_typesupport_target})

# --- ecrl_data_collector_node target ---
add_executable(ecrl_data_collector_node src/ecrl_data_collector_node.cpp)
ament_target_dependencies(ecrl_data_collector_node
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ament_index_cpp
)
add_dependencies(ecrl_data_collector_node ${cpp_typesupport_target})
target_link_libraries(ecrl_data_collector_node ${cpp_typesupport_target})

# Install executables and scripts
install(TARGETS
  ommpc_node
  ecrl_data_collector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  px4_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ament_index_cpp
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nominal_dynamics test/test_nominal_dynamics.cpp)
  target_include_directories(test_nominal_dynamics PRIVATE
    include
    ${EIGEN3_INCLUDE_DIR}
    /home/dinesh/acados/include
  )
  ament_target_dependencies(test_nominal_dynamics
    rclcpp
    px4_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
  )
  target_link_libraries(test_nominal_dynamics osqp::osqp)
  add_dependencies(test_nominal_dynamics ${cpp_typesupport_target})
  target_link_libraries(test_nominal_dynamics ${cpp_typesupport_target})

  # Pure-math fixture generator (Eigen only, no ROS/OSQP deps) consumed by
  # scripts/test_residuals_parity.py to cross-check the Python port of
  # nominal_dynamics.h against the C++ original.
  add_executable(gen_nominal_dynamics_fixture test/gen_nominal_dynamics_fixture.cpp)
  target_include_directories(gen_nominal_dynamics_fixture PRIVATE include ${EIGEN3_INCLUDE_DIR})
endif()

ament_package()

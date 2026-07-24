# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target ommpc_ros2::ommpc_ros2
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${ommpc_ros2_TARGETS}.
if(ommpc_ros2_TARGETS AND NOT TARGET ommpc_ros2::ommpc_ros2)
  add_library(ommpc_ros2::ommpc_ros2 INTERFACE IMPORTED)
  set_target_properties(ommpc_ros2::ommpc_ros2 PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ommpc_ros2_TARGETS}")
endif()

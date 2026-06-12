cmake_minimum_required(VERSION 3.8)
project(kuka_landing_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# gz-transport for pose_sync node
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)

# Pose sync node (syncs KUKA world pose to WAM-V)
add_executable(pose_sync src/pose_sync.cpp)
target_link_libraries(pose_sync
  gz-transport12::gz-transport12
  gz-msgs9::gz-msgs9
)

# Landing tracker node
add_executable(landing_tracker src/landing_tracker.cpp)
target_include_directories(landing_tracker PRIVATE include)
ament_target_dependencies(landing_tracker
  rclcpp moveit_ros_planning_interface geometry_msgs
  sensor_msgs tf2_ros tf2_geometry_msgs tf2_eigen std_msgs
)
target_link_libraries(landing_tracker ${tf2_ros_LIBRARIES})

# Install executables
install(TARGETS landing_tracker pose_sync
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS scripts/scale_mass.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install launch, urdf, srdf, and world files
install(DIRECTORY launch urdf srdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

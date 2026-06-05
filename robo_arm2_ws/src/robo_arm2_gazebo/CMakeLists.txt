cmake_minimum_required(VERSION 3.8)
project(robo_arm2_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/gz_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

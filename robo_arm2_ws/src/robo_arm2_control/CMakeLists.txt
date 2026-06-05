cmake_minimum_required(VERSION 3.8)
project(robo_arm2_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/auto_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

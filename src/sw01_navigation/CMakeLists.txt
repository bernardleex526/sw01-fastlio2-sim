cmake_minimum_required(VERSION 3.8)
project(sw01_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/send_nav_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

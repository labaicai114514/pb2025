cmake_minimum_required(VERSION 3.8)
project(pb_minco_sim_integration)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

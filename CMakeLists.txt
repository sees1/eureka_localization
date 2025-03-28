cmake_minimum_required(VERSION 3.5)
project(eureka_localization)
find_package(ament_cmake REQUIRED)

# INSTALL
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

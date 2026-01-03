cmake_minimum_required(VERSION 3.8)
project(flembot_description)

find_package(ament_cmake REQUIRED)

# Install URDF and meshes
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


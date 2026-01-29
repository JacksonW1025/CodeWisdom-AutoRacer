cmake_minimum_required(VERSION 3.8)
project(autoracer_robot_urdf)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# 纯资源包，无 C++ 编译
install(DIRECTORY urdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

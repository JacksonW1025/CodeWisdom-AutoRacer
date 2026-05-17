cmake_minimum_required(VERSION 3.8)
project(autoracer_slam_toolbox)

find_package(ament_cmake REQUIRED)

# 安装配置文件和 launch 文件
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

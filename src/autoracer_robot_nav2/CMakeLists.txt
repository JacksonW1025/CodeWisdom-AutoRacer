cmake_minimum_required(VERSION 3.8)
project(autoracer_robot_nav2)

find_package(ament_cmake REQUIRED)

# 安装配置文件、launch 文件、地图文件和 RViz 配置
install(DIRECTORY launch param map rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/twist_to_ackermann.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME twist_to_ackermann
)

ament_package()

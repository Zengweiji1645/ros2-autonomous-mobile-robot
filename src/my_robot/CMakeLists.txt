
cmake_minimum_required(VERSION 3.8)
project(my_robot)

# ==========================================
# 1. 编译器设置
# ==========================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ==========================================
# 2. 查找依赖
# ==========================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)


# ==========================================
# 3. 基础 ROS2 学习节点
# ==========================================

add_executable(my_node
  src/my_node.cpp
)

ament_target_dependencies(my_node
  rclcpp
)

add_executable(publisher_node
  src/publisher_node.cpp
)

ament_target_dependencies(publisher_node
  rclcpp
  std_msgs
)

add_executable(subscriber_node
  src/subscriber_node.cpp
)

ament_target_dependencies(subscriber_node
  rclcpp
  std_msgs
)

add_executable(velocity_publisher
  src/velocity_publisher.cpp
)

ament_target_dependencies(velocity_publisher
  rclcpp
  geometry_msgs
)


# ==========================================
# 4. RobotNode：机器人仿真
# ==========================================

add_executable(robot_node
  src/robot_node.cpp
  src/Robot.cpp
  src/Odometry.cpp
  src/WheelEncoder.cpp
  src/Lidar.cpp
  src/World.cpp
)

ament_target_dependencies(robot_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros
  nav_msgs
  sensor_msgs
)

target_include_directories(robot_node
  PRIVATE
  include
)


# ==========================================
# 5. ControllerNode：自研 A* + PID
# ==========================================

add_executable(controller_node
  src/controller_node.cpp
  src/Controller.cpp
  src/PIDController.cpp
  src/AStarPlanner.cpp
)

ament_target_dependencies(controller_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
)

target_include_directories(controller_node
  PRIVATE
  include
)


# ==========================================
# 6. A* 独立测试
# ==========================================

add_executable(astar_test
  src/astar_test.cpp
  src/AStarPlanner.cpp
)

target_include_directories(astar_test
  PRIVATE
  include
)


# ==========================================
# 7. MissionNode：Nav2 任务管理
# ==========================================

add_executable(mission_node
  src/mission_node.cpp
)

ament_target_dependencies(mission_node
  rclcpp
  rclcpp_action
  nav2_msgs
  std_msgs
  geometry_msgs
)

target_link_libraries(mission_node
  yaml-cpp
)


# ==========================================
# 8. 安装可执行程序
# ==========================================

install(
  TARGETS
    my_node
    publisher_node
    subscriber_node
    velocity_publisher
    robot_node
    controller_node
    astar_test
    mission_node
  DESTINATION lib/${PROJECT_NAME}
)


# ==========================================
# 9. 安装配置文件
# ==========================================

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# ==========================================
# 10. 测试配置
# ==========================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()


# ==========================================
# 11. ROS2 包声明
# ==========================================

ament_package()

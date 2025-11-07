cmake_minimum_required(VERSION 3.8)
project(sl_nav2_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED) 

# add the executable
add_executable(waypoint_saver src/waypoint_saver.cpp)
add_executable(waypoint_publisher src/waypoint_publisher.cpp)
add_executable(waypoint_publisher_navthrough src/waypoint_publisher_navthrough.cpp)
add_executable(get_poses_remaining src/get_poses_remaining.cpp)
add_executable(add_new_goals src/add_new_goals.cpp)


# link dependencies
ament_target_dependencies(waypoint_saver
  rclcpp
  geometry_msgs
)
ament_target_dependencies(waypoint_publisher
  rclcpp
  rclcpp_action
  nav2_msgs
  std_srvs
  geometry_msgs
)
ament_target_dependencies(waypoint_publisher_navthrough
  rclcpp
  rclcpp_action
  nav2_msgs
  std_srvs
  geometry_msgs
)

ament_target_dependencies(get_poses_remaining
  rclcpp
  rclcpp_action
  nav2_msgs
  std_srvs
  geometry_msgs
)

ament_target_dependencies(add_new_goals
  rclcpp
  rclcpp_action
  nav2_msgs
  std_srvs
  geometry_msgs
)

# 外部ライブラリ(yaml-cpp)をリンク
target_link_libraries(waypoint_saver
  ${YAML_CPP_LIBRARIES}
)
target_link_libraries(waypoint_publisher
  ${YAML_CPP_LIBRARIES}
)
target_link_libraries(waypoint_publisher_navthrough
  ${YAML_CPP_LIBRARIES}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY paths DESTINATION share/${PROJECT_NAME})
install(TARGETS
  waypoint_saver
  waypoint_publisher
  waypoint_publisher_navthrough
  get_poses_remaining
  add_new_goals
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
endif()

ament_package()
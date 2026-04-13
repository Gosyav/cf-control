cmake_minimum_required(VERSION 3.8)
project(uav_dynamics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_executable(trajectory_node
  src/trajectory_node.cpp
  src/flatness_mapper.cpp
)

ament_target_dependencies(trajectory_node
  rclcpp
  geometry_msgs
  tf2
)

target_link_libraries(trajectory_node Eigen3::Eigen)

install(TARGETS trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

add_executable(test_csv
  test/test_csv.cpp
  src/flatness_mapper.cpp
)

target_link_libraries(test_csv Eigen3::Eigen)

install(TARGETS test_csv
  DESTINATION lib/${PROJECT_NAME}
)

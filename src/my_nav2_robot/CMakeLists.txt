cmake_minimum_required(VERSION 3.8)
project(my_nav2_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(small_gicp REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(data_handle_node src/data_handle.cpp)
add_executable(imu_adapter_node src/imu_adapter.cpp)

ament_target_dependencies(data_handle_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  Eigen3
  rm_interfaces
  pcl_conversions
)
target_link_libraries(data_handle_node ${PCL_LIBRARIES} small_gicp::small_gicp)

ament_target_dependencies(imu_adapter_node
  rclcpp
  sensor_msgs
  rm_interfaces
)

install(TARGETS
  data_handle_node
  imu_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

# This package only contains config files, no libraries to build

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY behaviour_trees
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

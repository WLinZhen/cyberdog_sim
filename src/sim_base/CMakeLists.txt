cmake_minimum_required(VERSION 3.8)
project(sim_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ros_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  Eigen3
  gazebo_msgs
  gazebo_dev
  gazebo_ros
  sensor_msgs
  geometry_msgs
  rosidl_default_generators
  rosidl_typesupport_cpp
  ros_bridge
  tf2_ros
  std_msgs 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

install(
  DIRECTORY launch config model world
  DESTINATION share/sim_base  
)


ament_export_dependencies(rosidl_default_runtime rosidl_typesupport_cpp)

# Add your plugin
add_library(legged_plugin SHARED 
  src/actors_gazebo/legged_plugin.cpp
  src/actors_rviz/state_publisher.cpp
  #src/actors_gazebo/visual_plugin.cpp
)

ament_target_dependencies(legged_plugin ${dependencies})


target_link_libraries(legged_plugin ${GAZEBO_LIBRARIES} )

install(TARGETS legged_plugin 
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

add_executable(state_publisher_node src/actors_rviz/state_publisher_node.cpp src/actors_rviz/state_publisher.cpp)
target_link_libraries(state_publisher_node legged_plugin)

install(
  TARGETS state_publisher_node
  DESTINATION lib/sim_base
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(cyber_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_bridge REQUIRED)
find_package(lcm REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  rosidl_default_generators
  rosidl_typesupport_cpp
  ros_bridge
  std_msgs 
  lcm
  sensor_msgs
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

ament_export_dependencies(rosidl_default_runtime rosidl_typesupport_cpp)

# Add your plugin

add_library(cyber_dog_motor_sdk SHARED 
  src/custom_interface.cpp 
  src/MotorCtrl.cpp
)
# set_target_properties(cyber_dog_motor_sdk PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib)

# target_include_directories(cyber_dog_motor_sdk PUBLIC
#                     "include"
# )
ament_target_dependencies(cyber_dog_motor_sdk  ${dependencies})
target_link_libraries(cyber_dog_motor_sdk
  pthread
  lcm
)

install(TARGETS cyber_dog_motor_sdk 
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# compile use code
add_executable(Example_MotorCtrl src/Example_MotorCtrl.cpp)
# link_directories(${CMAKE_INSTALL_PREFIX}/lib)
target_link_libraries(Example_MotorCtrl cyber_dog_motor_sdk)
install(
  TARGETS Example_MotorCtrl
  DESTINATION lib/cyber_base
)
ament_package()

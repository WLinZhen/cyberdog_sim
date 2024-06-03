cmake_minimum_required(VERSION 3.8)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(PLATFORM gazebo)

if(NOT DEFINED PLATFORM)
  message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${PLATFORM} STREQUAL "gazebo")
  set(DRAW_SWING ON)  # support ON and OFF
  if(${DRAW_SWING} STREQUAL "ON")
    message("**** USE DRAW_SWING ****")
    add_definitions(-DDRAW_SWING_TRAJECTORY)
  endif()
  #  add_definitions(-DSIMULATION_WITH_GAZEBO)
endif()
# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(cyber_guide)

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
set(CMAKE_COLOR_MAKEFILE   ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
option(MINI_CHEETAH_BUILD "use compiler flags for mini cheetah computer" ON)
set(BUILD_TYPE_RELEASE TRUE)
option(NO_SIM "Do not build simulator" OFF)
SET (THIS_COM "../" )

set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native  -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wsign-promo -Wcpp")
set(CMAKE_C_FLAGS "-O3 -march=native  -ggdb -std=gnu99 -I.")
message("**** CYBER quadruped robot build enabled ****")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

if(${PLATFORM} STREQUAL "gazebo")
  find_package(Boost)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(ros_bridge REQUIRED)
  find_package(geometry_msgs REQUIRED)
  #find_package(gazebo_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  #find_package(gazebo_ros REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rosidl_typesupport_cpp REQUIRED)

  #  for catkin make
  link_directories(./common/include/SimUtilities/)
  include_directories("./")
  include_directories("/opt/ros/galactic/include")
  include_directories("./sim/gazebo/include")
  #  include_directories("./sim/gazebo/include")
  include_directories("./common")
  include_directories("./common/include")
  include_directories("./user/MIT_Controller")
  include_directories("./robot/include")
  include_directories("./common/FootstepPlanner")
  include_directories("./common/FootstepPlanner")
  include_directories("/usr/local/include/lcm")
  include_directories(SYSTEM "third-party/qpOASES/include")

  link_directories(
          ${GAZEBO_LIBRARY_DIRS}
  )

  set(dependencies
          rclcpp
          Eigen3
          #gazebo_msgs
          #gazebo_dev
          #gazebo_ros
          rosidl_default_generators
          rosidl_typesupport_cpp
          ros_bridge
          std_msgs
  )

  add_subdirectory(sim)
  add_subdirectory(robot)
  add_subdirectory(third-party)
  add_subdirectory(common)
  add_subdirectory(user)
  # Setup the target executable.
  add_executable(cyber_guide ${SOURCES})

  target_link_libraries(cyber_guide  mit_ctrl robot biomimetics qpOASES osqp WBC_Ctrl Goldfarb_Optimizer pthread lcm inih dynacore_param_handler lord_imu soem)
  target_link_libraries(cyber_guide  ${catkin_LIBRARIES})
  target_link_libraries(cyber_guide  ioros)

  ament_target_dependencies(cyber_guide ${${PROJECT_NAME}_EXPORTED_TARGETS} ${dependencies})

  install(
          TARGETS cyber_guide
          DESTINATION lib/cyber_guide
  )
endif()

ament_package()

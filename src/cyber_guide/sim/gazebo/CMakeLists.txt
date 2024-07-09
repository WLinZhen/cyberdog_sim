include_directories("/usr/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include")

#find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(ros_bridge REQUIRED)

SET(C_SOURCES "./src/interface/IOROS.cpp")
link_directories(${ament_cmake_LIB_DIRS} lib)

add_library(ioros SHARED ${C_SOURCES})

target_link_libraries (ioros Eigen3::Eigen)
ament_target_dependencies(ioros  ros_bridge)

install(TARGETS ioros
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

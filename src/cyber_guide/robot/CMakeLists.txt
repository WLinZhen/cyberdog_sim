include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include/")
include_directories("./")
include_directories("../common/include/")
include_directories("../third-party")
include_directories("../third-party/ParamHandler")
include_directories("../third-party/vectornav/include")
include_directories("../third-party/vectornav/include/vn")
include_directories("../third-party/lord_imu/Include")
include_directories("../../../install/ros_bridge/include")

include_directories("../third-party/SOEM/osal")
include_directories("../third-party/SOEM/osal/linux")
include_directories("../third-party/SOEM/oshw")
include_directories("../third-party/SOEM/oshw/linux")

file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp")
link_directories(${ament_cmake_LIB_DIRS} lib)
add_library(robot SHARED ${sources})

target_link_libraries(robot biomimetics pthread inih dynacore_param_handler lord_imu soem)
if(CMAKE_SYSTEM_NAME MATCHES Linux)
#target_link_libraries(robot libvnc rt)
# use for keyboard control
target_link_libraries(robot libvnc rt -pthread)

install(TARGETS robot
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
endif()

include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")
include_directories("../../../../install/ros_bridge/include")


include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
# "Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")
find_package(rclcpp REQUIRED)
add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)


add_library(mit_ctrl ${sources} MIT_Controller.cpp)

ament_target_dependencies(mit_ctrl  rclcpp)
target_link_libraries(mit_ctrl robot biomimetics )
target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(mit_ctrl WBC_Ctrl)


install(TARGETS mit_ctrl
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
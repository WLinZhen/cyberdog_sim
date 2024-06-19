// generated from
// rosidl_typesupport_fastrtps_c/resource/rosidl_typesupport_fastrtps_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef ROS_BRIDGE__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
#define ROS_BRIDGE__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_ros_bridge __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_bridge __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_ros_bridge __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_bridge __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_C_BUILDING_DLL_ros_bridge
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_bridge ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_ros_bridge
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_bridge ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_bridge
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_ros_bridge __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ros_bridge
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_bridge __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros_bridge
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // ROS_BRIDGE__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

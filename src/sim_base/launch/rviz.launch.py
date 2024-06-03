import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    #   file_name
    package_name = 'sim_base'
    rviz_config = "rviz2.config.rviz"

    #   file_path
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    pkg_prefix = get_package_prefix(package_name)

    rviz_config_path = os.path.join(pkg_share, f'config/{rviz_config}')

    #   gazebo_env
    my_env = os.environ.copy()
    my_env["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share,'model')
    
    ld = LaunchDescription()

    my_env_mod = SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_prefix, "share"))
  
    rviz2_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_path],
        )

    ld.add_action(my_env_mod)

    ld.add_action(rviz2_node)

    return ld

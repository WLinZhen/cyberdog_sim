import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    #   file_name
    package_name = 'sim_base'
    urdf_name = "actors.urdf"
    rviz_config = "rviz2.config.rviz"
    robot_name_in_model = 'actors'
    world_name = "stair7cm.world"#stair7cm.world
    visual_name_in_model = 'traj_base'
    visual_name = "visual.urdf"
    #   file_path
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    pkg_prefix = get_package_prefix(package_name)
    pkg_lib = os.path.join(pkg_prefix, "lib")

    urdf_model_path = os.path.join(pkg_share, f'model/{urdf_name}')
    visual_model_path = os.path.join(pkg_share, f'model/{visual_name}')
    rviz_config_path = os.path.join(pkg_share, f'config/{rviz_config}')
    world_path = os.path.join(pkg_share, f'world/{world_name}')
    
    #   gazebo_env
    my_env = os.environ.copy()
    my_env["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share,'model')
    
    ld = LaunchDescription()

    my_env_mod = SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(pkg_prefix, "share"))

    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output='screen',
        arguments = [urdf_model_path],
        parameters=[{"use_sim_time": True}],
        )
    
    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd = ['gazebo', '--verbose',world_path,'-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output = 'screen',
        )

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', urdf_model_path,
            '-x', '0',
            '-y', '0',
            '-z', '0.4',
            ], 
        output='screen',
        )
    
    visual_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', visual_name_in_model,
            '-file', visual_model_path,
            '-x', '0',
            '-y', '0',
            '-z', '0',
            ], 
        output='screen',
        )
    
    state_publisher_node = Node(
        package='sim_base',  # replace with your actual package name
        executable='state_publisher_node',  # replace with your actual executable
        name='state_publisher_node',
        parameters=[{"use_sim_time": True}],
    ) 

    rviz2_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_path],
        parameters=[{"use_sim_time": True}],
        )

    ld.add_action(my_env_mod)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(state_publisher_node)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(visual_cmd)
    #ld.add_action(rviz2_node)
    return ld


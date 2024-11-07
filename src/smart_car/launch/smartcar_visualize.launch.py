from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Locate the URDF file using the package name and the shared directory
    pkg_name = 'smart_car'
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'smartcar.urdf.xacro'
    )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Define the robot description using xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Path to your Gazebo world file, if any
    world_file = os.path.join(pkg_name, 'world', 'smalltown.world')  # Optional, remove if unnecessary

    default_world_path = PathJoinSubstitution([FindPackageShare(pkg_name), 'world', 'smalltown.world'])

    ld.add_action(DeclareLaunchArgument(name='world', 
        default_value=default_world_path,
        description='Path to the Gazebo world file'
    ))
    
    rviz_config = os.path.join(
    get_package_share_directory(pkg_name),
    'rviz',
    'visualize.rviz'
    )

    # Node for robot_state_publisher
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    ))
    # Launch Gazebo and load the world file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    ))
    # Spawn the robot entity into the Gazebo world
    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'smart_robot_car', '-topic', 'robot_description'],
        output='screen',
    ))


    return ld

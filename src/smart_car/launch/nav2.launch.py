from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()
    package_name = 'smart_car'
    smartcar_sim_path = FindPackageShare(package_name)
    nav2_bringup_path = FindPackageShare('nav2_bringup')
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'smartcar.urdf.xacro'
    )

    default_rviz_config_path = PathJoinSubstitution([nav2_bringup_path, 'rviz', 'nav2_default_view.rviz'])

    default_world_path = PathJoinSubstitution([smartcar_sim_path, 'world', 'smalltown.world'])

    default_ekf_config_path = PathJoinSubstitution([smartcar_sim_path, 'config', 'config.yaml'])
    default_map_config_path = PathJoinSubstitution([smartcar_sim_path, 'map', 'smalltown_world.yaml'])

    rviz_config = os.path.join(
    get_package_share_directory(package_name),
    'rviz',
    'visualize.rviz'
    )

    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))


    ld.add_action(DeclareLaunchArgument(name='world', default_value=default_world_path,
                                        description='Path to the Gazebo world file'))
    
    ld.add_action(DeclareLaunchArgument(name="ekf_config", default_value=default_ekf_config_path,
                                        description='Path to extended kalman filter config'))
    
    ld.add_action(DeclareLaunchArgument(name="map_config", default_value=default_map_config_path,
                                        description='Path to map config'))
    # Define the robot description using xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # In the robot_state_publisher node section
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

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0','0','0','0','0','0','map','odom']
    ))
        
    ld.add_action(Node(
        package=package_name,
        executable='joint_state_publisher.py',
        parameters=[{'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package=package_name,
        executable='wheel_odom_publisher.py',
        parameters=[{'use_sim_time': True}]
    ))

    ld.add_action(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config'), {'use_sim_time': True}]
    ))

    # Call this launch file before the next otherwise the map config doesn't get passed to the launch file????
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])]
        ),
        launch_arguments={'map': LaunchConfiguration('map_config')}.items()
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    ))

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
# launch/localization.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the config file
    config_file_path = os.path.join(
        get_package_share_directory('smart_car'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        # Joint State Publisher
        Node(
            package='smart_car',
            executable='joint_state_publisher.py',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Wheel Odometry Publisher
        Node(
            package='smart_car',
            executable='wheel_odom_publisher.py',
            name='wheel_odom_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Robot Localization EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file_path, {'use_sim_time': True}]
        )
    ])

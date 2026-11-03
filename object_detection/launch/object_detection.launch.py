from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('object_detection'),
        'rviz',
        'rviz_config.rviz'
    )

    static_transform_node = Node(
        package='object_detection',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    object_detection_node = Node(
        package='object_detection',
        executable='object_detection',
        name='object_detection',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        static_transform_node,
        object_detection_node,
        rviz_node
    ])
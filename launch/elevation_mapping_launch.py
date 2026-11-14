from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rslidar = Node(
        package = 'rslidar_sdk', # first part of what you get when you run ros2 pkg executables rslidar_sdk
        executable = 'rslidar_sdk_node',
        name = 'rslidar_sdk_node',
        output = 'screen'
    )

    static_tf = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'lidar_static_tf',
        arguments = ['--x', '0', '--y', '0', '--z', '0.5', # change the offset from robot to lidar frame
               '--roll', '0', '--pitch', '0.2', '--yaw', '0',
               '--frame-id', 'map',
               '--child-frame-id', 'rslidar'] 
    )

    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen'
    )

    return LaunchDescription([rslidar, static_tf, rviz])




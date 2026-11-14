from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rslidar = Node(
        package = 'rslidar_sdk', # first part of what you get when you run ros2 pkg executables rslidar_sdk
        executable = 'rslidar_sdk_node',
        name = 'rslidar_sdk_node',
        output = 'screen'
    )

    # robot pose comes from FAST-LIO2 (run separately), which publishes tf camera_init -> body
    # names below are FAST-LIO2 defaults, check with: ros2 topic list / ros2 run tf2_tools view_frames
    elevation_map = Node(
        package = 'mapping',
        executable = 'elevation_map_node',
        name = 'elevation_mapping_node',
        output = 'screen',
        parameters = [{
            'map_frame': 'camera_init',          # FAST-LIO2 world frame (pose at start-up)
            'base_frame': 'body',                # FAST-LIO2 moving frame (lidar imu)
            'cloud_topic': '/cloud_registered',  # FAST-LIO2 scan already in camera_init frame. for patchwork++ use its ground topic instead
        }]
    )

    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen'
    )

    return LaunchDescription([rslidar, elevation_map, rviz])




from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='camera_calibration',
            name='camera_calibration_node',
            output='screen',
            parameters=['src/camera_calibation/camera_calibration/config/config.yaml']
        )
    ])

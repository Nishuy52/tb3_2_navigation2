from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='tb3_2',
            description='Robot namespace — must match the running Nav2/SLAM instance',
        ),
        DeclareLaunchArgument(
            'unknown_threshold',
            default_value='0.05',
            description=(
                'Stop exploring when the fraction of unknown cells drops below '
                'this value (0.0–1.0). Default 0.05 = 5%.'
            ),
        ),
        Node(
            package='tb3_2_navigation2',
            executable='auto_nav.py',
            name='frontier_explorer',
            output='screen',
            # Topics are constructed as absolute paths inside the node, so we do
            # NOT set namespace= here — that would cause double-prefixing (same
            # reason the relay nodes in navigation2.launch.py avoid it).
            parameters=[{
                'namespace':         LaunchConfiguration('namespace'),
                'unknown_threshold': LaunchConfiguration('unknown_threshold'),
            }],
        ),
    ])

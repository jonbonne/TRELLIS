"""Launch file for trellis_ros."""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """Generate LaunchDescription object."""
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='trellis_server'),
        DeclareLaunchArgument('log_level', default_value=["info"], description="Logging level"),
        Node(
            package='trellis_ros',
            executable='trellis_node',
            name=LaunchConfiguration('name'),
            parameters=[PathJoinSubstitution([FindPackageShare('trellis_ros'), 'config', 'config.yaml'])],
            output='screen',
            emulate_tty=True,
            arguments=[
                '--ros-args',
                '--log-level',
                LaunchConfiguration('log_level')
            ]
        )
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()

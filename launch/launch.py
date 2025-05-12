from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'maze_solve'

    camera_urdf_path = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        'cam.urdf'
    ])

    return LaunchDescription([
        Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'overhead_camera',
        '-file', camera_urdf_path,
        '-x', '0.0', '-y', '0.0', '-z', '5.0',
        '-R', '1.57', '-P', '0.0', '-Y', '0.0'  # Rotate to look downward
    ],
    output='screen'
)
    ])


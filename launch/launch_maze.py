from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package and world file path
    pkg_share = FindPackageShare('maze_solve')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'mazeworld.world'])

    # Launch Gazebo with your custom world that includes the camera model
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        gazebo,
    ])

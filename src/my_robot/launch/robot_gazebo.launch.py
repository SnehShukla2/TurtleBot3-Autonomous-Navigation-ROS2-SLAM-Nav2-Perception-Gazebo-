from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Path to the world file
    world_file_path = os.path.join(
        os.path.dirname(__file__), '..', 'worlds', 'my_world.world'
    )

    # Path to the robot URDF file
    urdf_file_path = os.path.join(
        os.path.dirname(__file__), '..', 'resource', 'robot.urdf'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-file', urdf_file_path,
                '-entity', 'my_robot'
            ],
            output='screen'
        ),
    ])



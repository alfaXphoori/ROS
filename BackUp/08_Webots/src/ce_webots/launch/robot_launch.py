import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('ce_webots')
    world_file = os.path.join(package_dir, 'worlds', 'simple_robot.wbt')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['webots', '--mode=realtime', world_file],
            output='screen'
        )
    ])

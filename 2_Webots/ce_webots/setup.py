from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ce_webots'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CE Robotics',
    maintainer_email='admin@example.com',
    description='Webots + ROS2 tutorial package for learning robotics simulation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            '001_simple_robot_controller = ce_webots.001_simple_robot_controller:main',
            '002_keyboard_teleop = ce_webots.002_keyboard_teleop:main',
            '011_wheel_encoder_mission = ce_webots.011_wheel_encoder_mission:main',
            '012_keyboard_with_distance = ce_webots.012_keyboard_with_distance:main',
            '021_touch_sensor_controller = ce_webots.021_touch_sensor_controller:main',
            '022_distance_sensor_controller = ce_webots.022_distance_sensor_controller:main',
            '031_imu_controller = ce_webots.031_imu_controller:main',
            '032_accelerometer_controller = ce_webots.032_accelerometer_controller:main',
            '041_camera_controller = ce_webots.041_camera_controller:main',
            '042_line_follower_controller = ce_webots.042_line_follower_controller:main',
            '051_lidar_controller = ce_webots.051_lidar_controller:main',
            '052_slam_controller = ce_webots.052_slam_controller:main',
            '061_depth_camera_controller = ce_webots.061_depth_camera_controller:main',
            '062_3d_mapping = ce_webots.062_3d_mapping:main',
            '071_go_to_goal = ce_webots.071_go_to_goal:main',
            'save_map = ce_webots.save_map:main',
        ],
    },
)

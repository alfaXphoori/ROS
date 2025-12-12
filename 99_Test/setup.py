from setuptools import setup

package_name = 'ce_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@ksu.ac.th',
    description='CE Robot ROS 2 examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 00_Install
            'first_node = ce_robot.first_node:main',
            
            # 01_Publisher_Subscriber
            '01_first_pub = ce_robot.first_publisher:main',
            '01_first_sub = ce_robot.first_subscriber:main',
            '01_simple_publisher = ce_robot.simple_publisher:main',
            '01_simple_subscriber = ce_robot.simple_subscriber:main',
            '01_counter_publisher = ce_robot.counter_publisher:main',
            '01_counter_processor = ce_robot.counter_processor:main',
            '01_counter_logger = ce_robot.counter_logger:main',
            '01_temperature_publisher = ce_robot.temperature_publisher:main',
            '01_temperature_monitor = ce_robot.temperature_monitor:main',
            '01_humidity_publisher = ce_robot.humidity_publisher:main',
            '01_pressure_publisher = ce_robot.pressure_publisher:main',
            '01_sensor_monitor = ce_robot.sensor_monitor:main',
            
            # 02_Server_Client
            '02_add_two_server = ce_robot.add_two_ints_server:main',
            '02_add_two_client = ce_robot.add_twts_ints_client:main',
            '02_temp_converter_server = ce_robot.temp_converter_server:main',
            '02_temp_converter_client = ce_robot.temp_converter_client:main',
            '02_database_server = ce_robot.database_server:main',
            '02_database_client = ce_robot.database_client:main',
            '02_robot_controller_server = ce_robot.robot_controller_server:main',
            '02_robot_controller_client = ce_robot.robot_controller_client:main',
            
            # 03_Message
            '03_hw_status_publisher = ce_robot.HardwareStatus_publish:main',
            '03_hw_status_subscriber = ce_robot.HardwareStatus_subscribe:main',
            '03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main',
            '03_robot_status_publisher = ce_robot.RobotStatus_publisher:main',
            '03_robot_safety_monitor = ce_robot.RobotStatus_safety_monitor:main',
            '03_robot_fleet_coordinator = ce_robot.RobotStatus_fleet_coordinator:main',
            
            # 04_Service
            '04_CalRect_server = ce_robot.CalRect_server:main',
            '04_CalRect_client = ce_robot.CalRect_client:main',
            '04_navigate_server = ce_robot.navigate_to_position_server:main',
            '04_navigate_client = ce_robot.navigate_to_position_client:main',
            '04_gripper_server = ce_robot.gripper_control_server:main',
            '04_gripper_client = ce_robot.gripper_control_client:main',
            'cal_rect_server = ce_robot.CalRect_server:main',  # Alias for launch files
            
            # 05_Parameters
            '05_hw_status_param = ce_robot.HwStatus_para_publish:main',
            '05_robot_tag_param = ce_robot.robot_tag_param_pub:main',
            '05_robot_tag_callback = ce_robot.robot_tag_callback_pub:main',
            '05_robot_tag_validated = ce_robot.robot_tag_validated_pub:main',
            'hw_status_param_pub = ce_robot.hw_status_param_pub:main',  # For launch files
            'hw_status_callback_pub = ce_robot.hw_status_callback_pub:main',  # For launch files
            
            # 06_Action
            '06_count_until_server = ce_robot.count_until_server:main',
            '06_count_until_client = ce_robot.count_until_client:main',
            '06_battery_charging_server = ce_robot.battery_charging_server:main',
            '06_battery_charging_client = ce_robot.battery_charging_client:main',
            '06_navigate_server = ce_robot.navigate_server:main',
            '06_navigate_client = ce_robot.navigate_client:main',
            '06_gripper_server = ce_robot.gripper_server:main',
            '06_gripper_client = ce_robot.gripper_client:main',
        ],
    },
)

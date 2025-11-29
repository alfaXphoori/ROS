from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_hw_pub_topic              = ("hardware_status", "boot_hw_status")
    remap_cal_rect_server_topic     = ("cal_rect","boot_cal_rectangle")
    remap_hw_para_topic             = ("hardware_status_para", "boot_hw_parameter")
    remap_count_until_topic         = ("count_until_act", "boot_count_until")

    HardwareStatus_pub_node = Node(
        package="ce_robot",
        executable="hw_status",
        name="HWStatus_pub",
        remappings=[
            remap_hw_pub_topic
        ]
    )

    CalRect_server_node = Node(
        package="ce_robot",
        executable="cal_rect_server",
        name="CalRect_sv",
        remappings=[
            remap_cal_rect_server_topic
        ]
    )   
    
    HardwareStauts_para_node = Node(
        package="ce_robot",
        executable="hw_para",
        name="HWStatus_para",
        remappings=[
            remap_hw_para_topic
        ],
        parameters=[
            {"rb_name": "RB_CE_1"},
            {"rb_no": 1111}
        ]
    )

    Count_until_action_node = Node(
        package="ce_robot",
        executable="count_server",
        name="Count_until_act",
        parameters=[
            {"topic_name": "boot_count_until_act"}
        ]
    )
    
    ld.add_action(HardwareStatus_pub_node)
    ld.add_action(CalRect_server_node)
    ld.add_action(HardwareStauts_para_node)
    ld.add_action(Count_until_action_node)

    return ld

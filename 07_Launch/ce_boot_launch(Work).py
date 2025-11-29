from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_hw_pub_topic              = ("hardware_status", "_boot_hw_pub_phoori")
    remap_cal_rect_server_topic     = ("cal_rect","cal_rect")
    remap_hw_para_topic             = ("hardware_status_para", "boot_hw_para_phoori")

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
            {"_name": "Phoori_chantima"},
            {"_no": 99},
            {"_class": "CE6641"}
        ]
    )

    Count_until_action_node = Node(
        package="ce_robot",
        executable="count_until_server",
        name="Count_until_act",
        parameters=[
            {"topic_name": "boot_count_act_phoori"}
        ]
    )
    
    ld.add_action(HardwareStatus_pub_node)
    ld.add_action(CalRect_server_node)
    ld.add_action(HardwareStauts_para_node)
    ld.add_action(Count_until_action_node)

    return ld
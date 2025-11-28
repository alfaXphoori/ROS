from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_cal_rect_server_topic = ("cal_tri","cal_tri_angle")
    remap_hw_para_topic = ("hardware_status","hw_para_sta")
   
    Rb_info_Node = Node(
        package="ce_robot",
        executable="rb_info",
        name="rb_info_pub",
    )

    Cal_tri_server_node = Node(
        package="ce_robot",
        executable="cal_tri",
        name="cal_tri_sv",
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
            {"rb_name": "Robot_CE"},
            {"rb_no": 6789},
            {"rb_ready": True},
        ]
    )

    ld.add_action(Rb_info_Node)
    ld.add_action(Cal_tri_server_node)
    ld.add_action(HardwareStauts_para_node)
    return ld

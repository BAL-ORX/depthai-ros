import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    params_file = os.path.join(depthai_prefix, "config", "multicam_oakdprow2.yaml")
    cams = ["oak_d_pro_w1", "oak_d_pro_w2"]
    nodes = []
    for i, cam_name in enumerate(cams):
        node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": cam_name,
                "parent_frame": "map",
                "params_file": params_file,
                "cam_pos_y": str(-i* 0.1),
            }.items(),
        )
        nodes.append(node)
        i = i + 0.1

    return nodes


def generate_launch_description():

    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )

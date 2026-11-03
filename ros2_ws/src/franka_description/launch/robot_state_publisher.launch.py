"""robot_state_publisher for the sim's Franka: /tf of the panda links from /sim/joint_states, plus the static
world -> panda_link0 (the robot stands on a pedestal at the world origin).

    ros2 launch franka_description robot_state_publisher.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    urdf = Path(get_package_share_directory("franka_description"), "urdf", "panda.urdf").read_text()
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("joint_states", default_value="/sim/joint_states"),
        DeclareLaunchArgument("base_z", default_value="0.75",
                              description="panda_link0 height in world (robot.base_height in isaac_sim/config/cell.yaml)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf, "use_sim_time": use_sim_time}],
            remappings=[("joint_states", LaunchConfiguration("joint_states"))],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_panda_link0",
            arguments=["--z", LaunchConfiguration("base_z"), "--frame-id", "world", "--child-frame-id", "panda_link0"],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])

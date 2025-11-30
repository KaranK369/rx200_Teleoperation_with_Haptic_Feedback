from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_pkg = get_package_share_directory("omni_description")
    urdf_file = os.path.join(description_pkg, "urdf", "omni.urdf")
    rviz_config_path = os.path.join(description_pkg, "resources", "omni.rviz")

    nodes = []

    # Robot State Publisher (remap phantom -> joint_states)
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_file]), value_type=str
                )
            }],
            remappings=[("/joint_states", "/phantom/joint_states")]
        )
    )

    # Omni control node (publishes /phantom/joint_states)
    nodes.append(
        Node(
            package="omni_common",
            executable="omni_state",
            name="omni_haptic_node",
            output="screen",
            parameters=[
                {"publish_rate": 1000},
                {"reference_frame": "map"},
                {"units": "mm"}
            ]
        )
    )

    # RViz2 (load your config automatically)
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path]
        )
    )

    return LaunchDescription(nodes)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    # -------------------------
    # Launch Arguments
    # -------------------------
    haptic_arg = DeclareLaunchArgument(
        "haptic_feedback",
        default_value="false",
        description="Enable (true) or disable (false) haptic feedback (ff.py)"
    )

    haptic = LaunchConfiguration("haptic_feedback")

    # -------------------------
    # Zenoh Bridge Process
    # -------------------------
    zenoh_bridge = ExecuteProcess(
        cmd=[
            "zenoh-bridge-ros2dds",
            "--connect", "tcp/10.196.205.208:7447"
        ],
        output="screen"
    )

    # -------------------------
    # Including omni.launch.py
    # -------------------------
    omni_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                "omni.launch.py"
            )
        )
    )

    # -------------------------
    # Run ff.py only when haptic enabled
    # -------------------------
    def launch_ff(context, *args, **kwargs):
        if context.perform_substitution(haptic) == "true":
            return [
                ExecuteProcess(
                    cmd=["ros2", "run", "omni_common", "ff.py"],
                    output="screen"
                )
            ]
        else:
            print("[INFO] Haptic feedback disabled → Not launching ff.py")
            return []

    ff_launcher = OpaqueFunction(function=launch_ff)

    # -------------------------
    # Final Launch Description
    # -------------------------
    return LaunchDescription([
        haptic_arg,
        zenoh_bridge,
        omni_launch,
        ff_launcher
    ])


#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from math import atan2, asin, degrees
from omni_msgs.msg import OmniButtonEvent
from sensor_msgs.msg import JointState


roll_mapped = 0.0
pitch_remapped = 0.0

T = np.array([
    [0,  -1,  0,  0.2],     
    [1,  0,  0, 0],     
    [0,  0,  1,  0.1],    
    [0,  0,  0,  1]
])

# RX200 workspace limits for safety
WORKSPACE_LIMITS = {
    'x_min': 0.05,
    'x_max': 0.50,
    'y_min': -0.40,
    'y_max': 0.40,
    'z_min': 0.02,
    'z_max': 0.45,
}

def transform_point(p_geomagic):
    p_hom = np.array([p_geomagic[0], p_geomagic[1], p_geomagic[2]+0.06, 1.0])
    p_transformed = T @ p_hom
    return p_transformed[:3]

def clamp_to_workspace(x, y, z):
    """Clamp positions to safe workspace limits"""
    x_clamped = np.clip(x, WORKSPACE_LIMITS['x_min'], WORKSPACE_LIMITS['x_max'])
    y_clamped = np.clip(y, WORKSPACE_LIMITS['y_min'], WORKSPACE_LIMITS['y_max'])
    z_clamped = np.clip(z, WORKSPACE_LIMITS['z_min'], WORKSPACE_LIMITS['z_max'])
    return x_clamped, y_clamped, z_clamped

class PhantomPoseToInterbotix(Node):
    def __init__(self, bot):
        super().__init__('arm_control')
        self.bot = bot
        self.bot.gripper.release()
        self.last_pose_time = self.get_clock().now()
        self.last_joint_time = self.get_clock().now()
        
        self.white_button_pressed = False
        self.grey_button_pressed = False

        self.subscription = self.create_subscription(
            PoseStamped,
            '/phantom/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /phantom/pose")

        self.button_sub = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState,
            'phantom/joint_states',
            self.joint_callback,
            10
        )

    def button_callback(self, msg: OmniButtonEvent):
        grey = msg.grey_button
        white = msg.white_button

        # Gripper control - close on grey press, open on grey release
        if grey == 1:
            self.bot.gripper.grasp(delay=0.0)
            self.get_logger().info("Gripper closed.")
        else:
            self.bot.gripper.release(delay=0.0)

        # Update button states
        self.white_button_pressed = white == 1
        self.grey_button_pressed = grey == 1

    def pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now()
        if (now - self.last_pose_time).nanoseconds < 20_000_000:  # 50 Hz
            return
        self.last_pose_time = now

        pos = msg.pose.position
        p_geomagic = [pos.x, pos.y, pos.z]
        p_inter = transform_point(p_geomagic)
        
        # Keep original scaling (x2)
        x_raw = p_inter[0] * 2
        y_raw = p_inter[1] * 2
        z_raw = p_inter[2]
        
        # Apply safety clamping
        x_safe, y_safe, z_safe = clamp_to_workspace(x_raw, y_raw, z_raw)

        # Determine roll based on button state
        if self.white_button_pressed:
            # White button: use roll from Geomagic
            use_roll = roll_mapped
        else:
            # No white button (no buttons or grey only): roll = 0
            use_roll = 0.0

        # Move arm with appropriate roll
        self.bot.arm.set_ee_pose_components(
            x=x_safe,
            y=y_safe,
            z=z_safe,
            roll=use_roll,
            pitch=pitch_remapped,
            moving_time=0.3,
            accel_time=0.6,
            blocking=False
        )

    def joint_callback(self, msg: JointState):
        now = self.get_clock().now()
        if (now - self.last_joint_time).nanoseconds < 20_000_000:  # 50 Hz
            return
        self.last_joint_time = now
        
        name_to_pos = dict(zip(msg.name, msg.position))

        roll = name_to_pos.get('roll', 0.0)
        pitch = name_to_pos.get('pitch', 0.0)

        roll += 2.61
        global roll_mapped
        roll_mapped = roll
        
        pitch += 2.60
        global pitch_remapped
        pitch_remapped = pitch


def main(args=None):
    rclpy.init(args=args)

    # Start Interbotix Robot
    bot = InterbotixManipulatorXS(
        robot_model='rx200',
        robot_name='rx200',
        group_name='arm',
        gripper_name='gripper'
    )
    robot_startup()

    node = PhantomPoseToInterbotix(bot)
    
    print("\n" + "="*50)
    print("Geomagic Touch to RX200 Teleoperation")
    print("="*50)
    print("\nControls:")
    print("  No buttons: Free movement (roll=0)")
    print("  Grey button: Close gripper + free movement (roll=0)")
    print("  White button: Free movement with roll from Geomagic")
    print("="*50 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        robot_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

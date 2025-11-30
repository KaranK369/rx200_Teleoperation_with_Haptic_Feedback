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
from std_msgs.msg import Bool


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
    'x_max': 0.50,   # Increased from 0.25
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
        super().__init__('phantom_to_interbotix_node')
        self.bot = bot
        self.bot.gripper.release()
        self.last_pose_time = self.get_clock().now()
        self.last_joint_time = self.get_clock().now()
        
        self.white_button_pressed = False
        self.grey_button_pressed = False
        
        # Store x1 for continuity
        self.x1 = 0.25

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

        self.leader_valid_pub = self.create_publisher(Bool, '/pose_valid_leader', 10)
        self.mimic_valid = True
        self.create_subscription(Bool, '/pose_valid_mimic', self.mimic_valid_cb, 10)
        
        self.get_logger().info("Enhanced workspace - limits removed, safety clamping active")

    def mimic_valid_cb(self, msg: Bool):
        self.mimic_valid = msg.data

    def button_callback(self, msg: OmniButtonEvent):
        grey = msg.grey_button
        white = msg.white_button

        if grey == 1 and white == 0:
            self.bot.gripper.grasp(delay=0.0)
            self.get_logger().info("Gripper closed.")
        else:
            self.bot.gripper.release(delay=0.0)

        # Update button states
        self.white_button_pressed = white == 1
        self.grey_button_pressed = grey == 1

    def pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now()
        if (now - self.last_pose_time).nanoseconds < 20_000_000:  # 50 Hz = 0.02 sec
            return
        self.last_pose_time = now

        pos = msg.pose.position
        p_geomagic = [pos.x, pos.y, pos.z]
        p_inter = transform_point(p_geomagic)
        
        # Keep your original scaling (x2) that was working
        x_raw = p_inter[0] * 2
        y_raw = p_inter[1] * 2
        z_raw = p_inter[2]
        
        # Apply safety clamping instead of hard limits
        x_safe, y_safe, z_safe = clamp_to_workspace(x_raw, y_raw, z_raw)

        if not self.white_button_pressed and not self.grey_button_pressed:
            # Free movement mode
            self.x1 = x_safe  # Update x1 for continuity
            
            self.bot.arm.set_ee_pose_components(
                x=x_safe,
                y=y_safe,
                z=z_safe,
                roll=0.0,
                pitch=pitch_remapped,
                moving_time=0.3,
                accel_time=0.6,
                blocking=False
            )
            
        elif not self.white_button_pressed and self.grey_button_pressed:
            # Coordinated movement mode
            if not self.mimic_valid:
                return

            _, success = self.bot.arm.set_ee_pose_components(
                x=x_safe,
                y=y_safe,
                z=z_safe,
                roll=roll_mapped,
                pitch=pitch_remapped,
                blocking=False
            )
            self.leader_valid_pub.publish(Bool(data=success))
            
        elif self.white_button_pressed and self.grey_button_pressed:
            # Both buttons pressed
            self.x1 = x_safe  # Update x1
            
            self.bot.arm.set_ee_pose_components(
                x=self.x1,
                y=y_safe,
                z=z_safe,
                roll=roll_mapped,
                pitch=pitch_remapped,
                blocking=False
            )
            
        elif self.white_button_pressed and not self.grey_button_pressed:
            # White button only
            self.x1 = x_safe  # Update x1
            
            self.bot.arm.set_ee_pose_components(
                x=self.x1,
                y=y_safe,
                z=z_safe,
                roll=roll_mapped,
                pitch=pitch_remapped,
                blocking=False
            )

    def joint_callback(self, msg: JointState):
        now = self.get_clock().now()
        if (now - self.last_joint_time).nanoseconds < 20_000_000:  # 50 Hz = 0.02 sec
            return
        self.last_joint_time = now
        
        if not (not self.white_button_pressed and self.grey_button_pressed):
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
    print("Teleoperation with Extended Workspace")
    print("="*50)
    print("Changes:")
    print("  ✓ Removed 0.25m X-axis limit")
    print("  ✓ Extended to full robot workspace")
    print("  ✓ Kept original scaling (2x) for sync")
    print("  ✓ Added safety clamping at boundaries")
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

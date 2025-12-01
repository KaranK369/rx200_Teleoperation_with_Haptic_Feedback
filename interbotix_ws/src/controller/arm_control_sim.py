#!/usr/bin/env python3

"""
Teleoperation from Geomagic Touch to RX200 using Modern Robotics IK
Matches Interbotix SDK behavior exactly
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from math import atan2, sqrt, acos, sin, cos, pi
from omni_msgs.msg import OmniButtonEvent
from std_msgs.msg import Bool
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
import interbotix_common_modules.angle_manipulation as ang


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


class ModernRoboticsIKController(Node):
    def __init__(self):
        super().__init__('arm_control_sim')
        
        # Load RX200 robot description from Modern Robotics
        self.robot_des = mrd.rx200
        
        # Number of joints
        self.num_joints = self.robot_des.Slist.shape[1]
        
        # Initialize IK initial guesses (same as Interbotix SDK)
        self.initial_guesses = [[0.0] * self.num_joints for _ in range(3)]
        self.initial_guesses[1][0] = np.deg2rad(-120)
        self.initial_guesses[2][0] = np.deg2rad(120)
        
        # Joint limits for RX200 (from URDF/motor config)
        self.joint_lower_limits = [-3.14159, -1.88, -2.146, -1.8, -3.14159]
        self.joint_upper_limits = [3.14159, 1.5, 1.617, 2.146, 3.14159]
        self.joint_velocity_limits = [3.14159, 3.14159, 3.14159, 3.14159, 3.14159]
        
        # Current joint commands
        self.joint_commands = [0.0] * self.num_joints
        
        # Moving time for trajectory
        self.moving_time = 0.3
        
        # Arm trajectory action client
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/rx200/arm_controller/follow_joint_trajectory'
        )
        self.arm_action_client.wait_for_server()
        self.get_logger().info("Arm controller connected!")
        
        # Gripper action client - try FollowJointTrajectory for gripper
        self.gripper_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/rx200/gripper_controller/follow_joint_trajectory'
        )
        
        self.gripper_joint_names = ['left_finger', 'right_finger']
        
        # Joint names for RX200
        self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
        
        # Subscribe to joint states to get current positions
        self.current_joint_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/rx200/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.last_pose_time = self.get_clock().now()
        self.last_joint_time = self.get_clock().now()
        
        self.white_button_pressed = False
        self.grey_button_pressed = False
        
        # Store x1 for continuity
        self.x1 = 0.25
        
        # Current gripper state
        self.gripper_closed = False
        
        # Subscriptions
        self.pose_sub = self.create_subscription(
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
        
        self.phantom_joint_sub = self.create_subscription(
            JointState,
            'phantom/joint_states',
            self.phantom_joint_callback,
            10
        )


        
        # Open gripper on startup
        self.create_timer(1.0, self.open_gripper_on_startup)
        self.gripper_opened_on_startup = False
        
        self.get_logger().info("Modern Robotics IK controller initialized!")

    def joint_state_callback(self, msg: JointState):
        """Store current joint positions"""
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == self.num_joints:
            self.current_joint_positions = positions
            self.joint_commands = list(positions)


    def open_gripper_on_startup(self):
        """Open gripper once on startup"""
        if not self.gripper_opened_on_startup:
            self.control_gripper(close=False)
            self.gripper_opened_on_startup = True
            self.get_logger().info("Gripper opened on startup")

    def control_gripper(self, close=True):
        """Control gripper using FollowJointTrajectory"""
        if not self.gripper_action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().debug("Gripper action server not available")
            return
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.gripper_joint_names
        
        point = JointTrajectoryPoint()
        if close:
            # Closed position
            position = 0.0
        else:
            # Open position  
            position = 0.037
        
        point.positions = [position, -position]  # Left positive, right negative
        point.time_from_start = Duration(sec=1, nanosec=0)
        goal_msg.trajectory.points = [point]
        
        self.gripper_action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Gripper {'closed' if close else 'opened'}")

    def button_callback(self, msg: OmniButtonEvent):
        grey = msg.grey_button
        white = msg.white_button

        # Control gripper - close on grey press, open on grey release
        if grey == 1:
            # Grey button pressed - close gripper
            if not self.gripper_closed:
                self.control_gripper(close=True)
                self.gripper_closed = True
        else:
            # Grey button released - open gripper
            if self.gripper_closed:
                self.control_gripper(close=False)
                self.gripper_closed = False

        # Update button states
        self.white_button_pressed = white == 1
        self.grey_button_pressed = grey == 1

    def _check_joint_limits(self, positions):
        """Check if joint positions are within limits (from Interbotix SDK)"""
        # Reject NaN values
        if any(np.isnan(elem) for elem in positions):
            return False
        
        theta_list = [int(elem * 1000) / 1000.0 for elem in positions]
        speed_list = [
            abs(goal - current) / float(self.moving_time)
            for goal, current in zip(theta_list, self.joint_commands)
        ]
        
        # Check position and velocity limits
        for x in range(self.num_joints):
            if not (self.joint_lower_limits[x] <= theta_list[x] <= self.joint_upper_limits[x]):
                return False
            if speed_list[x] > self.joint_velocity_limits[x]:
                return False
        return True

    def _wrap_theta_list(self, theta_list):
        """Wrap joint angles (from Interbotix SDK)"""
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] += REV
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] -= REV
        return theta_list

    def set_ee_pose_components(self, x, y, z, roll, pitch, yaw=None):
        """
        Solve IK for end-effector pose using Modern Robotics (from Interbotix SDK)
        
        Args:
            x, y, z: Position in meters
            roll, pitch: Orientation in radians
            yaw: Auto-calculated for <6DOF arms
        
        Returns:
            (joint_positions, success): Joint solution and success flag
        """
        # For arms with < 6 DOF, auto-calculate yaw
        if self.num_joints < 6 or (self.num_joints >= 6 and yaw is None):
            yaw = atan2(y, x)
        
        # Build transformation matrix
        T_sd = np.identity(4)
        T_sd[:3, :3] = ang.euler_angles_to_rotation_matrix([roll, pitch, yaw])
        T_sd[:3, 3] = [x, y, z]
        
        # Try IK with multiple initial guesses
        for guess in self.initial_guesses:
            theta_list, success = mr.IKinSpace(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=0.001,
                ev=0.001,
            )
            
            solution_found = True
            
            # Check if solution was found and within limits
            if success:
                theta_list = self._wrap_theta_list(theta_list)
                solution_found = self._check_joint_limits(theta_list)
            else:
                solution_found = False
            
            if solution_found:
                return list(theta_list), True
        
        # No valid solution found
        return None, False

    def send_joint_trajectory(self, joint_positions):
        """Send joint trajectory to arm controller"""
        if joint_positions is None:
            return False
        
        try:
            # Create trajectory goal
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = Duration(sec=0, nanosec=int(self.moving_time * 1e9))
            
            goal_msg.trajectory.points = [point]
            
            # Send goal
            self.arm_action_client.send_goal_async(goal_msg)
            
            # Update joint commands
            self.joint_commands = list(joint_positions)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending trajectory: {e}")
            return False

    def pose_callback(self, msg: PoseStamped):
        """Handle incoming pose from Geomagic Touch"""
        now = self.get_clock().now()
        if (now - self.last_pose_time).nanoseconds < 50_000_000:  # 20 Hz
            return
        self.last_pose_time = now

        if self.current_joint_positions is None:
            return

        pos = msg.pose.position
        p_geomagic = [pos.x, pos.y, pos.z]
        p_inter = transform_point(p_geomagic)
        
        # Keep original scaling (x2)
        x_raw = p_inter[0] * 2
        y_raw = p_inter[1] * 2
        z_raw = p_inter[2]
        
        # Apply safety clamping
        x_safe, y_safe, z_safe = clamp_to_workspace(x_raw, y_raw, z_raw)
        
        # Debug output matching original code format
        debug_msg = f"\n--- Pose Debug ---\n"
        debug_msg += f"Geomagic Raw:        x={p_geomagic[0]:.3f}, y={p_geomagic[1]:.3f}, z={p_geomagic[2]:.3f}\n"
        debug_msg += f"Transformed (T):     x={p_inter[0]:.3f}, y={p_inter[1]:.3f}, z={p_inter[2]:.3f}\n"
        debug_msg += f"Scaled Raw:          x={x_raw:.3f}, y={y_raw:.3f}, z={z_raw:.3f}\n"
        debug_msg += f"Clamped Safe Pose:   x={x_safe:.3f}, y={y_safe:.3f}, z={z_safe:.3f}\n"
        debug_msg += f"roll={roll_mapped:.3f}, pitch={pitch_remapped:.3f}\n"
        debug_msg += f"Buttons: white={self.white_button_pressed}, grey={self.grey_button_pressed}"

        if not self.white_button_pressed and not self.grey_button_pressed:
            # Free movement mode
            self.x1 = x_safe
            joint_positions, success = self.set_ee_pose_components(
                x=x_safe, 
                y=y_safe, 
                z=z_safe, 
                roll=0.0, 
                pitch=pitch_remapped
            )
            if success:
                self.get_logger().info(debug_msg)
                self.get_logger().info(f"IK Success - Joints: {[f'{j:.3f}' for j in joint_positions]}")
                self.send_joint_trajectory(joint_positions)
            else:
                self.get_logger().warn(f"No valid pose could be found. x={x_safe:.3f}, y={y_safe:.3f}, z={z_safe:.3f}")
            
        elif not self.white_button_pressed and self.grey_button_pressed:
 
            joint_positions, success = self.set_ee_pose_components(
                x=x_safe,
                y=y_safe,
                z=z_safe,
                roll=0.0,
                pitch=pitch_remapped
            )
            if success:
                self.send_joint_trajectory(joint_positions)
            self.leader_valid_pub.publish(Bool(data=success))
            
        elif self.white_button_pressed and self.grey_button_pressed:
            # Both buttons pressed
            self.x1 = x_safe
            joint_positions, success = self.set_ee_pose_components(
                x=self.x1,
                y=y_safe,
                z=z_safe,
                roll=roll_mapped,
                pitch=pitch_remapped
            )
            if success:
                self.send_joint_trajectory(joint_positions)
            
        elif self.white_button_pressed and not self.grey_button_pressed:
            # White button only
            self.x1 = x_safe
            joint_positions, success = self.set_ee_pose_components(
                x=self.x1,
                y=y_safe,
                z=z_safe,
                roll=roll_mapped,
                pitch=pitch_remapped
            )
            if success:
                self.send_joint_trajectory(joint_positions)

    def phantom_joint_callback(self, msg: JointState):
        """Handle joint state updates from Geomagic Touch"""
        now = self.get_clock().now()
        if (now - self.last_joint_time).nanoseconds < 20_000_000:
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

    node = ModernRoboticsIKController()
    
    print("\n" + "="*50)
    print("Modern Robotics IK Teleoperation")
    print("="*50)
    print("Features:")
    print("  ✓ Uses Interbotix Modern Robotics IK")
    print("  ✓ Exact same algorithm as SDK")
    print("  ✓ Direct trajectory control")
    print("  ✓ Full workspace (0.05-0.50m X-axis)")
    print("="*50)
    print("\nControls:")
    print("  Grey Button: Close gripper")
    print("  No Buttons: Free movement")
    print("  White + Grey: Constrained X-axis")
    print("="*50 + "\n")
    
    print("Waiting for joint states...")
    while node.current_joint_positions is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    print("Joint states received! Ready to go.\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

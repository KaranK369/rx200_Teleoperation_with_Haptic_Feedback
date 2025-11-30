#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from omni_msgs.msg import OmniFeedback, OmniState
import numpy as np
from collections import deque
import time

class HapticFeedbackController(Node):
    def __init__(self):
        super().__init__('haptic_feedback_controller')
        
        # Publishers
        self.feedback_pub = self.create_publisher(
            OmniFeedback,
            '/phantom/force_feedback',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/rx200/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.omni_state_sub = self.create_subscription(
            OmniState,
            '/phantom/state',
            self.omni_state_callback,
            10
        )
        
        self.omni_pose_sub = self.create_subscription(
            Pose,
            '/phantom/pose',
            self.omni_pose_callback,
            10
        )
        
        # Parameters for collision detection
        self.declare_parameter('collision_force', 5.0)  # Newton
        self.declare_parameter('joint_limit_threshold', 0.08)  # radians from limit
        self.declare_parameter('joint_limit_force', 4.0)  # Newton
        
        # Effort change detection (collision causes sudden spike)
        # Per-joint thresholds to handle different effort levels
        self.declare_parameter('waist_threshold', 50.0)  # Lower threshold for low-effort joint
        self.declare_parameter('shoulder_threshold', 60.0)  # Lower threshold for low-effort joint
        self.declare_parameter('elbow_threshold', 150.0)  # Higher threshold for high-effort joint
        self.declare_parameter('effort_window_size', 8)  # samples for smoother baseline
        self.declare_parameter('collision_duration', 0.5)  # seconds to maintain feedback
        
        # State variables
        self.robot_joint_positions = None
        self.robot_joint_velocities = None
        self.robot_joint_efforts = None
        self.omni_position = None
        self.omni_pose = None
        self.previous_omni_pose = None
        
        # Effort tracking for spike detection
        self.effort_history = {}  # Rolling window of efforts
        self.effort_smooth = {}   # Smoothed effort values
        
        # Joint limits
        self.joint_limits = {
            'waist': (-3.14, 3.14),
            'shoulder': (-1.88, 1.99),
            'elbow': (-2.15, 1.60),
            'wrist_angle': (-1.74, 2.15),
            'wrist_rotate': (-3.14, 3.14)
        }
        
        # Feedback state tracking
        self.collision_detected = False
        self.collision_start_time = None
        self.collision_joint = None
        self.collision_direction = None
        self.last_log_time = 0
        
        self.get_logger().info('Haptic Feedback Controller initialized')
        self.get_logger().info('Collision detection: Only WAIST, SHOULDER, ELBOW')
        self.get_logger().info('Ignored joints: wrist_angle, wrist_rotate, gripper, fingers')
        
    def joint_state_callback(self, msg):
        """Process robot joint states"""
        self.robot_joint_positions = dict(zip(msg.name, msg.position))
        self.robot_joint_velocities = dict(zip(msg.name, msg.velocity))
        self.robot_joint_efforts = dict(zip(msg.name, msg.effort))
        
        # Update effort tracking
        self.update_effort_tracking()
        
        # Calculate and send haptic feedback
        self.calculate_haptic_feedback()
    
    def omni_state_callback(self, msg):
        """Store current Geomagic Touch state"""
        self.omni_position = msg.pose.position
        
    def omni_pose_callback(self, msg):
        """Track controller pose"""
        self.previous_omni_pose = self.omni_pose
        self.omni_pose = msg
    
    def update_effort_tracking(self):
        """Maintain rolling window of joint efforts for spike detection"""
        window_size = self.get_parameter('effort_window_size').value
        
        # Only track these joints (exclude wrist and gripper)
        tracked_joints = ['waist', 'shoulder', 'elbow']
        
        for joint_name, effort in self.robot_joint_efforts.items():
            # Skip joints we don't want feedback from
            if joint_name not in tracked_joints:
                continue
            
            # Initialize history if needed
            if joint_name not in self.effort_history:
                self.effort_history[joint_name] = deque(maxlen=window_size)
                self.effort_smooth[joint_name] = effort
            
            # Add to history
            self.effort_history[joint_name].append(effort)
            
            # Calculate smoothed value (moving average)
            if len(self.effort_history[joint_name]) >= window_size:
                self.effort_smooth[joint_name] = np.mean(self.effort_history[joint_name])
    
    def detect_collision_or_obstacle(self):
        """
        Detect collision by checking for SUDDEN SPIKES in joint efforts
        Only monitors: waist, shoulder, elbow (with per-joint thresholds)
        """
        current_time = time.time()
        collision_duration = self.get_parameter('collision_duration').value
        
        # Get per-joint thresholds
        thresholds = {
            'waist': self.get_parameter('waist_threshold').value,
            'shoulder': self.get_parameter('shoulder_threshold').value,
            'elbow': self.get_parameter('elbow_threshold').value
        }
        
        # If we're in a collision state, maintain it for duration
        if self.collision_detected:
            if current_time - self.collision_start_time < collision_duration:
                # Still in collision, return same force
                return True, self.collision_direction, f"Collision on {self.collision_joint}"
            else:
                # Collision duration expired, clear it
                self.collision_detected = False
                self.collision_start_time = None
                self.collision_joint = None
                self.collision_direction = None
        
        # Check for new collision (effort spike) - only on tracked joints
        for joint_name in ['waist', 'shoulder', 'elbow']:
            if joint_name not in self.robot_joint_efforts or joint_name not in self.effort_smooth:
                continue
            
            current_effort = self.robot_joint_efforts[joint_name]
            smooth_effort = self.effort_smooth[joint_name]
            
            # Calculate sudden change in effort (absolute value)
            effort_spike = abs(abs(current_effort) - abs(smooth_effort))
            
            # Get threshold for this specific joint
            threshold = thresholds.get(joint_name, 100.0)
            
            # Detect collision via sudden spike
            if effort_spike > threshold:
                # Collision detected!
                self.collision_detected = True
                self.collision_start_time = current_time
                self.collision_joint = joint_name
                self.collision_direction = self.map_joint_to_cartesian_direction(joint_name, current_effort)
                
                return True, self.collision_direction, f"Collision on {joint_name}: spike {effort_spike:.1f} Nm (threshold: {threshold:.1f})"
        
        # Check for joint limits (only for tracked joints)
        joint_limit_threshold = self.get_parameter('joint_limit_threshold').value
        
        for joint_name in ['waist', 'shoulder', 'elbow']:
            if joint_name in self.joint_limits and joint_name in self.robot_joint_positions:
                min_limit, max_limit = self.joint_limits[joint_name]
                pos = self.robot_joint_positions[joint_name]
                
                # Near minimum limit
                if pos < min_limit + joint_limit_threshold:
                    force_direction = self.map_joint_limit_to_force(joint_name, "min")
                    return True, force_direction, f"{joint_name} at MIN limit: {pos:.3f} rad"
                
                # Near maximum limit
                if pos > max_limit - joint_limit_threshold:
                    force_direction = self.map_joint_limit_to_force(joint_name, "max")
                    return True, force_direction, f"{joint_name} at MAX limit: {pos:.3f} rad"
        
        return False, (0.0, 0.0, 0.0), ""
    
    def map_joint_to_cartesian_direction(self, joint_name, effort):
        """Map joint effort to Cartesian force direction (opposing motion)
        Only for waist, shoulder, and elbow"""
        force_magnitude = self.get_parameter('collision_force').value
        
        # Map based on joint - force opposes the attempted motion
        if joint_name == 'waist':
            return (-np.sign(effort) * force_magnitude, 0.0, 0.0)
        elif joint_name == 'shoulder':
            return (0.0, 0.0, -np.sign(effort) * force_magnitude)
        elif joint_name == 'elbow':
            return (0.0, -np.sign(effort) * force_magnitude, 0.0)
        else:
            return (0.0, 0.0, 0.0)  # Should never reach here
    
    def map_joint_limit_to_force(self, joint_name, limit_type):
        """Map joint limit to repulsive force direction
        Only for waist, shoulder, and elbow"""
        force_magnitude = self.get_parameter('joint_limit_force').value
        
        # Force pushes away from limit
        direction = 1.0 if limit_type == "min" else -1.0
        
        if joint_name == 'waist':
            return (direction * force_magnitude, 0.0, 0.0)
        elif joint_name == 'shoulder':
            return (0.0, 0.0, direction * force_magnitude)
        elif joint_name == 'elbow':
            return (0.0, direction * force_magnitude, 0.0)
        else:
            return (0.0, 0.0, 0.0)  # Should never reach here
    
    def calculate_haptic_feedback(self):
        """Calculate forces to send to haptic device - only on collision/obstacle"""
        if self.robot_joint_positions is None or self.robot_joint_efforts is None:
            return
        
        feedback = OmniFeedback()
        
        # Always set position to zero
        feedback.position.x = 0.0
        feedback.position.y = 0.0
        feedback.position.z = 0.0
        
        # Detect collision or obstacle
        is_obstacle, force_direction, reason = self.detect_collision_or_obstacle()
        
        current_time = time.time()
        
        if is_obstacle:
            # Apply feedback force
            feedback.force.x = force_direction[0]
            feedback.force.y = force_direction[1]
            feedback.force.z = force_direction[2]
            
            # Log feedback info (throttled to once per second)
            if current_time - self.last_log_time > 1.0:
                self.get_logger().warn('=' * 60)
                self.get_logger().warn('⚠️  COLLISION DETECTED - HAPTIC FEEDBACK ACTIVE')
                self.get_logger().warn(f'Reason: {reason}')
                self.get_logger().warn(f'Force: X={force_direction[0]:.2f} Y={force_direction[1]:.2f} Z={force_direction[2]:.2f} N')
                self.get_logger().warn('=' * 60)
                self.last_log_time = current_time
        else:
            # No collision - zero force
            feedback.force.x = 0.0
            feedback.force.y = 0.0
            feedback.force.z = 0.0
        
        # Publish feedback
        self.feedback_pub.publish(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = HapticFeedbackController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

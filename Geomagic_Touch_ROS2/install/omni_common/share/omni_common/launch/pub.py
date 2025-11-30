#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3, Wrench
from std_msgs.msg import Float64MultiArray, String

class ForceFeedbackMapper(Node):
    def __init__(self):
        super().__init__('force_feedback_mapper')
        
        # Joint indices for waist, shoulder, elbow
        self.joint_indices = {
            'waist': 0,
            'shoulder': 1,
            'elbow': 2
        }
        
        # Interbotix RX200 DH parameters (approximate)
        # Link lengths in meters
        self.L1 = 0.05  # waist to shoulder height
        self.L2 = 0.20  # shoulder to elbow length
        self.L3 = 0.20  # elbow to wrist length
        
        # Maximum force limit (2N)
        self.max_force = 2.0
        
        # Store previous values for collision detection
        self.prev_effort = np.zeros(3)
        self.prev_velocity = np.zeros(3)
        self.prev_time = self.get_clock().now()
        
        # Collision detection parameters - VERY AGGRESSIVE
        self.vel_threshold = 0.1  # rad/s - velocity considered as "stopped"
        self.effort_increase_threshold = 2.0  # VERY LOW - detect any resistance
        self.moving_vel_threshold = 0.03  # VERY LOW - detect any movement
        self.effort_absolute_threshold = 10.0  # If effort > this, always consider it
        
        # Scaling factor for effort to force conversion - INCREASED
        self.effort_to_force_scale = 0.05  # INCREASED from 0.015
        
        # State tracking
        self.was_moving = np.array([False, False, False])
        self.collision_detected = False
        self.current_positions = np.zeros(3)
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/rx200/joint_states',
            self.joint_state_callback,
            10)
        
        # Publisher for Geomagic Touch
        self.haptic_pub = self.create_publisher(
            Wrench,
            '/phantom/force_feedback',
            10)
        
        # Debug publishers
        self.debug_pub = self.create_publisher(
            Float64MultiArray,
            '/force_feedback/debug',
            10)
        
        self.force_vector_pub = self.create_publisher(
            String,
            '/force_feedback/force_vector',
            10)
        
        self.joint_monitor_pub = self.create_publisher(
            String,
            '/force_feedback/joint_monitor',
            10)
        
        # Timer for continuous monitoring (10Hz)
        self.monitor_timer = self.create_timer(0.1, self.monitor_callback)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Force Feedback Mapper - Interbotix RX200")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Monitoring joints: waist, shoulder, elbow")
        self.get_logger().info(f"Maximum force limit: {self.max_force}N")
        self.get_logger().info(f"Effort scaling: {self.effort_to_force_scale}")
        self.get_logger().info(f"Velocity threshold: {self.vel_threshold} rad/s")
        self.get_logger().info(f"Effort threshold: {self.effort_increase_threshold}")
        self.get_logger().info("Force feedback only on collision/resistance detection")
        self.get_logger().info("Monitor topics:")
        self.get_logger().info("  - /force_feedback/joint_monitor (continuous)")
        self.get_logger().info("  - /force_feedback/force_vector (on collision)")
        self.get_logger().info("=" * 60)
    
    def monitor_callback(self):
        """Continuous monitoring of joint states"""
        if hasattr(self, 'prev_velocity') and hasattr(self, 'prev_effort'):
            monitor_msg = String()
            monitor_msg.data = (
                f"Vel:[{self.prev_velocity[0]:6.3f},{self.prev_velocity[1]:6.3f},{self.prev_velocity[2]:6.3f}] | "
                f"Eff:[{self.prev_effort[0]:7.1f},{self.prev_effort[1]:7.1f},{self.prev_effort[2]:7.1f}] | "
                f"Moving:[{int(self.was_moving[0])},{int(self.was_moving[1])},{int(self.was_moving[2])}]"
            )
            self.joint_monitor_pub.publish(monitor_msg)
    
    def compute_jacobian(self, q):
        """
        Compute analytical Jacobian for Interbotix RX200 (3 DOF)
        q: joint angles [waist, shoulder, elbow]
        Returns: 3x3 Jacobian matrix mapping joint velocities to end-effector velocities
        """
        q1, q2, q3 = q[0], q[1], q[2]
        
        # Precompute trigonometric values
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)
        
        # Jacobian for RX200 (simplified 3DOF manipulator)
        # Each column represents contribution of joint i to end-effector velocity
        J = np.array([
            # dx/dq1, dx/dq2, dx/dq3
            [-(self.L2*s2 + self.L3*s23)*s1, (self.L2*c2 + self.L3*c23)*c1, self.L3*c23*c1],
            # dy/dq1, dy/dq2, dy/dq3
            [(self.L2*s2 + self.L3*s23)*c1, (self.L2*c2 + self.L3*c23)*s1, self.L3*c23*s1],
            # dz/dq1, dz/dq2, dz/dq3
            [0, -(self.L2*s2 + self.L3*s23), -self.L3*s23]
        ])
        
        return J
    
    def extract_joint_data(self, msg):
        """Extract velocity and effort for waist, shoulder, elbow"""
        vel = np.array([
            msg.velocity[self.joint_indices['waist']],
            msg.velocity[self.joint_indices['shoulder']],
            msg.velocity[self.joint_indices['elbow']]
        ])
        
        eff = np.array([
            msg.effort[self.joint_indices['waist']],
            msg.effort[self.joint_indices['shoulder']],
            msg.effort[self.joint_indices['elbow']]
        ])
        
        pos = np.array([
            msg.position[self.joint_indices['waist']],
            msg.position[self.joint_indices['shoulder']],
            msg.position[self.joint_indices['elbow']]
        ])
        
        return vel, eff, pos
    
    def detect_collision(self, velocity, effort):
        """
        Detect collision: joint was moving, suddenly velocity goes to 0,
        and effort keeps increasing.
        Returns meaningful torque values only during collision/resistance.
        """
        torque = np.zeros(3)
        collision_detected = False
        collision_joints = []
        
        for i in range(3):
            vel = velocity[i]
            eff = effort[i]
            prev_vel = self.prev_velocity[i]
            prev_eff = self.prev_effort[i]
            
            # Check if joint was moving
            if abs(prev_vel) > self.moving_vel_threshold:
                self.was_moving[i] = True
            
            # Collision detection logic:
            # 1. Joint was moving
            # 2. Velocity suddenly drops to near zero
            # 3. Effort is increasing (resistance)
            if self.was_moving[i]:
                velocity_dropped = abs(vel) < self.vel_threshold
                effort_increasing = abs(eff) > abs(prev_eff) + 0.5  # Small margin
                significant_effort = abs(eff) > self.effort_increase_threshold
                
                if velocity_dropped and effort_increasing and significant_effort:
                    # Collision detected! Use absolute effort value
                    torque[i] = abs(eff)
                    collision_detected = True
                    joint_names = ['waist', 'shoulder', 'elbow']
                    collision_joints.append(joint_names[i])
                elif not velocity_dropped:
                    # Still moving normally
                    self.was_moving[i] = abs(vel) > self.moving_vel_threshold
                    torque[i] = 0.0
                else:
                    torque[i] = 0.0
            else:
                # Joint wasn't moving, no collision
                torque[i] = 0.0
                # Update moving state
                if abs(vel) > self.moving_vel_threshold:
                    self.was_moving[i] = True
        
        if collision_detected:
            self.get_logger().warn(
                f"🔴 COLLISION on {collision_joints}: "
                f"torques=[{torque[0]:.1f}, {torque[1]:.1f}, {torque[2]:.1f}]"
            )
        
        return torque, collision_detected
    
    def torque_to_force(self, torque, positions):
        """
        Map joint torques to Cartesian force using Jacobian transpose.
        F = J^T * τ
        """
        # Compute current Jacobian based on joint positions
        J = self.compute_jacobian(positions)
        
        # Apply scaling to torques
        scaled_torque = torque * self.effort_to_force_scale
        
        # Map through Jacobian transpose: F = J^T * tau
        force = np.dot(J.T, scaled_torque)
        
        return force, J
    
    def limit_force(self, force):
        """Limit force magnitude to max_force (2N)"""
        magnitude = np.linalg.norm(force)
        
        if magnitude > self.max_force:
            # Scale down to maximum
            force = force * (self.max_force / magnitude)
            limited = True
        else:
            limited = False
        
        return force, magnitude, limited
    
    def joint_state_callback(self, msg):
        """Main callback to process joint states and publish force feedback"""
        try:
            # Extract relevant joint data
            velocity, effort, positions = self.extract_joint_data(msg)
            self.current_positions = positions
            
            # Detect collision and compute meaningful torque values
            torque, collision_detected = self.detect_collision(velocity, effort)
            
            # Only compute force if collision detected
            if collision_detected or np.any(torque > 0):
                # Map torque to Cartesian force using RX200 Jacobian
                force, jacobian = self.torque_to_force(torque, positions)
                
                # Apply force limit
                force_limited, force_mag, was_limited = self.limit_force(force)
                
                # Log force vector being sent - SIMPLIFIED
                force_info = (
                    f"FORCE ACTIVE | "
                    f"Torques:[{torque[0]:.1f},{torque[1]:.1f},{torque[2]:.1f}] | "
                    f"Force:[{force_limited[0]:.3f},{force_limited[1]:.3f},{force_limited[2]:.3f}]N | "
                    f"Mag:{force_mag:.3f}N"
                )
                
                force_vector_msg = String()
                force_vector_msg.data = force_info
                self.force_vector_pub.publish(force_vector_msg)
                self.get_logger().info(force_info)
            else:
                # No collision - zero force
                force_limited = np.zeros(3)
                force_mag = 0.0
            
            # Create Wrench message
            wrench_msg = Wrench()
            
            # Force feedback (only non-zero during collision)
            wrench_msg.force.x = float(force_limited[0])
            wrench_msg.force.y = float(force_limited[1])
            wrench_msg.force.z = float(force_limited[2])
            
            # Position always zero (using torque field as per your requirement)
            wrench_msg.torque.x = 0.0
            wrench_msg.torque.y = 0.0
            wrench_msg.torque.z = 0.0
            
            self.haptic_pub.publish(wrench_msg)
            
            # Publish debug information
            debug_msg = Float64MultiArray()
            debug_msg.data = [
                velocity[0], velocity[1], velocity[2],           # 0-2: velocities
                effort[0], effort[1], effort[2],                 # 3-5: efforts
                torque[0], torque[1], torque[2],                 # 6-8: computed torques
                force_limited[0], force_limited[1], force_limited[2],  # 9-11: final force
                float(force_mag),                                # 12: force magnitude
                float(collision_detected),                       # 13: collision flag
                float(self.was_moving[0]), float(self.was_moving[1]), float(self.was_moving[2])  # 14-16: moving flags
            ]
            self.debug_pub.publish(debug_msg)
            
            # Store for next iteration
            self.prev_velocity = velocity.copy()
            self.prev_effort = effort.copy()
            self.prev_time = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    mapper = ForceFeedbackMapper()
    
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

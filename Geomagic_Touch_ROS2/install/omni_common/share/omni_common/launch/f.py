#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from omni_msgs.msg import OmniFeedback, OmniState
from std_msgs.msg import Float64MultiArray, String
from collections import deque

class ForceFeedbackMapper(Node):
    def __init__(self):
        super().__init__('force_feedback_mapper')
        
        # Joint indices
        self.joint_indices = {
            'waist': 0,
            'shoulder': 1,
            'elbow': 2
        }
        
        # RX200 DH parameters
        self.L1 = 0.05  
        self.L2 = 0.20  
        self.L3 = 0.20  
        
        # Force limit
        self.max_force = 2.0
        
        # PER-JOINT thresholds (shoulder and elbow need higher thresholds)
        # These account for gravity and movement dynamics
        self.absolute_effort_thresholds = {
            0: 250.0,  # waist - lower threshold
            1: 150.0,  # shoulder - higher (fights gravity)
            2: 300.0   # elbow - highest (long moment arm)
        }
        
        self.excess_effort_thresholds = {
            0: 150.0,  # waist
            1: 100.0,  # shoulder - needs more excess
            2: 300.0   # elbow - needs most excess
        }
        
        # Velocity gate - only detect contact when moving slowly or stopped
        self.max_contact_velocity = 0.3  # rad/s - faster = ignore (free motion)
        
        # Moving average for effort baseline
        self.effort_history = [deque(maxlen=30) for _ in range(3)]  # Longer history
        self.effort_baseline = np.zeros(3)
        self.effort_max_seen = np.zeros(3)  # Track max during calibration
        
        # State
        self.prev_effort = np.zeros(3)
        self.prev_velocity = np.zeros(3)
        self.current_positions = np.zeros(3)
        self.current_force = np.zeros(3)
        self.contact_active = np.array([False, False, False])
        
        # Contact hysteresis - once detected, sustain with lower threshold
        self.contact_sustain_threshold_ratio = 0.7  # 70% of detection threshold
        
        # Force scaling - CONSERVATIVE
        self.effort_to_force_scale = 0.02
        
        # Warmup
        self.sample_count = 0
        self.warmup_samples = 80  # Longer to see full range of motion
        
        # Publishers and Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/rx200/joint_states', self.joint_state_callback, 10)
        
        self.feedback_pub = self.create_publisher(
            OmniFeedback, '/phantom/force_feedback', 10)
        
        self.position_pub = self.create_publisher(
            Vector3, '/phantom/position_feedback', 10)
        
        self.debug_pub = self.create_publisher(
            Float64MultiArray, '/force_feedback/debug', 10)
        
        self.force_vector_pub = self.create_publisher(
            String, '/force_feedback/force_vector', 10)
        
        self.joint_monitor_pub = self.create_publisher(
            String, '/force_feedback/joint_monitor', 10)
        
        self.monitor_timer = self.create_timer(0.1, self.monitor_callback)
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("🔧 Force Feedback Mapper - FIXED FORCE GENERATION")
        self.get_logger().info("=" * 80)
        self.get_logger().info("📊 Learning baseline effort during normal movement...")
        self.get_logger().info("=" * 80)
    
    def monitor_callback(self):
        """Continuous monitoring"""
        if self.sample_count < self.warmup_samples:
            progress = (self.sample_count / self.warmup_samples) * 100
            self.get_logger().info(f"🔄 Calibrating... {progress:.0f}%")
            return
            
        if hasattr(self, 'prev_effort'):
            contact_str = "".join(["🔴" if c else "⚪" for c in self.contact_active])
            force_mag = np.linalg.norm(self.current_force)
            
            # Show baseline vs current
            monitor_msg = String()
            monitor_msg.data = (
                f"{contact_str} | "
                f"F:[{self.current_force[0]:6.3f},{self.current_force[1]:6.3f},{self.current_force[2]:6.3f}]N "
                f"M:{force_mag:.3f} | "
                f"Eff:[{self.prev_effort[0]:6.1f},{self.prev_effort[1]:6.1f},{self.prev_effort[2]:6.1f}] | "
                f"Base:[{self.effort_baseline[0]:6.1f},{self.effort_baseline[1]:6.1f},{self.effort_baseline[2]:6.1f}]"
            )
            self.joint_monitor_pub.publish(monitor_msg)
    
    def compute_jacobian(self, q):
        """Compute Jacobian"""
        q1, q2, q3 = q[0], q[1], q[2]
        
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)
        
        J = np.array([
            [-(self.L2*s2 + self.L3*s23)*s1, (self.L2*c2 + self.L3*c23)*c1, self.L3*c23*c1],
            [(self.L2*s2 + self.L3*s23)*c1, (self.L2*c2 + self.L3*c23)*s1, self.L3*c23*s1],
            [0, -(self.L2*s2 + self.L3*s23), -self.L3*s23]
        ])
        
        return J
    
    def extract_joint_data(self, msg):
        """Extract data"""
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
    
    def detect_contact_and_compute_torque(self, velocity, effort):
        """
        IMPROVED APPROACH with per-joint thresholds:
        1. Different thresholds for waist/shoulder/elbow
        2. Velocity gating - only detect when moving slowly
        3. Hysteresis - easier to sustain than to trigger
        """
        resistance_torque = np.zeros(3)
        contact_detected = False
        contact_joints = []
        joint_names = ['waist', 'shoulder', 'elbow']
        
        for i in range(3):
            current_effort_abs = abs(effort[i])
            baseline_effort_abs = abs(self.effort_baseline[i])
            current_velocity_abs = abs(velocity[i])
            
            # Calculate excess over baseline
            effort_excess = current_effort_abs - baseline_effort_abs
            
            # Get per-joint thresholds
            abs_threshold = self.absolute_effort_thresholds[i]
            excess_threshold = self.excess_effort_thresholds[i]
            
            # Velocity gate - don't detect during fast movement
            # Fast movement naturally has high efforts
            is_moving_slowly = current_velocity_abs < self.max_contact_velocity
            
            # If already in contact, use lower sustain threshold (hysteresis)
            if self.contact_active[i]:
                # Sustain with 70% of original thresholds
                abs_threshold *= self.contact_sustain_threshold_ratio
                excess_threshold *= self.contact_sustain_threshold_ratio
            
            # Contact criteria
            is_high_absolute = current_effort_abs > abs_threshold
            is_high_excess = effort_excess > excess_threshold
            
            # NEW CONTACT detection
            if not self.contact_active[i]:
                # Need ALL conditions for new contact
                if is_high_absolute and is_high_excess and is_moving_slowly:
                    self.contact_active[i] = True
                    contact_detected = True
                    contact_joints.append(joint_names[i])
                    resistance_torque[i] = effort_excess
                    
                    self.get_logger().warn(
                        f"💥 CONTACT: {joint_names[i]} | "
                        f"Eff:{current_effort_abs:.1f} (>{abs_threshold:.1f}) | "
                        f"Excess:{effort_excess:.1f} (>{excess_threshold:.1f}) | "
                        f"Vel:{current_velocity_abs:.3f}"
                    )
            
            # SUSTAIN existing contact (easier threshold)
            else:
                if is_high_absolute and is_high_excess:
                    contact_detected = True
                    contact_joints.append(f"{joint_names[i]}_sustain")
                    resistance_torque[i] = effort_excess
                else:
                    # Contact ended
                    self.get_logger().info(
                        f"✅ RELEASE: {joint_names[i]} | "
                        f"Eff:{current_effort_abs:.1f} | Excess:{effort_excess:.1f}"
                    )
                    self.contact_active[i] = False
                    resistance_torque[i] = 0.0
        
        if contact_detected:
            self.get_logger().info(
                f"🔴 ACTIVE: {contact_joints} | "
                f"Torque:[{resistance_torque[0]:.1f},{resistance_torque[1]:.1f},"
                f"{resistance_torque[2]:.1f}]"
            )
        
        return resistance_torque, contact_detected
    
    def torque_to_force(self, torque, positions):
        """
        FIXED: Map resistance torque to Cartesian force
        F = J^T * tau
        """
        J = self.compute_jacobian(positions)
        
        # Scale torque to force - use resistance directly
        scaled_torque = torque * self.effort_to_force_scale
        
        # Transform to Cartesian space
        force = np.dot(J.T, scaled_torque)
        
        self.get_logger().debug(
            f"Torque:[{scaled_torque[0]:.3f},{scaled_torque[1]:.3f},"
            f"{scaled_torque[2]:.3f}] → "
            f"Force:[{force[0]:.3f},{force[1]:.3f},{force[2]:.3f}]"
        )
        
        return force, J
    
    def limit_force(self, force):
        """Limit force magnitude"""
        magnitude = np.linalg.norm(force)
        
        if magnitude < 0.001:
            return np.zeros(3), 0.0, False
        
        if magnitude > self.max_force:
            force = force * (self.max_force / magnitude)
            limited = True
        else:
            limited = False
        
        return force, magnitude, limited
    
    def joint_state_callback(self, msg):
        """Main callback"""
        try:
            velocity, effort, positions = self.extract_joint_data(msg)
            self.current_positions = positions
            
            self.sample_count += 1
            
            # WARMUP: Learn baseline effort during normal movement
            if self.sample_count <= self.warmup_samples:
                for i in range(3):
                    # Store effort values during normal operation
                    eff_abs = abs(effort[i])
                    self.effort_history[i].append(eff_abs)
                    # Track maximum seen
                    self.effort_max_seen[i] = max(self.effort_max_seen[i], eff_abs)
                
                self.prev_velocity = velocity.copy()
                self.prev_effort = effort.copy()
                
                # At end of warmup, calculate baseline
                if self.sample_count == self.warmup_samples:
                    for i in range(3):
                        if len(self.effort_history[i]) > 0:
                            # Use 80th percentile as baseline (higher for dynamic joints)
                            efforts = sorted(list(self.effort_history[i]))
                            idx = int(len(efforts) * 0.80)
                            self.effort_baseline[i] = efforts[idx]
                    
                    joint_names = ['waist', 'shoulder', 'elbow']
                    self.get_logger().info("=" * 80)
                    self.get_logger().info("✅ Baseline learned!")
                    for i in range(3):
                        self.get_logger().info(
                            f"📊 {joint_names[i]}: Baseline={self.effort_baseline[i]:.1f} Nm, "
                            f"Max={self.effort_max_seen[i]:.1f} Nm | "
                            f"Thresholds: Abs>{self.absolute_effort_thresholds[i]:.1f}, "
                            f"Excess>{self.excess_effort_thresholds[i]:.1f}"
                        )
                    self.get_logger().info("🎮 HAPTIC FEEDBACK ACTIVE")
                    self.get_logger().info("💡 Touch something SLOWLY to feel feedback")
                    self.get_logger().info("⚠️  Fast movements ignored (velocity < 0.3 rad/s)")
                    self.get_logger().info("=" * 80)
                
                return
            
            feedback = OmniFeedback()
            
            # DETECT contact and compute resistance torque
            resistance_torque, contact_detected = self.detect_contact_and_compute_torque(
                velocity, effort
            )
            
            # GENERATE FORCE only when contact detected
            if contact_detected and np.any(resistance_torque > 0):
                # Map resistance torque to Cartesian force
                force, jacobian = self.torque_to_force(resistance_torque, positions)
                
                # Limit force
                force_limited, force_mag, was_limited = self.limit_force(force)
                
                self.current_force = force_limited
                
                self.get_logger().info(
                    f"⚡ HAPTIC ON | "
                    f"Force:[{force_limited[0]:.3f},{force_limited[1]:.3f},"
                    f"{force_limited[2]:.3f}]N | Mag:{force_mag:.3f}N"
                )
            else:
                # NO CONTACT = ZERO FORCE (this is the key fix)
                self.current_force = np.zeros(3)
            
            # Publish to Geomagic
            feedback.force.x = float(self.current_force[0])
            feedback.force.y = float(self.current_force[1])
            feedback.force.z = float(self.current_force[2])
            feedback.position.x = 0.0
            feedback.position.y = 0.0
            feedback.position.z = 0.0
            self.feedback_pub.publish(feedback)
            
            # Debug
            debug_msg = Float64MultiArray()
            debug_msg.data = [
                velocity[0], velocity[1], velocity[2],
                effort[0], effort[1], effort[2],
                resistance_torque[0], resistance_torque[1], resistance_torque[2],
                self.current_force[0], self.current_force[1], self.current_force[2],
                float(np.linalg.norm(self.current_force)),
                float(contact_detected),
                float(self.contact_active[0]), 
                float(self.contact_active[1]), 
                float(self.contact_active[2])
            ]
            self.debug_pub.publish(debug_msg)
            
            self.prev_velocity = velocity.copy()
            self.prev_effort = effort.copy()
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
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

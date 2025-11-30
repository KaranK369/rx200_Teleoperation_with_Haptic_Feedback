#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from omni_msgs.msg import OmniFeedback
from std_msgs.msg import String
from collections import deque

class ForceFeedbackMapper(Node):
    def __init__(self):
        super().__init__('force_feedback_mapper')
        
        # ====================================================================
        # TUNING PARAMETERS - Adjust these to fine-tune force feedback
        # ====================================================================
        
        # CALIBRATION SETTINGS
        self.calibration_samples = 150  # Number of samples (15 seconds at 10Hz)
        self.calibration_percentile_min = 5   # Use 5th percentile as min (ignore outliers)
        self.calibration_percentile_max = 95  # Use 95th percentile as max (ignore outliers)
        
        # CONTACT DETECTION SENSITIVITY (Per Joint)
        # Lower = more sensitive (detects lighter touches)
        # Higher = less sensitive (only heavy contacts)
        self.contact_thresholds = {
            'waist': {
                'margin': 80.0,      # Nm outside range to trigger contact
                'sustain_margin': 50.0,  # Nm to sustain contact (lower = easier to maintain)
            },
            'shoulder': {
                'margin': 100.0,     # Shoulder carries more weight, less sensitive
                'sustain_margin': 60.0,
            },
            'elbow': {
                'margin': 120.0,     # Elbow has most leverage, least sensitive
                'sustain_margin': 70.0,
            }
        }
        
        # FORCE GENERATION
        self.effort_to_force_scale = 0.04  # Scaling factor: effort → force
        self.max_force = 2.0                 # Maximum force output (N)
        self.min_force_threshold = 0.01     # Minimum force to send (filter noise)
        
        # SMOOTHING (reduce jitter)
        self.effort_smooth_window = 5       # Samples to average
        self.force_smooth_window = 3        # Force output smoothing
        
        # VELOCITY GATING (optional - set to None to disable)
        self.max_contact_velocity = None    # rad/s - ignore fast movements (None = disabled)
        
        # HYSTERESIS (prevents flickering)
        self.contact_min_duration = 3       # Frames - contact must persist this long
        self.release_delay = 2              # Frames - delay before releasing contact
        
        # ====================================================================
        # END OF TUNING PARAMETERS
        # ====================================================================
        
        # Joint indices
        self.joint_indices = {'waist': 0, 'shoulder': 1, 'elbow': 2}
        
        # RX200 DH parameters
        self.L1 = 0.05  
        self.L2 = 0.20  
        self.L3 = 0.20  
        
        # Calibration data structures
        self.effort_history = [deque(maxlen=self.calibration_samples) for _ in range(3)]
        self.effort_min = np.zeros(3)
        self.effort_max = np.zeros(3)
        self.effort_mean = np.zeros(3)
        self.effort_std = np.zeros(3)
        
        # State tracking
        self.prev_effort = np.zeros(3)
        self.prev_velocity = np.zeros(3)
        self.current_positions = np.zeros(3)
        self.current_force = np.zeros(3)
        
        # Contact state with counters
        self.contact_active = np.array([False, False, False])
        self.contact_counter = np.zeros(3, dtype=int)  # Frames in contact
        self.release_counter = np.zeros(3, dtype=int)  # Frames since contact ended
        
        # Smoothing buffers
        self.effort_smooth = [deque(maxlen=self.effort_smooth_window) for _ in range(3)]
        self.force_history = deque(maxlen=self.force_smooth_window)
        
        # Calibration
        self.sample_count = 0
        self.calibrated = False
        
        # Publishers and Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/rx200/joint_states_with_effort', self.joint_state_callback, 10)
        
        self.feedback_pub = self.create_publisher(
            OmniFeedback, '/phantom/force_feedback', 10)
        
        self.position_pub = self.create_publisher(
            Vector3, '/phantom/position_feedback', 10)
        
        self.joint_monitor_pub = self.create_publisher(
            String, '/force_feedback/joint_monitor', 10)
        
        self.monitor_timer = self.create_timer(0.1, self.monitor_callback)
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("🔧 ADVANCED FORCE FEEDBACK - Calibration-Based Detection")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"📊 Calibration: {self.calibration_samples} samples "
                             f"({self.calibration_samples/10:.1f} seconds)")
        self.get_logger().info("⚠️  IMPORTANT: Move arm through FULL range during calibration!")
        self.get_logger().info("=" * 80)
        self.get_logger().info("🎛️  TUNING PARAMETERS:")
        self.get_logger().info(f"   Waist margin: {self.contact_thresholds['waist']['margin']:.1f} Nm")
        self.get_logger().info(f"   Shoulder margin: {self.contact_thresholds['shoulder']['margin']:.1f} Nm")
        self.get_logger().info(f"   Elbow margin: {self.contact_thresholds['elbow']['margin']:.1f} Nm")
        self.get_logger().info(f"   Force scale: {self.effort_to_force_scale:.4f}")
        self.get_logger().info(f"   Max force: {self.max_force:.1f} N")
        self.get_logger().info("=" * 80)
        self.get_logger().info("🔄 Starting calibration...")
    
    def monitor_callback(self):
        """Continuous monitoring with detailed status"""
        if not self.calibrated:
            progress = (self.sample_count / self.calibration_samples) * 100
            self.get_logger().info(
                f"📊 Calibrating... {progress:.0f}% "
                f"({self.sample_count}/{self.calibration_samples})"
            )
            return
            
        if hasattr(self, 'prev_effort'):
            contact_str = "".join(["🔴" if c else "⚪" for c in self.contact_active])
            force_mag = np.linalg.norm(self.current_force)
            
            # Show how far outside range for each joint
            deviations = []
            for i in range(3):
                if self.prev_effort[i] < self.effort_min[i]:
                    dev = self.effort_min[i] - self.prev_effort[i]
                    deviations.append(f"↓{dev:.0f}")
                elif self.prev_effort[i] > self.effort_max[i]:
                    dev = self.prev_effort[i] - self.effort_max[i]
                    deviations.append(f"↑{dev:.0f}")
                else:
                    deviations.append("✓")
            
            monitor_msg = String()
            monitor_msg.data = (
                f"{contact_str} | "
                f"F:[{self.current_force[0]:6.3f},{self.current_force[1]:6.3f},"
                f"{self.current_force[2]:6.3f}]N M:{force_mag:.3f} | "
                f"Eff:[{self.prev_effort[0]:7.1f},{self.prev_effort[1]:7.1f},"
                f"{self.prev_effort[2]:7.1f}] | "
                f"Dev:[{deviations[0]},{deviations[1]},{deviations[2]}]"
            )
            self.joint_monitor_pub.publish(monitor_msg)
    
    def compute_jacobian(self, q):
        """Compute analytical Jacobian"""
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
        """Extract joint data from message"""
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
    
    def detect_contact_calibrated(self, velocity, effort):
        """
        ACCURATE CALIBRATION-BASED CONTACT DETECTION
        
        Uses learned effort ranges with per-joint thresholds.
        Includes hysteresis to prevent flickering.
        """
        torque = np.zeros(3)
        contact_detected = False
        contact_joints = []
        joint_names = ['waist', 'shoulder', 'elbow']
        
        for i in range(3):
            # Smooth effort
            self.effort_smooth[i].append(effort[i])
            if len(self.effort_smooth[i]) >= 3:
                smoothed_effort = np.mean(list(self.effort_smooth[i]))
            else:
                smoothed_effort = effort[i]
            
            # Get thresholds for this joint
            joint_name = joint_names[i]
            margin = self.contact_thresholds[joint_name]['margin']
            sustain_margin = self.contact_thresholds[joint_name]['sustain_margin']
            
            # Use sustain margin if already in contact (hysteresis)
            active_margin = sustain_margin if self.contact_active[i] else margin
            
            # Calculate deviations from normal range
            below_range = self.effort_min[i] - smoothed_effort  # Positive if below
            above_range = smoothed_effort - self.effort_max[i]  # Positive if above
            
            # Optional velocity gating
            if self.max_contact_velocity is not None:
                velocity_ok = abs(velocity[i]) < self.max_contact_velocity
            else:
                velocity_ok = True
            
            # Check if outside range
            is_outside = (below_range > active_margin) or (above_range > active_margin)
            
            if is_outside and velocity_ok:
                # Contact detected
                self.contact_counter[i] += 1
                self.release_counter[i] = 0
                
                # Only activate after minimum duration
                if self.contact_counter[i] >= self.contact_min_duration:
                    if not self.contact_active[i]:
                        # New contact
                        direction = "BELOW" if below_range > active_margin else "ABOVE"
                        deviation = max(below_range, above_range)
                        
                        self.get_logger().warn(
                            f"💥 CONTACT ({direction}): {joint_name} | "
                            f"Effort:{smoothed_effort:.1f} Nm | "
                            f"Range:[{self.effort_min[i]:.1f}, {self.effort_max[i]:.1f}] | "
                            f"Deviation:{deviation:.1f} Nm (threshold:{margin:.1f})"
                        )
                    
                    self.contact_active[i] = True
                    contact_detected = True
                    contact_joints.append(joint_name)
                    
                    # Use deviation magnitude as torque
                    torque[i] = max(below_range, above_range, 0)
            
            else:
                # Not outside range
                self.contact_counter[i] = 0
                
                if self.contact_active[i]:
                    # Increment release counter
                    self.release_counter[i] += 1
                    
                    # Only release after delay (hysteresis)
                    if self.release_counter[i] >= self.release_delay:
                        self.get_logger().info(
                            f"✅ RELEASE: {joint_name} | "
                            f"Effort:{smoothed_effort:.1f} back in range"
                        )
                        self.contact_active[i] = False
                        self.release_counter[i] = 0
                    else:
                        # Still sustaining during release delay
                        contact_detected = True
                        contact_joints.append(f"{joint_name}_release")
                        torque[i] = max(below_range, above_range, 0)
        
        if contact_detected:
            self.get_logger().info(
                f"🔴 ACTIVE: {contact_joints} | "
                f"Torques:[{torque[0]:.1f},{torque[1]:.1f},{torque[2]:.1f}] Nm"
            )
        
        return torque, contact_detected
    
    def torque_to_force(self, torque, positions):
        """Map joint torques to Cartesian force"""
        J = self.compute_jacobian(positions)
        scaled_torque = torque * self.effort_to_force_scale
        force = np.dot(J.T, scaled_torque)
        return force
    
    def smooth_force(self, force):
        """Apply temporal smoothing to force"""
        self.force_history.append(force.copy())
        if len(self.force_history) > 1:
            smoothed = np.mean(list(self.force_history), axis=0)
            return smoothed
        return force
    
    def limit_force(self, force):
        """Limit and filter force"""
        magnitude = np.linalg.norm(force)
        
        # Filter very small forces (noise)
        if magnitude < self.min_force_threshold:
            return np.zeros(3), 0.0, False
        
        # Limit to maximum
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
            
            # CALIBRATION PHASE
            if not self.calibrated:
                for i in range(3):
                    self.effort_history[i].append(effort[i])
                    self.effort_smooth[i].append(effort[i])
                
                self.prev_velocity = velocity.copy()
                self.prev_effort = effort.copy()
                
                # Complete calibration
                if self.sample_count >= self.calibration_samples:
                    joint_names = ['waist', 'shoulder', 'elbow']
                    
                    self.get_logger().info("=" * 80)
                    self.get_logger().info("✅ CALIBRATION COMPLETE!")
                    self.get_logger().info("=" * 80)
                    
                    for i in range(3):
                        efforts = sorted(list(self.effort_history[i]))
                        
                        # Use percentiles to ignore outliers
                        idx_min = int(len(efforts) * self.calibration_percentile_min / 100)
                        idx_max = int(len(efforts) * self.calibration_percentile_max / 100)
                        
                        self.effort_min[i] = efforts[idx_min]
                        self.effort_max[i] = efforts[idx_max]
                        self.effort_mean[i] = np.mean(efforts)
                        self.effort_std[i] = np.std(efforts)
                        
                        range_size = self.effort_max[i] - self.effort_min[i]
                        margin = self.contact_thresholds[joint_names[i]]['margin']
                        
                        self.get_logger().info(
                            f"📊 {joint_names[i].upper()}: "
                            f"Range=[{self.effort_min[i]:7.1f}, {self.effort_max[i]:7.1f}] Nm "
                            f"(span:{range_size:6.1f}) | "
                            f"Mean:{self.effort_mean[i]:7.1f} ±{self.effort_std[i]:5.1f} | "
                            f"Margin:{margin:.1f} Nm"
                        )
                    
                    self.calibrated = True
                    self.get_logger().info("=" * 80)
                    self.get_logger().info("🎮 HAPTIC FEEDBACK ACTIVE")
                    self.get_logger().info("💡 Touch arm to test force feedback")
                    self.get_logger().info("=" * 80)
                
                return
            
            # OPERATION PHASE
            feedback = OmniFeedback()
            
            # Detect contact
            torque, contact_detected = self.detect_contact_calibrated(velocity, effort)
            
            # Generate force
            if contact_detected:
                force = self.torque_to_force(torque, positions)
                force_smoothed = self.smooth_force(force)
                force_limited, force_mag, was_limited = self.limit_force(force_smoothed)
                
                self.current_force = force_limited
                
                if force_mag > self.min_force_threshold:
                    self.get_logger().info(
                        f"⚡ HAPTIC: [{force_limited[0]:.3f},{force_limited[1]:.3f},"
                        f"{force_limited[2]:.3f}]N | Mag:{force_mag:.3f}N"
                        f"{' (LIMITED)' if was_limited else ''}"
                    )
            else:
                self.current_force = np.zeros(3)
                self.force_history.clear()
            
            # Publish to Geomagic
            feedback.force.x = float(self.current_force[0])
            feedback.force.y = float(self.current_force[1])
            feedback.force.z = float(self.current_force[2])
            feedback.position.x = 0.0
            feedback.position.y = 0.0
            feedback.position.z = 0.0
            self.feedback_pub.publish(feedback)
            
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

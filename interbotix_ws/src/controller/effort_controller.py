#!/usr/bin/env python3
"""
Effort Calculator for Gazebo Simulation
Computes joint effort from damping and velocity
Detects lateral collisions and increases waist effort
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class EffortCalculator(Node):
    def __init__(self):
        super().__init__('effort_controller')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'rx200')
        
        # Damping coefficients for each joint
        self.damping_coefficients = {
            'waist': 0.3,
            'shoulder': 0.3,
            'elbow': 0.3,
            'forearm_roll': 0.3,
            'wrist_angle': 0.3,
            'wrist_rotate': 0.3,
            'gripper': 0.2,
            'left_finger': 0.2,
            'right_finger': 0.2,
        }
        
        # Friction coefficients for each joint
        self.friction_coefficients = {
            'waist': 1.0,
            'shoulder': 1.0,
            'elbow': 1.0,
            'forearm_roll': 1.0,
            'wrist_angle': 1.0,
            'wrist_rotate': 1.0,
            'gripper': 0.8,
            'left_finger': 0.8,
            'right_finger': 0.8,
        }
        
        # Collision detection parameters
        self.collision_effort_multiplier = 5.0  # Multiplier when collision detected
        self.velocity_change_threshold = 0.5  # rad/s - sudden velocity change indicates collision
        
        # Store previous velocities for collision detection
        self.previous_velocities = {}
        self.previous_timestamp = None
        
        self.robot_name = self.get_parameter('robot_name').value
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            f'/{self.robot_name}/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish modified joint states with computed effort
        self.publisher = self.create_publisher(
            JointState,
            f'/{self.robot_name}/joint_states_with_effort',
            10
        )
        
        self.get_logger().info(f'Effort Calculator initialized for {self.robot_name}')
        self.get_logger().info('Waist collision detection enabled')
    
    def detect_lateral_collision(self, joint_name, current_velocity, position):
        """
        Detect if there's a lateral collision affecting the waist
        Returns True if collision detected on arm joints (not waist itself)
        """
        if joint_name not in self.previous_velocities:
            return False
        
        # Check for sudden velocity changes in arm joints (not waist)
        # When arm hits something laterally, arm joints show velocity changes
        if joint_name in ['shoulder', 'elbow', 'forearm_roll', 'wrist_angle']:
            prev_vel = self.previous_velocities[joint_name]
            velocity_change = abs(current_velocity - prev_vel)
            
            # If there's a sudden deceleration/acceleration, it indicates collision
            if velocity_change > self.velocity_change_threshold:
                self.get_logger().info(
                    f'Lateral collision detected on {joint_name}: '
                    f'velocity change = {velocity_change:.3f} rad/s'
                )
                return True
        
        return False
    
    def joint_state_callback(self, msg: JointState):
        """Compute effort from velocity and publish augmented joint state"""
        computed_efforts = []
        
        # Detect if any arm joint experienced collision
        collision_detected = False
        for i, (joint_name, velocity, position) in enumerate(zip(msg.name, msg.velocity, msg.position)):
            if self.detect_lateral_collision(joint_name, velocity, position):
                collision_detected = True
                break
        
        # Compute effort for each joint
        for i, (joint_name, velocity) in enumerate(zip(msg.name, msg.velocity)):
            # Damping torque: proportional to velocity
            damping_effort = self.damping_coefficients.get(joint_name, 0.1) * abs(velocity) * 100
            
            # Friction torque: constant opposing motion
            friction_effort = self.friction_coefficients.get(joint_name, 0.5) * (1.0 if velocity != 0 else 0)
            
            # Total effort (Nm)
            total_effort = damping_effort + friction_effort
            
            # Add gravity effect for arm joints
            if joint_name in ['shoulder', 'elbow', 'wrist_angle']:
                gravity_effort = 2.0
                total_effort += gravity_effort
            
            # CRITICAL: If collision detected, significantly increase WAIST effort only
            if joint_name == 'waist' and collision_detected:
                collision_resistance = self.collision_effort_multiplier * 10.0
                total_effort += collision_resistance
                self.get_logger().info(
                    f'Waist effort increased due to lateral collision: {total_effort:.2f} Nm'
                )
            
            computed_efforts.append(total_effort)
        
        # Store current velocities for next iteration
        for joint_name, velocity in zip(msg.name, msg.velocity):
            self.previous_velocities[joint_name] = velocity
        
        # Create augmented message with computed efforts
        augmented_msg = JointState()
        augmented_msg.header = msg.header
        augmented_msg.name = msg.name
        augmented_msg.position = msg.position
        augmented_msg.velocity = msg.velocity
        augmented_msg.effort = computed_efforts
        
        self.publisher.publish(augmented_msg)

def main(args=None):
    rclpy.init(args=args)
    calculator = EffortCalculator()
    rclpy.spin(calculator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Effort Calculator for Gazebo Simulation
Computes joint effort from damping and velocity
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class EffortCalculator(Node):
    def __init__(self):
        super().__init__('effort_calculator')
        
        # Declare parameters
        self.declare_parameter('robot_name', 'rx200')
        
        # Damping coefficients for each joint (from control.urdf.xacro)
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
    
    def joint_state_callback(self, msg: JointState):
        """Compute effort from velocity and publish augmented joint state"""
        # Compute effort for each joint: effort = damping * velocity + friction * sign(velocity)
        computed_efforts = []
        
        for i, (joint_name, velocity) in enumerate(zip(msg.name, msg.velocity)):
            # Damping torque: proportional to velocity
            damping_effort = self.damping_coefficients.get(joint_name, 0.1) * abs(velocity) * 100
            
            # Friction torque: constant opposing motion
            friction_effort = self.friction_coefficients.get(joint_name, 0.5) * (1.0 if velocity != 0 else 0)
            
            # Total effort (Nm)
            total_effort = damping_effort + friction_effort
            
            # Add gravity effect (rough approximation for arm joints)
            # This would need to be calculated properly based on link masses and COM
            if joint_name in ['shoulder', 'elbow', 'wrist_angle']:
                gravity_effort = 2.0  # Rough estimate in Nm
                total_effort += gravity_effort
            
            computed_efforts.append(total_effort)
        
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

#!/usr/bin/env python3

"""
Simple action client for controlling rx200 arm
Uses ROS 2 actions to send trajectory goals to the arm
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from math import pi
import time


class SimpleArmController(Node):
    def __init__(self):
        super().__init__('simple_arm_controller')
        
        # Create action client for arm control
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/rx200/arm_controller/follow_joint_trajectory'
        )
        
        # Joint names for rx200
        self.joint_names = [
            'waist',
            'shoulder', 
            'elbow',
            'wrist_angle',
            'wrist_rotate'
        ]
        
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server connected!")
    
    def move_to_positions(self, positions, duration_sec=3.0):
        """
        Move arm to specified joint positions
        
        Args:
            positions: List of 5 joint angles in radians [waist, shoulder, elbow, wrist_angle, wrist_rotate]
            duration_sec: Time to complete the motion
        """
        if len(positions) != 5:
            self.get_logger().error("Need exactly 5 joint positions!")
            return False
        
        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), 
                                        nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"Sending goal: {positions}")
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Motion completed with error code: {result.error_code}')
    
    # Predefined poses
    def home_position(self):
        """Move to home position"""
        self.get_logger().info("Moving to HOME position")
        return self.move_to_positions([0.0, 0.0, 0.0, 0.0, 0.0])
    
    def sleep_position(self):
        """Move to sleep position"""
        self.get_logger().info("Moving to SLEEP position")
        return self.move_to_positions([0.0, -1.88, 1.5, 0.8, 0.0])
    
    def upright_position(self):
        """Move to upright position"""
        self.get_logger().info("Moving to UPRIGHT position")
        return self.move_to_positions([0.0, -0.97, 1.0, 0.0, 0.0])
    
    def custom_pose_1(self):
        """Example custom pose - reaching forward"""
        self.get_logger().info("Moving to CUSTOM pose 1")
        return self.move_to_positions([0.0, -0.5, 0.5, -0.3, 0.0])
    
    def custom_pose_2(self):
        """Example custom pose - reaching to the side"""
        self.get_logger().info("Moving to CUSTOM pose 2")
        return self.move_to_positions([pi/4, -0.8, 0.8, -0.2, 0.0])
    
    def scan_motion(self):
        """Perform a scanning motion"""
        self.get_logger().info("Performing SCAN motion")
        
        # Move through several positions
        positions_sequence = [
            [0.0, -0.5, 0.5, 0.0, 0.0],
            [pi/6, -0.5, 0.5, 0.0, 0.0],
            [pi/3, -0.5, 0.5, 0.0, 0.0],
            [pi/6, -0.5, 0.5, 0.0, 0.0],
            [0.0, -0.5, 0.5, 0.0, 0.0],
        ]
        
        for pos in positions_sequence:
            self.move_to_positions(pos, duration_sec=2.0)
            time.sleep(2.5)  # Wait for motion to complete


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Create action client for gripper
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/rx200/gripper_controller/follow_joint_trajectory'
        )
        
        self.joint_names = ['left_finger', 'right_finger']
        
        self.get_logger().info("Waiting for gripper action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Gripper action server connected!")
    
    def set_gripper(self, position, duration_sec=1.0):
        """
        Set gripper position
        
        Args:
            position: 0.0 (closed) to 0.037 (open)
            duration_sec: Time to complete motion
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [position, -position]  # Left opens positive, right opens negative
        point.time_from_start = Duration(sec=int(duration_sec),
                                        nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"Setting gripper to: {position}")
        self._action_client.send_goal_async(goal_msg)
    
    def open_gripper(self):
        """Open gripper fully"""
        self.get_logger().info("Opening gripper")
        self.set_gripper(0.037)
    
    def close_gripper(self):
        """Close gripper fully"""
        self.get_logger().info("Closing gripper")
        self.set_gripper(0.0)
    
    def partial_close(self, percentage):
        """
        Partially close gripper
        
        Args:
            percentage: 0-100, where 0 is closed and 100 is fully open
        """
        position = 0.037 * (percentage / 100.0)
        self.get_logger().info(f"Setting gripper to {percentage}%")
        self.set_gripper(position)


def main():
    rclpy.init()
    
    arm = SimpleArmController()
    gripper = GripperController()
    
    try:
        print("\n" + "="*50)
        print("rx200 Arm Control Demo")
        print("="*50)
        
        # Demo sequence
        print("\n1. Moving to HOME position...")
        arm.home_position()
        time.sleep(4)
        
        print("\n2. Opening gripper...")
        gripper.open_gripper()
        time.sleep(2)
        
        print("\n3. Moving to CUSTOM pose 1...")
        arm.custom_pose_1()
        time.sleep(4)
        
        print("\n4. Closing gripper...")
        gripper.close_gripper()
        time.sleep(2)
        
        print("\n5. Moving to UPRIGHT position...")
        arm.upright_position()
        time.sleep(4)
        
        print("\n6. Opening gripper...")
        gripper.open_gripper()
        time.sleep(2)
        
        print("\n7. Performing scan motion...")
        arm.scan_motion()
        
        print("\n8. Returning to HOME...")
        arm.home_position()
        time.sleep(4)
        
        print("\n" + "="*50)
        print("Demo completed successfully!")
        print("="*50 + "\n")
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        arm.destroy_node()
        gripper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

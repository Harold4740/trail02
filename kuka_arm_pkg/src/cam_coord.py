#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.optimize import minimize

class CoordinateSubscriber(Node):

    def __init__(self):
        super().__init__('coordinate_subscriber')

        # Subscriber to receive coordinates
        self.subscription = self.create_subscription(
            Point,
            'camera/coordinates',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for sending joint trajectory commands
        self.publisher_ = self.create_publisher(JointTrajectory, '/kuka_arm_controller/joint_trajectory', 10)

        # Initialize joint names (Replace with your actual joint names)
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ]

        # Define DH parameters for KUKA KR 120 (values are placeholders and should be replaced with actual robot parameters)
        self.dh_params = [
            {'a': 0, 'alpha': -np.pi/2, 'd': 0.6, 'theta': 0},
            {'a': 0.675, 'alpha': 0, 'd': 0, 'theta': 0},
            {'a': 0.672, 'alpha': 0, 'd': 0, 'theta': 0},
            {'a': 0, 'alpha': -np.pi/2, 'd': 0.2, 'theta': 0},
            {'a': 0, 'alpha': np.pi/2, 'd': 0, 'theta': 0},
            {'a': 0, 'alpha': 0, 'd': 0.155, 'theta': 0}
        ]

    def dh_transformation(self, a, alpha, d, theta):
        """Calculate the DH transformation matrix."""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        """Compute the forward kinematics using the DH parameter method."""
        T = np.eye(4)
        for i in range(6):
            a = self.dh_params[i]['a']
            alpha = self.dh_params[i]['alpha']
            d = self.dh_params[i]['d']
            theta = joint_angles[i] + self.dh_params[i]['theta']
            T = np.dot(T, self.dh_transformation(a, alpha, d, theta))
        position = T[0:3, 3]
        return position

    def inverse_kinematics(self, target_position):
        """Calculate the inverse kinematics to find the joint angles for the target position."""
        def objective(joint_angles):
            end_effector_position = self.forward_kinematics(joint_angles)
            return np.linalg.norm(end_effector_position - target_position)

        # Initial guess for joint angles
        initial_guess = np.zeros(6)
        
        # Minimize the objective function
        result = minimize(objective, initial_guess, method='L-BFGS-B', bounds=[(-np.pi, np.pi)]*6)
        return result.x

    def listener_callback(self, msg):
        """Callback function to handle received coordinates and publish joint trajectory."""
        # Round coordinates
        x = round(msg.x, 2)
        y = round(msg.y, 2)
        z = round(msg.z, 2)

        # Log received coordinates
        self.get_logger().info(f'Received coordinates: x={x}, y={y}, z={z}')

        # Convert coordinates to joint positions
        joint_positions = self.convert_coordinates_to_joint_positions(x, y, z)

        # Create a JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # Create a JointTrajectoryPoint message
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 1  # Adjust as needed
        
        # Add the point to the trajectory
        traj_msg.points = [point]
        
        # Publish the trajectory command
        self.publisher_.publish(traj_msg)
        self.get_logger().info(f'Published trajectory command: {joint_positions}')

    def convert_coordinates_to_joint_positions(self, x, y, z):
        """Convert the target position (x, y, z) into joint positions using inverse kinematics."""
        target_position = np.array([x, y, z])
        joint_positions = self.inverse_kinematics(target_position)
        return joint_positions.tolist()

def main(args=None):
    rclpy.init(args=args)
    coordinate_subscriber = CoordinateSubscriber()
    rclpy.spin(coordinate_subscriber)
    coordinate_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CoordinateSubscriber(Node):

    def __init__(self):
        super().__init__('coordinate_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'camera/coordinates',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        x = round(msg.x, 2)
        y = round(msg.y, 2)
        z = round(msg.z, 2)
        self.get_logger().info(f'Received coordinates: x={x}, y={y}, z={z}')

def main(args=None):
    rclpy.init(args=args)
    coordinate_subscriber = CoordinateSubscriber()
    rclpy.spin(coordinate_subscriber)
    coordinate_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class GazeboCameraNode(Node):
    def __init__(self):
        super().__init__('gazebo_camera_node')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Point, 'camera/coordinates', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust this if necessary
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Timer for publishing coordinates every 10 seconds
        self.timer = self.create_timer(2, self.timer_callback)
        
        # Camera intrinsic parameters
        self.fx = 10477.87  # Focal length in x
        self.fy = 10477.45  # Focal length in y
        self.cx = 10477.45  # Optical center x
        self.cy = 10477.45  # Optical center y
        self.scale_factor = 1.0  # Scale factor for real-world conversion

        # Variable to store the latest coordinates
        self.latest_coords = None

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Simple object detection with color thresholding
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([0, 50, 50])  # Example lower bound for red color
        upper_bound = np.array([10, 255, 255])  # Example upper bound for red color
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cX = float(M['m10'] / M['m00'])
                cY = float(M['m01'] / M['m00'])
                
                # Convert pixel coordinates to real-world coordinates
                X_real = (cX - self.cx) / self.fx * self.scale_factor
                Y_real = (cY - self.cy) / self.fy * self.scale_factor

                X_real=round(X_real, 2)
                Y_real=round(Y_real, 2)

                
                # Update the latest coordinates
                self.latest_coords = (X_real, Y_real)
                self.get_logger().info(f'Updated coordinates: x={X_real}, y={Y_real}')
            else:
                self.latest_coords = None
        else:
            self.latest_coords = None

    def timer_callback(self):
        if self.latest_coords:
            coord_msg = Point()
            coord_msg.x = self.latest_coords[0]
            coord_msg.y = self.latest_coords[1]
            coord_msg.z = 1.0 # Assuming 2D detection; z-coordinate is 0
            self.publisher_.publish(coord_msg)
            self.get_logger().info(f'Published real-world coordinates: x={coord_msg.x}, y={coord_msg.y}')
        else:
            self.get_logger().info('No coordinates to publish')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

    def listener_callback(self, msg):
        # Round coordinates
        x = round(msg.x, 2)
        y = round(msg.y, 2)
        z = round(msg.z, 2)  # Assuming z is 0 for 2D detection

        # Log received coordinates
        self.get_logger().info(f'Received coordinates: x={x}, y={y}, z={z}')

        # Convert coordinates to joint positions
        # This is a placeholder. Replace with your actual logic for converting coordinates
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
        # Implement your logic to convert (x, y, z) to joint positions
        # For example, you might use inverse kinematics here
        # This is just a placeholder
        return [x, y, z, 0.0, 0.0, 0.0]

def main(args=None):
    rclpy.init(args=args)
    coordinate_subscriber = CoordinateSubscriber()
    rclpy.spin(coordinate_subscriber)
    coordinate_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

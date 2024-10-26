#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info("Turtle Controller Node Started")

        # Desired target coordinates for the turtle to reach
        self.target_x = 1  # Adjust as needed
        self.target_y = 1  # Adjust as needed

        # Publisher and Subscriber setup
        self.pose_subscription = self.create_subscription(
            Pose, 
            "/turtle1/pose", 
            self.pose_callback, 
            10
        )
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def pose_callback(self, msg: Pose):
        # Log the current pose of the turtle
        self.get_logger().info(f"Current Position - x: {msg.x}, y: {msg.y}, angle: {msg.theta}")
        
        # Calculate the error in position
        position_error_x = self.target_x - msg.x
        position_error_y = self.target_y - msg.y
        
        # Calculate the distance error (magnitude of the error vector)
        distance_error = (position_error_x**2 + position_error_y**2) ** 0.5
        
        # Log the position errors
        self.get_logger().info(f"Position Error - x: {position_error_x}, y: {position_error_y}")

        # Calculate the desired heading based on the position error
        desired_heading = math.atan2(position_error_y, position_error_x)
        
        # Calculate the error in heading
        heading_error = desired_heading - msg.theta
        
        # Handle angle wrap-around (e.g., +pi to -pi)
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi
        
        # Log the desired and current angles along with the heading error
        self.get_logger().info(f"Desired Heading: {desired_heading}, Current Angle: {msg.theta}, Heading Error: {heading_error}")

        # Proportional gain constants for controlling linear and angular velocities
        Kp_linear = 0.4
        Kp_angular = 2.0
        
        # PID control for linear velocity (distance control)
        linear_velocity = Kp_linear * abs(distance_error)

        # PID control for angular velocity (heading control)
        angular_velocity = Kp_angular * heading_error  

        # Send the calculated velocities to the turtle
        self.publish_velocities(linear_velocity, angular_velocity)

    def publish_velocities(self, linear_velocity, angular_velocity):
        # Log the commanded velocities
        self.get_logger().info(f"Publishing Velocities - Linear: {linear_velocity}, Angular: {angular_velocity}")
        
        # Create a Twist message to send velocities
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_velocity
        velocity_msg.angular.z = angular_velocity
        
        # Publish the velocity command
        self.velocity_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller_node = TurtleControllerNode()
    rclpy.spin(turtle_controller_node)
    turtle_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


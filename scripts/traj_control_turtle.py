#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import sqrt, atan2
import rclpy
from rclpy.node import Node

class TurtleControl(Node):
    
    def __init__(self):
        # Creates a node with name 'turtlebot_controller'
        super().__init__("simple_turtle_controller")
        
        # Publisher which will publish to the topic '/turtle1/cmd_vel'
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # A subscriber to the topic '/turtle1/pose'
        # self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        
        self.pose = Pose()
        self.goal_pose = None
        self.tolerance = 1
        self.goals = []
        self.current_goal_index = 0

        # Run the code inside the node every 0.1s/at a rate of 10Hz
        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Init done")
        
    def add_goal(self, x, y):
        # Create a list of goals and follow them one by one
        self.goals.append((x, y))

    def update_pose(self, msg):
        # Callback function which is called when a new message of type Pose is received by the subscriber
        self.pose = msg
        self.pose.x = round(self.pose.x,4)
        self.pose.y = round(self.pose.y,4)
    
    def euclidean_distance(self, goal_pose):
        # Euclidean distance between current pose and the goal
        xsquared = pow(goal_pose.x - self.pose.x,2)
        ysquared = pow(goal_pose.y - self.pose.y,2)
        return sqrt(xsquared + ysquared)
    
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    
    def linear_vel(self, goal_pose, constant = 1.5):
        # Simple gain control
        return constant * self.euclidean_distance(goal_pose)
    
    def angular_vel(self, goal_pose, constant = 6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta )
    
    def move_to_goal(self, goal_x, goal_y):
        # Move the turtle to the goal
        self.goal_pose = Pose()
        self.goal_pose.x = float(goal_x)
        self.goal_pose.y = float(goal_y)

        self.get_logger().info(f"Moving to pose {self.goal_pose.x} {self.goal_pose.y}")
                
    def timer_callback(self):
        # Proportional controller

        if self.goal_pose is None and self.current_goal_index < len(self.goals):
            x, y = self.goals[self.current_goal_index]
            self.move_to_goal(x, y)
        
        distance = self.euclidean_distance(self.goal_pose)
        vel_msg = Twist()

        if distance >= self.tolerance:
            # Linear velocity in the x-axis
            self.get_logger().info("Calculating velocities")
            vel_msg.linear.x = self.linear_vel(self.goal_pose)
            # Angular velocity in the z-axis
            vel_msg.angular.z = self.angular_vel(self.goal_pose)

            self.get_logger().info(f"Setting linear to {vel_msg.linear.x} and angular to {vel_msg.angular.z}")

        else:
            # Stopping our robot after the movement is over
            vel_msg.linear.x = 0.0
            vel_msg.linear.z = 0.0
            self.goal_pose = None
            # Move to the next goal
            self.current_goal_index += 1
            
            # Exit if all goals reached
            if self.current_goal_index >= len(self.goals):
                self.get_logger().info("All goals reached")
                rclpy.shutdown()

        self.velocity_publisher.publish(vel_msg)       
    
if __name__ == '__main__':
    rclpy.init()
    controller = TurtleControl()
    controller.add_goal(1,3)
    controller.add_goal(5,2)
    controller.add_goal(2,10)

    try:
        rclpy.spin(controller)
    finally:
        rclpy.shutdown()  

    

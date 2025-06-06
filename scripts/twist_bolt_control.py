#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import ColorRGBA, Float32
from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color

msg = """

---------------------------------------------
Your input value is beyond the defined range!
It will not be changed. 
---------------------------------------------

"""


def get_toys_list():
    toy_names = []
    toys = scanner.find_toys()

    if not toys:
        print('The Sphero-BOLTs were not found!')
        sys.exit()

    print('Following Sphero-BOLTs are discovered:')
    for t in toys:
        toy_names.append(t.name)
        print(t.name)

    return toys, toy_names


def connect_toy(toys_list, toy_names):
    toy_name = input("Which Sphero-BOLT do you want to use? Please enter its name: ")
    index = toy_names.index(toy_name) if toy_name in toy_names else 0
    print(f'Connect to your desired sphero-BOLT: {toys_list[index].name}')
    return toys_list[index]


class BoltControl(Node):

    def __init__(self, my_toy):
        super().__init__('twist_bolt_control')

        self.pose_pub = self.create_publisher(Pose, '/bolt/bolt_pose', 10)
        self.create_subscription(Float32, '/bolt/cmd_duration', self.duration_callback, 10)
        self.create_subscription(ColorRGBA, '/bolt/cmd_color', self.color_callback, 10)
        self.create_subscription(Twist, '/bolt/cmd_vel', self.sphero_callback, 10)

        self.speed = 0
        self.angular = 0
        self.duration = 0.5

        self.color_r = 255
        self.color_g = 128
        self.color_b = 128

        self.pose_msg = Pose()
        self.twist_msg = Twist()

        self.my_toy = my_toy
        self.sphero_bolt_start()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.cycle_limit = 1300

    def sphero_bolt_start(self):
        self.my_toy.set_main_led(Color(r=self.color_r, g=self.color_g, b=self.color_b))

    def get_sphero_bolt_pose(self):
        loc = self.my_toy.get_location()
        ori = self.my_toy.get_orientation()

        self.pose_msg.position.x = round(loc['x'], 4)
        self.pose_msg.position.y = round(loc['y'], 4)
        self.pose_msg.orientation.x = round(ori['pitch'], 4)
        self.pose_msg.orientation.y = round(ori['roll'], 4)
        self.pose_msg.orientation.z = round(ori['yaw'], 4)

        self.get_logger().info(f'Pose: {self.pose_msg.position.x}, {self.pose_msg.position.y}, Yaw: {self.pose_msg.orientation.z}')
        return self.pose_msg

    def timer_callback(self):
        self.counter += 1
        if self.counter >= self.cycle_limit:
            self.stop_movement()
            self.destroy_timer(self.timer)
        else:
            self.sphero_bolt_execute_action()

    def sphero_bolt_execute_action(self):
        self.my_toy.set_main_led(Color(r=self.color_r, g=self.color_g, b=self.color_b))
        self.my_toy.set_speed(0)
        self.my_toy.roll(self.angular, self.speed, self.duration)
        self.pose_pub.publish(self.get_sphero_bolt_pose())

    def stop_movement(self):
        self.my_toy.set_speed(0)
        self.my_toy.strobe(Color(255, 0, 0), (3 / 15) * .5, 15)
        self.get_logger().info("Stopping Sphero BOLT...")

    def sphero_callback(self, msg):
        self.speed = int(round(msg.linear.x))
        self.angular += int(round(msg.angular.z))
        self.get_logger().info(f"Received cmd_vel: speed={self.speed}, angular={self.angular % 360}")

    def duration_callback(self, msg):
        if 0.1 < msg.data < 9:
            self.duration = msg.data
            self.get_logger().info(f"Updated duration: {self.duration}")
        else:
            print(msg)

    def color_callback(self, msg):
        if all(0 <= val <= 255 for val in [msg.r, msg.g, msg.b]):
            self.color_r = int(msg.r)
            self.color_g = int(msg.g)
            self.color_b = int(msg.b)
            self.get_logger().info(f"Updated color: ({self.color_r}, {self.color_g}, {self.color_b})")
        else:
            print(msg)


def main(args=None):
    rclpy.init(args=args)
    toys, toy_names = get_toys_list()
    toy = connect_toy(toys, toy_names)

    if toy is not None:
        with SpheroEduAPI(toy) as toy_:
            node = BoltControl(toy_)
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.stop_movement()
                node.get_logger().info("Shutting down from keyboard interrupt.")
            finally:
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    main()

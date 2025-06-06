#!/usr/bin/env python3

import sys
import select
import tty
import termios
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Float32

msg_in = """
Input arguments: filename duration(2~8) color-r(0~255) color-g(0~255) color-b(0~255)
Example:
ros2 run sphero_twist twist_bolt_teleop 3 255 0 255
"""

msg = """
Control your Sphero-Bolt!
---------------------------
Moving around:
        w
   a    s    d

space key or p : force stop

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def getDurationAndColor(_inputValue):
    durationValue_ = float(_inputValue[0])
    color_r_ = float(_inputValue[1])
    color_g_ = float(_inputValue[2])
    color_b_ = float(_inputValue[3])
    color_a_ = 1.0
    return durationValue_, color_r_, color_g_, color_b_, color_a_

class TwistBoltTeleopNode(Node):
    def __init__(self, duration, r, g, b, a):
        super().__init__('twist_bolt_teleop')

        self.duration = duration
        self.color = ColorRGBA(r=r, g=g, b=b, a=a)

        self.pub_action = self.create_publisher(Twist, '/bolt/cmd_vel', 10)
        self.pub_duration = self.create_publisher(Float32, '/bolt/cmd_duration', 10)
        self.pub_color = self.create_publisher(ColorRGBA, '/bolt/cmd_color', 10)

        self.vel = 0.0
        self.angular_vel = 0.0

    def run(self):
        print(msg)

        try:
            while rclpy.ok():
                key = getKey()
                if key == '':
                    continue
                elif key == 'w':
                    self.vel = 20.0
                    self.angular_vel = 0.0
                elif key == 's':
                    self.vel = -20.0
                    self.angular_vel = 0.0
                elif key == 'a':
                    self.vel = 0.0
                    self.angular_vel = -90.0
                elif key == 'd':
                    self.vel = 0.0
                    self.angular_vel = 90.0
                elif key == ' ' or key == 'p':
                    self.vel = 0.0
                    self.angular_vel = 0.0
                elif key == '\x03':  # Ctrl-C
                    break

                twist = Twist()
                twist.linear.x = self.vel
                twist.angular.z = self.angular_vel

                duration_msg = Float32()
                duration_msg.data = self.duration

                self.pub_action.publish(twist)
                self.pub_duration.publish(duration_msg)
                self.pub_color.publish(self.color)

                self.get_logger().info(f"Sent speed: {self.vel} -- angular speed: {self.angular_vel}")

        except Exception as e:
            self.get_logger().error(f"Failed: {e}")

        finally:
            # Stop the robot
            twist = Twist()
            self.pub_action.publish(twist)
            self.get_logger().info("The program terminated with keyboard interrupt.")


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    if len(sys.argv) != 5:
        print(msg_in)
        sys.exit()

    durationValue, r, g, b, a = getDurationAndColor(sys.argv[1:])

    rclpy.init()
    node = TwistBoltTeleopNode(durationValue, r, g, b, a)
    node.run()
    node.destroy_node()
    rclpy.shutdown()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
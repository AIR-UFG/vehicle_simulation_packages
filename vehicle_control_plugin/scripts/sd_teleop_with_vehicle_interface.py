#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')

        self.declare_parameter('velocity_increase_rate', 7.5)
        self.velocity_increase_rate = self.get_parameter('velocity_increase_rate').get_parameter_value().double_value

        self.declare_parameter('steering_increase_rate', 0.025)
        self.steering_increase_rate = self.get_parameter('steering_increase_rate').get_parameter_value().double_value

        self.declare_parameter('max_velocity', 1000.0)
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        self.declare_parameter('max_steering_angle', 0.35)
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value

        self.publisher = self.create_publisher(TwistStamped, 'twist_cmd', 10)
        self.twist = TwistStamped()
        self.current_velocity = 0.0
        self.current_steering_angle = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

        print("""
Reading from the keyboard and Publishing to TwistStamped!
Uses "w, a, s, d, x" keys
---------------------------
Move forward:
   'w'
Move backward:
    's'
Turn left:
    'a'
Turn right:
    'd'
Stop:
    'x'
    
CTRL-C to quit
""")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                if key == 'w':
                    self.current_velocity += self.velocity_increase_rate
                elif key == 's':
                    self.current_velocity -= self.velocity_increase_rate
                elif key == 'a':
                    self.current_steering_angle += self.steering_increase_rate
                elif key == 'd':
                    self.current_steering_angle -= self.steering_increase_rate
                elif key == 'x':
                    self.current_velocity = 0.0
                    self.current_steering_angle = 0.0
                elif key == '\x03':  # CTRL-C
                    break

                self.current_velocity = min(max(self.current_velocity, -self.max_velocity), self.max_velocity)
                self.current_steering_angle = min(max(self.current_steering_angle, -self.max_steering_angle), self.max_steering_angle)

                self.twist.twist.linear.x = self.current_velocity
                self.twist.twist.angular.z = self.current_steering_angle
                self.publisher.publish(self.twist)

                print(f"Velocity: {self.current_velocity}, Steering Angle: {self.current_steering_angle}")

        except Exception as e:
            print(e)
        finally:
            self.twist.twist.linear.x = 0.0
            self.twist.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


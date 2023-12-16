"""
Neato Teleoperator:
w = forward, a = left, s = back, d = right, e = stop
"""

import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Teleop(Node):

    def __init__(self):
        """
        Constructor to construct objects:
        in this case, cmd_vel messages of type Twist
        """
        super().__init__('teleop')
        
        # creates a timer for calling run_loop
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.run_loop)
            # makes a timer to schedule run_loop method to be called every timer_period seconds
        self.vel = Twist() # defines global "vel" variable as type Twist
            # Twist type expresses velocity in free space broken into linear and angular parts

        # publishing velocity to 'cmd_vel' topic!
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
            # message type, topic, QoS profile (number of messages stored at a time)

    def getKey(self):
        """
        Retrieves user's keyboard inputs.
        """
        settings = termios.tcgetattr(sys.stdin)
        self.key = None
        tty.setraw(sys.stdin.fileno())
            # tty.setraw changes file descriptor (sys here) to raw, part of the termios module
            # stdin (standard input) is a task that reads keyboard inputs from a user -- 
                # this is reading from stdin
            # fileno() method returns integer file descriptor that's used in data structure 
                # linux references while program is running
            
            # takes integer file descriptor of standard input keyboard press and converts it to
                # raw mode (raw mode sends input after it's written and doesn't do a carriage 
                    # return -- carriage return moves cursor to first position of same line)
            
            # this allows program to read keyboard inputs as they are made

        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return
    
    def run_loop(self):
        """
        Responds to keyboard input and publishes the velocity accordingly
            w: forward
            a: turn left
            s: backwards
            d: turn right
            e: stop
        """
        self.getKey()
        print(self.key)

        if self.key == 'w': # go forward
            self.vel.linear.x = 1.0 # meters/s
        elif self.key == 'a': # turn left
            self.vel.angular.z = 1.0 # radians/s
        elif self.key == 's': # go backward
            self.vel.linear.x = -1.0 # meters/s
        elif self.key == 'd': # turn right
            self.vel.angular.z = 1.0 # radians/s
        elif self.key == 'e': # stop
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
        elif self.key == '\x03': # ctrl+c
            raise KeyboardInterrupt
        self.vel_publisher.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
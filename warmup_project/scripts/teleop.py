#!/usr/bin/env python

import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import rospy

#initialize node and begin publishing to /cmd_vel to cause movement
rospy.init_node('better_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()

def getKey():
    """Provided function that waits for and returns key presses as they happen"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None

#while the exit character is not pressed, take basic instruction on how the robot should move.
while key != '\x03':
    key = getKey()
    
    if key == 'w':
      twist.linear.x = 1
      twist.angular.z = 0
    elif key == 's':
      twist.linear.x = -1
      twist.angular.z = 0
    elif key == 'a':
      twist.linear.x = 0
      twist.angular.z = 1
    elif key == 'd':
      twist.linear.x = 0
      twist.angular.z = -1
    else:
      twist.linear.x = 0
      twist.angular.z = 0
    
    pub.publish(twist)
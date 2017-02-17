#!/usr/bin/env python

"""
Drives the robot in a square using the onboard odometer to keep track of the angle and distance of the robot
 to determine how much to turn and how far to go in order to form a square.
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import rospy
import tf
from tf.transformations import euler_from_quaternion
from math import pi

class DriveSquare:
  def __init__(self):
    #instantiate instance variables
    self.twist = Twist()
    self.twist.linear.x = 1
    self.running_total = 0
    self.turn = False
    self.drive = True
    self.previous_rot = 'default'
    self.last_turn_x = 0
    self.last_turn_y = 0
    self.x = 0
    self.y = 0
    self.rot = 0
    self.rot_error = 0

    #instantiate rospy node, publisher and subscriber
    rospy.init_node('square_instructions')
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.sub = rospy.Subscriber('/odom', Odometry, self.process_odom)

    self.do_the_thing()

  def process_odom(self, msg):
    """Handles the strange behavior of the odometry coordinates when passing 360 degrees of rotation allowing
    for a continuous counter"""
    pose = msg.pose.pose
    self.x, self.y, self.rot = self.convert_pose_to_xy_and_theta(pose)
    temp = 'jump'

    if self.previous_rot == 'default':
      self.previous_rot = self.rot
    if (self.previous_rot > 0) and (self.rot < 0):
      temp = self.rot
      self.rot += 2*pi

    self.rot_error = abs(self.rot - self.previous_rot)
    self.running_total += self.rot_error
    if temp != 'jump':
      self.rot = temp
    self.previous_rot = self.rot

  def set_vals(self, speed=0, spin=0):
    """Sets twist values to be as the arguments specify or stops all movement"""
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

  def go_forward(self):
    self.set_vals(speed=1)

  def turn_left(self):
    self.set_vals(spin=.1)

  def dist_from_last_turn(self):
    """Keeps track of how far the robot has traveled since the last turn to force consistency in side lengths
    of the square"""
    x_diff = self.x - self.last_turn_x
    y_diff = self.y - self.last_turn_y
    return ((x_diff)**2 + (y_diff)**2)**.5

  def convert_pose_to_xy_and_theta(self, pose):
    """Provided function that converts the quaternary results of the onboard odometer into usable
    x,y, and theta coordinates."""
      orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
      angles = euler_from_quaternion(orientation_tuple)
      return (pose.position.x, pose.position.y, angles[2])

  def do_the_thing(self):
    """Main function that handles publishing the twist to velocity, coordinating turns and distances, and 
    monitoring the running tally of multiple instance variables"""
    self.pub.publish(self.twist)
    while not rospy.is_shutdown():
      if self.running_total >= (pi/2 - self.rot_error/2):
        print self.running_total
        self.go_forward()
        self.running_total = 0
      elif self.dist_from_last_turn() >= 1:
        self.turn_left()
        self.last_turn_x = self.x
        self.last_turn_y = self.y
      self.pub.publish(self.twist)
      # r.sleep()

if __name__ == "__main__":
  DriveSquare()
#!/usr/bin/env python

"""Obstacle avoiding behavior designed to use two states to navigate around obstacles and try to go in a
particular direction. The direction is always the direction the robot starts out facing. The robot dodges
obstacles by switching to a 'wall following' state until the robot doesn't detect anything in the way,
and returns to the goal following state and continues heading in the original direction.
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rospy
import tf
from tf.transformations import euler_from_quaternion
from math import pi
from wall_follower import FollowWall

class ObstacleAvoider:
  def __init__(self):
    #instantiate instance variables
    self.twist = Twist()
    self.twist.linear.x = 1

    self.running_total = 0
    self.previous_rot = 'default'
    self.x = 0
    self.y = 0
    self.rot = 0
    self.rot_error = 0
    self.avoid_dist = 1
    self.ranges = [0]*360
    self.wf = FollowWall()
    #setup node, publishing, and subscribing
    rospy.init_node('avoid_instructions')
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.sub = rospy.Subscriber('/odom', Odometry, self.process_odom)
    self.laser_sub = rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)

    self.pub.publish(self.twist)

  def process_odom(self, msg):
    """Converts odometry coordinates from quaternary to a more useable x,y,theta format"""
    pose = msg.pose.pose
    self.x, self.y, self.rot = self.convert_pose_to_xy_and_theta(pose)
    print(self.rot)

  def process_scan(self, msg):
    """Collects laser scan values for use elsewhere in the file"""
    self.ranges = msg.ranges

  def set_vals(self, speed=0, spin=0):
    """Sets values based on the inputs, or stops all motion if no arguments are provided"""
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin
    self.pub.publish(self.twist)

  def go_forward(self):
    self.set_vals(speed=1)

  def turn_right(self):
    self.set_vals(spin=-.1)

  def turn_left(self):
    self.set_vals(spin=.1)

  def make_a_turn(self, direction):
    """Based on which direction was passed, the function makes a turn at exactly the right angle before
    beginning wall following behavior."""
    #turn left or right depending on the value of direction
    direction = 1
    if direction > 0:
      while self.rot < 1.5:
        self.set_vals(spin=.1*direction)
    else:
      while self.rot > -1.5:
        self.set_vals(spin=.1*direction)

    #set specific values for what angle should be watched based on which side of the robot is facing the wall
    self.running_total = 0
    self.go_forward()
    if direction > 0:
      side = 270
    elif direction < 0:
      side = 90
    while self.ranges[side] < self.avoid_dist:
      self.wf.do_the_thing()
    while self.rot > .05 or self.rot < -.05:
      self.set_vals(spin=-.4*(self.rot/abs(self.rot)))
    self.go_forward()

  def convert_pose_to_xy_and_theta(self, pose):
    """Provided function that converts quaternary to x,y,theta coordinates"""
      orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
      angles = euler_from_quaternion(orientation_tuple)
      return (pose.position.x, pose.position.y, angles[2])

  def do_the_thing(self):
    """Main function responsible for starting the process and setting up relevant scan ranges for obstacles
    to avoid"""
    self.go_forward()

    cone_ranges = self.ranges[-30:] + self.ranges[:30]
    in_range_num = len([x for x in cone_ranges if x < 1 and not x==0])
    if in_range_num > 0 :
      self.make_a_turn(-1)

    self.pub.publish(self.twist)

if __name__ == "__main__":
  oa = ObstacleAvoider()
  while not rospy.is_shutdown():
    oa.do_the_thing()
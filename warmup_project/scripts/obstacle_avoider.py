#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import rospy
import tf
from tf.transformations import euler_from_quaternion
from math import pi
from wall_follower import FollowWall

class ObstacleAvoider:
  def __init__(self):
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

    rospy.init_node('avoid_instructions')
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.sub = rospy.Subscriber('/odom', Odometry, self.process_odom)
    self.laser_sub = rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)

    self.pub.publish(self.twist)

  def process_odom(self, msg):
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

  def process_scan(self, msg):
    self.ranges = msg.ranges

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

  def go_forward(self):
    self.set_vals(speed=1)

  def turn_right(self):
    self.set_vals(spin=-.1)

  def turn_left(self):
    self.set_vals(spin=.1)

  def make_a_turn(self, direction):
    self.set_vals(spin=.1*direction)
    while self.running_total < (pi/2 - self.rot_error/2):
      pass
    self.running_total = 0
    self.go_forward()
    wf = FollowWall()
    if direction > 0:
      side = 270
    elif direction < 0:
      side = 90
    while self.ranges[side] < self.avoid_dist:
      wf.do_the_thing()
    ### Implement returning to forward

  def convert_pose_to_xy_and_theta(self, pose):
      orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
      angles = euler_from_quaternion(orientation_tuple)
      return (pose.position.x, pose.position.y, angles[2])

  def do_the_thing(self):
    self.go_forward()

    cone_ranges = self.ranges[-30:] + self.ranges[:30]
    in_range_num = len([x for x < 1 in cone_ranges])
    if (in_range_num > 0) and (cone_ranges[0] == 0):
      self.make_a_turn(1)
    elif (in_range_num > 0) and (cone_ranges[-1] == 0):
      self.make_a_turn(-1)

    self.pub.publish(self.twist)

if __name__ == "__main__":
  oa = ObstacleAvoider()
  while not rospy.is_shutdown():
    oa.do_the_thing()
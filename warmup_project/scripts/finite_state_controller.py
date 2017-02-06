#!/usr/bin/env python

""" Print distance to object immediately in front of the robot """

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from neato_node.msg import Bump
import rospy
import random

class FiniteStateController:
  def __init__(self):
    self.bumped = 0
    rospy.init_node('e_stop')
    self.sub = rospy.Subscriber('/bump', Bump, self.process_bump)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.twist.linear.x = 1
    self.do_the_thing()
    print "Node is finished!"

  def process_bump(self, msg):
    self.bumped = msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin
    self.pub.publish(self.twist)

  def do_the_thing(self):
    if self.bumped:
      # deer in headlights
      self.set_vals()
    else:
      # freak out
      speed = 2*(random.random()) - 1
      spin = 2*(random.random()) - 1
      self.set_vals(speed=speed, spin=spin)
    self.pub.publish(self.twist)

if __name__ == "__main__":
  fsc = FiniteStateController()
  while not rospy.is_shutdown():
    fsc.do_the_thing()
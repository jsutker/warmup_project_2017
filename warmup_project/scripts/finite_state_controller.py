#!/usr/bin/env python

""" Print distance to object immediately in front of the robot """

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from neato_node.msg import Bump
import rospy
import random
from person_follower_helper import PersonFollower

class FiniteDanceController:
  def __init__(self):
    self.previous_bumped = 0
    self.bumped = 0
    self.dancing = False
    rospy.init_node('dancing_machine')
    self.sub = rospy.Subscriber('/bump', Bump, self.process_bump)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    
    self.pf = PersonFollower()

    print "Node is finished!"

  def process_bump(self, msg):
    self.previous_bumped = self.bumped
    self.bumped = msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide
    if (self.bumped > 0) and (self.previous_bumped == 0):
      self.dancing = not self.dancing

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin
    self.pub.publish(self.twist)

  def dance(self):
    speed = 2*(random.random()) - 1
    spin = 2*(random.random()) - 1
    self.set_vals(speed=speed, spin=spin)

  def do_the_thing(self):
    r = rospy.Rate(10)
    if self.dancing:
      # print("Dancing!")
      self.dance()
    else:
      print("Not dancing :(")
      self.pf.do_the_thing()
    r.sleep()    
    self.pub.publish(self.twist)

if __name__ == "__main__":
  fsc = FiniteDanceController()
  while not rospy.is_shutdown():
    fsc.do_the_thing()
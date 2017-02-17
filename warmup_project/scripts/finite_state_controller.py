#!/usr/bin/env python

""" A basic finite state controller demonstrating a person follower and a 'dancing' method. The dancing
state consists of randomly assigning twist values. """

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from neato_node.msg import Bump
import rospy
import random
from person_follower_helper import PersonFollower

class FiniteDanceController:
  def __init__(self):
    #instantiate instance variables
    self.previous_bumped = 0
    self.bumped = 0
    self.dancing = False
    #instantiate node, subscription, and publisher
    rospy.init_node('dancing_machine')
    self.sub = rospy.Subscriber('/bump', Bump, self.process_bump)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    #instantiate person following behavior
    self.pf = PersonFollower()

    print "Node is finished!"

  def process_bump(self, msg):
    """Handles the switching behavior between person following and dancing"""
    self.previous_bumped = self.bumped
    self.bumped = msg.leftFront + msg.rightFront + msg.leftSide + msg.rightSide
    if (self.bumped > 0) and (self.previous_bumped == 0):
      self.dancing = not self.dancing

  def set_vals(self, speed=0, spin=0):
    """Either sets the values of the twist to be the input parameters or stops all motion"""
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin
    self.pub.publish(self.twist)

  def dance(self):
    """Random movement that vaguely simulates dancing"""
    speed = 2*(random.random()) - 1
    spin = 2*(random.random()) - 1
    self.set_vals(speed=speed, spin=spin)

  def do_the_thing(self):
    """Main function that handles keeping track of which state the code is in and what functions should be run.
    Additionally handles publishing to velocity."""
    r = rospy.Rate(10)
    if self.dancing:
      print("Dancing!")
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
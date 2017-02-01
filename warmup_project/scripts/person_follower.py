#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

class DistEmergencyStop:
  def __init__(self):
    cone_width = 60 # if odd, truncates to cone_width - 1
    self.cone_left  = -1*(cone_width/2)
    self.cone_right = (cone_width/2) + 1
    self.perim_dist = 1.0
    self.follow_max = 2.0
    self.follow_min = 0.5

    # add set up for storing msg.ranges

    rospy.init_node('person_follow')
    self.sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.do_the_thing()
    print "Node is finished!"

  def process_scan(self, msg):
    self.

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

  def wait(self):
    pass

  def follow(self):
    pass

  def do_the_thing(self):
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
      if self.frontDist < 0.6:
        self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
      self.pub.publish(self.twist)

      r.sleep()

estop = DistEmergencyStop()
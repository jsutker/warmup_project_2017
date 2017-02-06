#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

class PersonFollower:
  def __init__(self):
    self.current_state = "wait"
    self.cone_width = 40 # if odd, truncates to self.cone_width - 1
    self.cone_left  = -1*(self.cone_width/2)
    self.cone_right = (self.cone_width/2) + 1
    self.perim_dist = 1.0
    self.follow_max = 2.0
    self.follow_min = 0.75
    self.ranges = [0]*360

    self.nearest_deg = 0
    self.nearest_deg_dist = 0

    # rospy.init_node('person_follow')
    self.sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()

    print "Node is finished!"

  def process_scan(self, msg):
    self.ranges = msg.ranges

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

  def wait(self):
    self.set_vals(spin=.2)
    nearest_deg = 0
    nearest_deg_dist = self.perim_dist + 1
    for i, x in enumerate(self.ranges):
      if (x != 0) and (x < nearest_deg_dist):
        nearest_deg = i
        nearest_deg_dist = x
    if nearest_deg_dist < self.perim_dist:
      nearest_deg = ((nearest_deg + 180) % 360) - 180
      self.center(degree=nearest_deg)
      self.current_state = "follow"

  def follow(self):
    cone_ranges = self.ranges[self.cone_left:] + self.ranges[:self.cone_right]
    nearest_deg = 0
    nearest_deg_dist = self.follow_max + 1
    for i, x in enumerate(cone_ranges):
      if (x != 0) and (x < nearest_deg_dist):
        nearest_deg = i - (self.cone_width/2)
        nearest_deg_dist = x
    if nearest_deg_dist < self.follow_min:
      self.center(degree=nearest_deg)
    elif nearest_deg_dist < self.follow_max:
      follow_speed = (nearest_deg_dist - self.follow_min)/(self.follow_max - self.follow_min)
      self.center(speed=follow_speed, degree=nearest_deg)
    else:
      self.current_state = "wait"

  def center(self, degree=0, speed=0):
    if degree == 0:
      sign = 1
    else:
      sign = (degree/abs(degree))

    spin = sign*((abs(degree)**.5)/10)
    self.set_vals(speed=speed, spin=spin)

  def do_the_thing(self):
    # r = rospy.Rate(2)
    if self.current_state == "wait":
      self.wait()
    elif self.current_state == "follow":
      self.follow()

    self.pub.publish(self.twist)

    # r.sleep()


if __name__ == "__main__":
  pfollow = PersonFollower()
  while not rospy.is_shutdown():
    pfollow.do_the_thing()
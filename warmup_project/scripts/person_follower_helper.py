#!/usr/bin/env python

"""
The person follower code uses the laser scanner to detect the nearest object within a specified distance and 
focusses on that object. If the object moves, the robot makes an effort to keep the object relatively in 
front of the robot and within a certain distance of the front of the robot. When it can't find anything in 
the surrounding area close enough to focus on, it spins in a circle idly.
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy

class PersonFollower:
  def __init__(self):
    #robot has two states, wait and follow
    self.current_state = "wait"
    self.cone_width = 40 # if odd, truncates to self.cone_width - 1
    #determine which degrees of laser scanner are relevant for the focusing point
    self.cone_left  = -1*(self.cone_width/2)
    self.cone_right = (self.cone_width/2) + 1
    self.perim_dist = 1.0
    self.follow_max = 2.0
    self.follow_min = 0.75
    self.ranges = [0]*360

    self.nearest_deg = 0
    self.nearest_deg_dist = 0

    #instantiate rospy node and subscriptions/publishings
    self.sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()

    print "Node is finished!"

  def process_scan(self, msg):
    #make the scans accessible elsewhere in the class
    self.ranges = msg.ranges

  def set_vals(self, speed=0, spin=0):
    """Helper function designed to take in a speed and angular velocity and set the appropriate values
    in the twist for later publishing."""
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

  def wait(self):
    """When in the waiting state, the robot waits for an object to move within range and continues scanning
    until that happens.
    """
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
    """If the robot is in the follow state, the robot attempts to follow the object it has noticed
    to the best of its ability. It narrows the field of view it pays attention to and attempts to 
    keep the object within that cone. When the object moves far away or closer, left or right, the
    robot moves to maintain distance and focus on the object."""
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
    """Centers the robot on whatever is closest to it."""
    if degree == 0:
      sign = 1
    else:
      sign = (degree/abs(degree))

    spin = sign*((abs(degree)**.5)/10)
    self.set_vals(speed=speed, spin=spin)

  def do_the_thing(self):
    """Main function that regulates which state based function is being called and publishes the resulting
    twist information."""
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
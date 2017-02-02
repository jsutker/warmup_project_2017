#!/usr/bin/env python

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3

class FollowWall:
  def __init__(self):
    self.twist = Twist()

    self.fore = 0
    self.aft = 0
    self.previous_fore = 0
    self.previous_aft = 0
    self.pls_stop = 1
    self.current_marker_id = 0
    self.fore_x = self.fore*(2**(-.5))
    self.fore_y = -1*self.fore*(2**(-.5))
    self.aft_x = -1*self.aft*(2**(-.5))
    self.aft_y = -1*self.aft*(2**(-.5))

    rospy.init_node('wall_follower')
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.marker_pub = rospy.Publisher('/visualization_messages/Marker', Marker, queue_size=10)
    self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)

  def process_scan(self, msg):
    fore_deg = 315
    aft_deg = 225
    self.previous_fore = self.fore
    self.previous_aft = self.aft
    self.fore = msg.ranges[fore_deg]
    self.aft = msg.ranges[aft_deg]
    self.pls_stop = msg.ranges[0]

    self.fore_x = self.fore*(2**(-.5))
    self.fore_y = -1*self.fore*(2**(-.5))
    self.aft_x = -1*self.aft*(2**(-.5))
    self.aft_y = -1*self.aft*(2**(-.5))

  def create_marker(self, x=0, y=0):
    scale = Vector3(x=.1, y=.1, z=.1)
    pt = Point(x=x, y=y)
    pose = Pose(position=pt)
    mark = Marker(type=2, pose=pose, scale=scale)
    mark.color.a = 1
    mark.color.b = 1
    mark.color.r = .5
    mark.color.g = .5
    mark.header.frame_id = "base_link"
    mark.id = self.current_marker_id
    self.current_marker_id += 1
    return mark

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

  def do_the_thing(self):
    wall_dist_sum = 2.6
    dist_sum = self.fore + self.aft
    previous_dist_sum = self.previous_fore + self.previous_aft
    spin_factor = 1.2

    self.set_vals(speed=1, spin=spin_factor*(wall_dist_sum - dist_sum))

    if self.aft and not self.fore:
      self.set_vals(speed=1, spin=-1*spin_factor*(wall_dist_sum - dist_sum))
    elif not (self.fore or self.aft):
      self.set_vals(speed=1, spin=spin_factor*(wall_dist_sum - previous_dist_sum))
      if not (self.previous_fore or self.previous_aft):
        self.set_vals(speed=.1)

    if (self.pls_stop < 1) and (self.pls_stop != 0):
      self.set_vals(spin=.5)
    elif (self.twist.linear.x > .2) and self.fore and self.aft:
      self.marker_pub.publish(self.create_marker(self.fore_x, self.fore_y))
      self.marker_pub.publish(self.create_marker(self.aft_x, self.aft_y))

    print self.twist.linear.x
    self.pub.publish(self.twist)

if __name__ == "__main__":
  fw = FollowWall()
  while not rospy.is_shutdown():
    fw.do_the_thing()
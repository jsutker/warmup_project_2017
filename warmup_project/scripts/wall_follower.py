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

    rospy.init_node('wall_follower')
    self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.marker_pub = rospy.Publisher('/visualization_messages/Marker', Marker, queue_size=10)
    self.sub = rospy.Subscriber('/stable_scan', LaserScan, self.process_scan)

    self.do_the_thing()

  def process_scan(self, msg):
    fore_deg = 315
    aft_deg = 225
    self.previous_fore = self.fore
    self.previous_aft = self.aft
    self.fore = msg.ranges[fore_deg]
    self.aft = msg.ranges[aft_deg]
    self.pls_stop = msg.ranges[0]

  def create_marker(self):
    scale = Vector3(x=.1, y=.1, z=.1)
    pt = Point(x=1, y=2)
    pose = Pose(position=pt)
    mark = Marker(type=2, pose=pose, scale=scale)
    mark.color.a = 1
    mark.color.b = 1
    mark.color.r = .5
    mark.color.g = .5
    mark.header.frame_id = "odom"
    return mark

  def set_vals(self, speed=0, spin=0):
    self.twist.linear.x = speed; self.twist.linear.y = 0; self.twist.linear.z = 0
    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = spin

  def do_the_thing(self):
    # r = rospy.Rate(100)

    while not rospy.is_shutdown():
      
      wall_dist_sum = 2.6
      dist_sum = self.fore + self.aft
      previous_dist_sum = self.previous_fore + self.previous_aft
      spin_factor = 1.2

      print '---------------'
      print self.pls_stop
      print self.fore
      print self.aft

      self.set_vals(speed=1, spin=spin_factor*(wall_dist_sum - dist_sum))

      if not (self.fore and self.aft):
        self.set_vals(speed=1, spin=spin_factor*(wall_dist_sum - previous_dist_sum))
        if not (self.previous_fore and self.previous_aft):
          self.set_vals(speed=1)


      print "stopping?"
      if (self.pls_stop < 1) and (self.pls_stop != 0):
        print "STOPPING"
        self.set_vals(spin=.5)

      print self.twist.linear.x
      self.pub.publish(self.twist)

      # r.sleep()

if __name__ == "__main__":
  FollowWall()
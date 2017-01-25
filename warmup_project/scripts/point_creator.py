#!/usr/bin/env python

""" Creates point """

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3
import rospy

class PointCreator:
  def __init__(self):
    rospy.init_node('surprise_point')
    self.pub = rospy.Publisher('/visualization_messages/Marker', Marker, queue_size=10)
    self.point = self.create_marker()
    self.do_the_thing()
    print "Node is finished!"

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

  def do_the_thing(self):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():

      self.pub.publish(self.point)

      r.sleep()

if __name__=="__main__":
  pc = PointCreator()
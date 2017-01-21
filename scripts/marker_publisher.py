#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def run():
  rospy.init_node('marker_publisher')
  publisher = rospy.Publisher('/my_marker', Marker, queue_size=10)

  marker = Marker()
  marker.header.frame_id = "/odom"
  marker.type = marker.SPHERE
  marker.action = marker.ADD
  marker.scale.x = 0.2
  marker.scale.y = 0.2
  marker.scale.z = 0.2
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  marker.pose.position.x = 1
  marker.pose.position.y = 2
  marker.pose.position.z = 0

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    publisher.publish(marker)
    r.sleep()

if __name__ == "__main__":
  run()
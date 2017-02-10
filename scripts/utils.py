#!/usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker
import math

def poseToXYTheta(pose):
  """
  Converts the transform and rotation data from a pose into an
  (x, y, theta) tuple. Fully derived from Paul Ruvolo's
  2015 comprobo libraries.
  """

  orientation = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w
  )

  return (
    pose.position.x,
    pose.position.y,
    euler_from_quaternion(orientation)[2]
  )

def wrapPi(val):
  """
  Wraps the input value so that it is always within
  the range [-pi, pi]. Derived from Paul Ruvolo's 2015
  comprobo libraries.
  """

  return math.atan2(math.sin(val), math.cos(val))

def diffAngle(a, b):
  """
  Calculates the difference between angle a and angle b (both should be in radians)
  the difference is always based on the closest rotation from angle a to angle b.
  Derived from Paul Ruvolo's 2015 comprobo libraries
  """
  a = wrapPi(a)
  b = wrapPi(b)
  d1 = a-b
  d2 = 2*math.pi - math.fabs(d1)
  if d1 > 0:
    d2 *= -1.0
  return d1 if math.fabs(d1) < math.fabs(d2) else d2

def marker(
  theta = 0,
  position = (0, 0, 0),
  scale = (0.2, 0.2, 0.2),
  rgba=(1.0, 1.0, 0.0, 1.0),
  frame="/base_link",
  markerType="ARROW"
):
  markerTypes = [
    "ARROW",
    "CUBE",
    "SPHERE",
    "CYLINDER",
    "LINE_STRIP",
    "LINE_LIST",
    "CUBE_LIST",
    "SPHERE_LIST",
    "POINTS",
    "TEXT_VIEW_FACING",
    "MESH_RESOUCE",
    "TRIANGLE_LIST"
  ]

  m = Marker()
  m.header.frame_id = frame
  m.type = markerTypes.index(markerType)
  m.action = m.ADD

  m.scale.x = scale[0]
  m.scale.y = scale[1]
  m.scale.z = scale[2]

  m.color.r = rgba[0]
  m.color.g = rgba[1]
  m.color.b = rgba[2]
  m.color.a = rgba[3]

  m.pose.position.x = position[0]
  m.pose.position.y = position[1]
  m.pose.position.z = position[2]

  q = quaternion_from_euler(0, 0, theta)

  m.pose.orientation.x = q[0]
  m.pose.orientation.y = q[1]
  m.pose.orientation.z = q[2]
  m.pose.orientation.w = q[3]

  return m

def clearMarkers():
  m = Marker()
  m.action = 3
  return m

def dist(point1, point2):
  return math.hypot(point1.x-point2.x, point1.y-point2.y)
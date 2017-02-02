#!/usr/bin/env python

from tf.transformations import euler_from_quaternion
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
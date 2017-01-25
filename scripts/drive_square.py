#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

DIRECTIONS = {
  'FORWARD': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)),
  'BACKWARD': Twist(linear=Vector3(-1, 0, 0), angular=Vector3(0, 0, 0)),
  'LEFT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 1)),
  'RIGHT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -1)),
  'STOP': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
}

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

class DriveSquare:
  """
  This class contains the behavior for driving in a square. On initialization,
  it creates a ros node with a publisher and subscriber and begins to attempt
  to drive either straight or turn. It hijacks the cmd_vel topic, so it will
  interfere with standard teleoperation.
  """

  def __init__(self, debug=False):
    rospy.init_node('drive_square')
    rospy.Subscriber('/odom', Odometry, self.dispatcher)
    self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Potential states of robot
    self.states = {
      "debug": self.debug,
      "driving": self.driving,
      "turning": self.turning
    }

    # Set initial state
    self.state = self.states["driving"] if not debug else self.state["debug"]
    self.start_x = None
    self.start_y = None
    self.start_angle = None

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      r.sleep()

  def dispatcher(self, odom):
    self.state(poseToXYTheta(odom.pose.pose))

  def debug(self, odom):
    print odom

  def switchBehavior(self):
    self.start_x = None
    self.start_y = None
    self.start_angle = None

    if self.state == self.states["driving"]:
      self.state = self.states["turning"]
    else:
      self.state = self.states["driving"]

  def driving(self, odom, sidelength=.5):
    x, y, _ = odom

    if self.start_x is not None and self.start_y is not None :
      dis = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
      if dis < sidelength:
        self.publisher.publish(DIRECTIONS["FORWARD"])
      else:
        self.switchBehavior()
    else:
      self.start_x = x
      self.start_y = y


  def turning(self, odom, turn_angle = math.pi/2, error_margin=0.5):
    _, _, angle = odom

    # Note: the error margin accounts for the fact that the odometry reading
    # is only updated once per four publications, and on top of that, seems to
    # undershoot the actual turn distance. The error margin was calibrated
    # to form a rough square in testing.
    if self.start_angle is not None:
      if diffAngle(angle, self.start_angle) < (turn_angle-error_margin):
        self.publisher.publish(DIRECTIONS["LEFT"])
      else:
        self.switchBehavior()
    else:
      self.start_angle = angle

if __name__ == "__main__":
  d = DriveSquare()
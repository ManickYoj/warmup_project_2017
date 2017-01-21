
#!/usr/bin/env python

import rospy
import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist, Vector3

DIRECTIONS = {
  'FORWARD': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)),
  'BACKWARD': Twist(linear=Vector3(-1, 0, 0), angular=Vector3(0, 0, 0)),
  'LEFT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 1)),
  'RIGHT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -1)),
  'STOP': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
}

KEYMAP = {
 'w' : DIRECTIONS['FORWARD'],
 'a' : DIRECTIONS['LEFT'],
 's' : DIRECTIONS['BACKWARD'],
 'd' : DIRECTIONS['RIGHT'],
 ' ' : DIRECTIONS['STOP'],
}

def getKey(settings=termios.tcgetattr(sys.stdin)):
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  print key
  return key


def run(key=None):
  rospy.init_node('teleop')
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  print "w: forward \na: left \ns:backward \nd:right \nspace: stop"

  while key != '\x03' and not rospy.is_shutdown():
    key = getKey()
    if key in KEYMAP:
      publisher.publish(KEYMAP[key])

if __name__ == "__main__":
  run()
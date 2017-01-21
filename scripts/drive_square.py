
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

DIRECTIONS = {
  'FORWARD': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)),
  'BACKWARD': Twist(linear=Vector3(-1, 0, 0), angular=Vector3(0, 0, 0)),
  'LEFT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 1)),
  'RIGHT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -1)),
  'STOP': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
}

class DriveSquare:
  def __init__(self):
    rospy.init_node('teleop')
    rospy.Subscriber('/odom', Odometry, self.dispatcher)
    self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    self.state = "driving"

    r = rospy.rate(10)
    while not rospy.is_shutdown():
      r.sleep()
      
  def dispatcher(odom):
    if self.state == "driving":
      self.check_odom(odom)
    else:
      self.spin(odom)

  def check_odom(odom):
    pose = odom.pose.pose
    cur_x = pose.position.x
    cur_y = pose.position.y
    
    if not self.start_pos:
      self.start_pos = (pose.position.x, pose.position.y)
      
    start_x, start_y = self.start_pos
    
		if sqrt((cur_x - start_x)^2+ (cur_y - start_y)^2) >= 1:
      self.state = turning
      dispatcher(odom)
      
  	
    self.prev_x = cur_x
    self.prev_y = cur_y
    
  def spin(odom):
    pose = odom.pose.pose
    self.start_pos = (pose.position.x, pose.position.y)
    
    orientation = tf.transformations.euler_from_quaternion(pose.orientation)
  	if not self.target_angle:
      self.target_angle = # TODO
    
    # TODO: Set target angle (and account for angle wrapping) and switch states if achieved
    # Otherwise, publish turn command

if __name__ == "__main__":
  d = DriveSquare()
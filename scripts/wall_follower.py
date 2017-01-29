#!/usr/bin/env python

import rospy, time, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

DIRECTIONS = {
	'FORWARD': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)),
	'FORWARD_LEFT': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 1)),
	'FORWARD_RIGHT': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, -1)),
	'BACKWARD': Twist(linear=Vector3(-1, 0, 0), angular=Vector3(0, 0, 0)),
	'LEFT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 1)),
	'RIGHT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -1)),
	'STOP': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
}

class WallFollower(object):
	def __init__(self):
		rospy.init_node('wall_follower')
		rospy.Subscriber('/stable_scan', LaserScan, self.processScan)
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
		self.minRange = None
		self.lastMinRange = None
		self.ranges = None
		
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			r.sleep()
		
	def processScan(self, scan):
		if self.minRange:
			self.lastMinRange = self.minRange

		# Find minimum range and angle
		self.minRange = {
			'range': float('Inf'),
			'angle': None,
			'time': time.time()
		}

		for angle, range in enumerate(scan.ranges):
			if (range != 0.0) and range < self.minRange['range']:
				self.minRange["range"] = range
				self.minRange["angle"] = angle
			
		# If no angle returns a viable reading, do not change course
		if self.minRange["angle"] is None or self.lastMinRange is None:
			return
			
		# Find difference from last minimum range (velocity away from wall)
		self.velocity = (self.minRange["range"] - self.lastMinRange["range"])/(self.minRange["time"] - self.lastMinRange["time"])
		
		self.controller(self.minRange, self.velocity)
		
	def controller(self, wallRange, wallVelocity, targetDistance = 0.5, errorRange=0.1):
		# If wallRange within target, spin robot to align s.t. wall is off to right
		if math.fabs(wallRange['range'] - targetDistance) < errorRange:
			if wallRange["angle"] <= 90:
				self.publisher.publish(DIRECTIONS["FORWARD_RIGHT"])
			elif wallRange["angle"] <= 180:
				self.publisher.publish(DIRECTIONS["FORWARD_LEFT"])
			elif wallRange["angle"] <= 270:
				self.publisher.publish(DIRECTIONS["FORWARD_RIGHT"])
			elif wallRange["angle"] <= 360:
				self.publisher.publish(DIRECTIONS["FORWARD_LEFT"])
		
		# If outside the target range
		elif wallRange["range"] >= targetDistance + errorRange:
			# If velocity away from wall is negative, keep going
			if self.velocity < 0:
				self.publisher.publish(DIRECTIONS["FORWARD"])
			# If velocity towards wall is negative, turn to face wall more
			else:
				self.publisher.publish(DIRECTIONS["LEFT"])
			
		elif wallRange["range"] <= targetDistance - errorRange:
			if self.velocity > 0:
				self.publisher.publish(DIRECTIONS["FORWARD"])
			else:
				if wallRange["angle"] <= 180:
					self.publisher.publish(DIRECTIONS["FORWARD_RIGHT"])
				else:
					self.publisher.publish(DIRECTIONS["FORWARD_LEFT"])
		
if __name__ == "__main__":
	w = WallFollower()
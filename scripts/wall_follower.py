#!/usr/bin/env python

import rospy, time, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump


# Utility direction definitions
DIRECTIONS = {
	'FORWARD': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)),
	'FORWARD_LEFT': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 1)),
	'FORWARD_RIGHT': Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, -1)),
	'BACKWARD': Twist(linear=Vector3(-1, 0, 0), angular=Vector3(0, 0, 0)),
	'LEFT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 1)),
	'RIGHT': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -1)),
	'STOP': Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
}

# Wall following configuration
INSTRUCTIONS = {
	# Robot is:
	'inRange': {
		# and the wall is to the robot's:
		'leftFront': DIRECTIONS["FORWARD_RIGHT"],
		'rightFront': DIRECTIONS["FORWARD_LEFT"],
		'leftBack': DIRECTIONS["FORWARD_LEFT"],
		'rightBack': DIRECTIONS["FORWARD_RIGHT"]
	},
	'tooClose': {
		# and the wall is to the robot's
		'leftFront': DIRECTIONS["RIGHT"],
		'rightFront': DIRECTIONS["LEFT"],
		'leftBack': DIRECTIONS["FORWARD_LEFT"],
		'rightBack': DIRECTIONS["FORWARD_RIGHT"]
	},
	'tooFar': {
		# and the wall is to the robot's
		'leftFront': DIRECTIONS["FORWARD"],
		'rightFront': DIRECTIONS["FORWARD"],
		'leftBack': DIRECTIONS["LEFT"],
		'rightBack': DIRECTIONS["RIGHT"]
	}
}

class WallFollower(object):
	def __init__(self, debug=False):
		rospy.init_node('wall_follower')
		rospy.Subscriber('/stable_scan', LaserScan, self.processScan)
		rospy.Subscriber('/bump', Bump, self.emergencyStop)
		self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.minRange = None
		self.lastMinRange = None
		self.instruction = DIRECTIONS["FORWARD"]
		self.stop = False
		self.debug = debug

		r = rospy.Rate(40)
		while not rospy.is_shutdown():
			if self.stop:
				self.publisher.publish(DIRECTIONS["STOP"])
			else:
				self.publisher.publish(self.instruction)
			r.sleep()

	def emergencyStop(self, b):
		bumpList = [b.leftFront, b.leftSide, b.rightFront, b.rightSide]
		for reading in bumpList:
			if reading:
				self.stop = True

	def processScan(self, scan):
		"""
		Convert the data from the scan into useful information;
		specifically, find the distance to the closest detected
		object (ideally a wall) and pass its range and angle from
		the robot on to the controller.
		"""

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

		# If no angle returns a viable reading, proceed until a wall is encountered
		if self.minRange["angle"] is None:
			self.instruction = DIRECTIONS['FORWARD']
			return

		# If the node is still starting up, wait until all variables are set
		if self.lastMinRange is None:
			return

		# Find velocity away from wall
		deltaRange = self.minRange["range"] - self.lastMinRange["range"]
		deltaTime = self.minRange["time"] - self.lastMinRange["time"]
		self.velocity = deltaRange / deltaTime

		self.controller(self.minRange, self.velocity)

	def controller(self, wallRange, wallVelocity, targetDistance = 0.70, errorRange=0.10):
		"""
		Based on the data distilled from the scan, decide on the robot's
		next course of action. Save it to an instruction variable for the
		main loop to act on.
		"""
		if wallRange["range"] >= targetDistance + errorRange:
			position = 'tooFar'
		elif wallRange["range"] <= targetDistance - errorRange:
			position = 'tooClose'
		else:
			position = 'inRange'

		if self.debug:
			print position + " " + str(wallRange)

		if wallRange['angle'] <= 90:
			wallQuadrant = 'leftFront'
		elif wallRange['angle'] <= 180:
			wallQuadrant = 'leftBack'
		elif wallRange['angle'] <= 270:
			wallQuadrant = 'rightBack'
		elif wallRange['angle'] <= 360:
			wallQuadrant = 'rightFront'

		self.instruction = INSTRUCTIONS[position][wallQuadrant]

if __name__ == "__main__":
	w = WallFollower(True)
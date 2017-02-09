#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud
from neato_node.msg import Bump
from visualization_msgs.msg import Marker

class Automaton(object):
	def __init__(
		self,
		name="automaton",
		states={},
		initialState=None,
		debug=False,
		markersTopics=[]
	):
		rospy.init_node(name)

		# Setup sensor subscribers
		rospy.Subscriber('/scan', LaserScan, self.onScan)
		rospy.Subscriber('/stable_scan', LaserScan, self.onStableScan)
		rospy.Subscriber('/projected_stable_scan', PointCloud, self.onProjectedScan)
		rospy.Subscriber('/bump', Bump, self.onBump)
		rospy.Subscriber('/odom', Odometry, self.onOdom)

		# Setup command publisher
		self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		# Set up dynamic marker publications
		self.markerPubs = {
			m:
			rospy.Publisher(m, Marker, queue_size=10)
			for m in markersTopics
		}

		self.states = states
		self.state = initialState
		self.debug = debug

		r = rospy.Rate(20)
		while not rospy.is_shutdown():
			instructions = self.states[self.state].update()

			# Execute movement commands
			self.cmd.publish(instructions["command"])

			# Change state if necessary
			if instructions["changeState"] is not None:
				self.state = instructions["changeState"]

			# Publish all markers included in the instructions
			markerData = self.instructions["markers"] or {}
			for markerName in self.markerPubs.keys():
				self.markerPubs[markerName].publish(
					markerData[markerName] if markerName in markerData
					else utils.clearMarkers()
				)

			r.sleep()

	def onBump(self, bump):
		self.states[self.state].onBump(bump)

	def onScan(self, scan):
		self.states[self.state].onScan(scan)

	def onStableScan(self, scan):
		self.states[self.state].onStableScan(scan)


class Behavior(object):
	def __init__(self, name="behavior", debug=False):
		self.debug = debug
		self.stateName = name
		self.linear = 0
		self.angular = 0
		self.markers = {}

	def onBump(self, bump):
		pass

	def onScan(self, scan):
		pass

	def onStableScan(self, scan):
		pass

	def onProjectedScan(self, scan):
		pass

	def onOdom(self, odom):
		pass

	def updateMarker(self, topic, datum):
		self.markers[topic] = datum

	def setLinear(self, linear):
		self.linear = linear

	def setAngular(self, angular):
		self.angular = angular

	def setSpeed(self, linear, angular):
		self.linear = linear
		self.angular = angular

	def changeState(self, stateName):
		self.stateName = stateName
		self.markers = {}

	def update(self):
		return {
			"changeState": self.stateName,
			"command": Twist(
				linear=Vector3(self.linear, 0, 0),
				angular=Vector3(0, 0, self.angular)
			),
			"markers": self.markers,
		}

if __name__ == "__main__":
	pass
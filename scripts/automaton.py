import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump

class Automaton(object):
	def __init__(
		self,
		name="automaton",
		states={},
		initialState=None,
		debug=False
	):

		rospy.Subscriber('/stable_scan', LaserScan, self.onScan)
    rospy.Subscriber('/bump', Bump, self.onBump)
    self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.states = states
		self.state = initialState
		self.debug = debug

		r = rospy.Rate(20)
		while not rospy.is_shutdown():
			instructions = self.states[self.state].update()
			# Publish instructions
			if instructions["changeState"] is not None:
				self.state = instructions["changeState"]
			self.cmd.publish(instructions["command"])
			r.sleep()

	def onBump(self, bump):
		self.states[self.state].onBump(bump)

	def onScan(self, scan):
		self.states[self.state].onScan(scan)


class Behavior(object):
	def __init__(self, name="behavior", debug=False):
		self.debug = debug
		self.stateName = name
		self.linear = 0
		self.angular = 0

	def onBump(self, bump):
		pass

	def onScan(self, scan):
		pass

	def setLinear(self, linear):
		self.linear = linear

	def setAngular(self, angular):
		self.angular = angular

	def setSpeed(self, linear, angular):
		self.linear = linear
		self.angular = angular

	def changeState(self, stateName):
		self.stateName = stateName

	def update(self):
		return {
			"changeState": self.stateName,
			"command": Twist(
				linear=Vector3(self.linear, 0, 0)
				angular=Vector3(0, 0, self.angular)
			)
		}

if __name__ == "__main__":
	pass
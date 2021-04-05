#!/usr/bin/env python3
# -*- coding: utf-8 -*-

""" Comms update ping """

import sys
import rospy
from diagnostic_msgs.msg import DiagnosticStatus

class Ping():
	def __init__(self,rate,recover_time):
		self.r = rate
		self.recover_time = recover_time
		# 0 = OK, 1 = stale, 2 = disconnected
		self.connected = 2
		#self.gs_sub = rospy.Subscriber("/ground_station/status",DiagnosticStatus,self.gsCallback)
		self.ag_pub = rospy.Publisher("/argos/status",DiagnosticStatus,queue_size = 10)
		self.createPing()

	def createPing(self):
		updateRate = rospy.Rate(self.r)
		while not rospy.is_shutdown():
			up = rospy.get_time()
			try:
				# ping
				gs_msg = rospy.wait_for_message("/ground_station/status",DiagnosticStatus,timeout=self.recover_time)
				self.connected = 0
			except:
				# lapsed
				self.connected = 2
				rospy.logwarn("Disconnected from GS")
			down = rospy.get_time()-up
			msg = DiagnosticStatus()
			msg.level = self.connected
			msg.message = str(down)
			msg.name = "Comms Status"
			self.ag_pub.publish(msg)
			updateRate.sleep()



def main(rate,recover_time):
	myPing = Ping(rate,recover_time)

if __name__ == "__main__":
	# hz, ping update
	rate = 10
	# seconds, until loss of comms triggered
	recover_time = 10
	try:
		rospy.init_node("Ping_node",anonymous=True)
		main(rate,recover_time)
	except rospy.ROSInterruptException:
		pass
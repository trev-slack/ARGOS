#!/usr/bin/python3
import rospy
import time
import numpy as np 
from geometry_msgs.msg import Twist, Quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticStatus
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from tf.transformations import *

# path data stack
class PathData():
	def __init__(self,stack_size):
		self.s = stack_size
		self.idx = 0
		self.data = None

	def addData(self,d):
		if self.idx<self.s:
			self.data.append(d)
			self.idx+=1
		else:
			self.data.dequeue()
			self.data.append(d)

	def getData(self):
		if self.idx == 0:
			return None
		self.idx-=1
		d = self.data.pop()
		return d



class PathFollow():
	def __init__(self):
		self.path = PathData(10000)
		self.currentfollow = False
		self.connected = 3
		self.glob_sub = rospy.Subscriber("/argos/odometry/global_filtered",Odometry,self.updateLoc)
		self.connectionSub = rospy.Subscriber("/argos/status",DiagnosticStatus,self.connection)
		self.client = actionlib.SimpleActionClient('/argos/move_base', MoveBaseAction)
		# wait for first diagnostic message
		rospy.wait_for_message("/ground_station/status",DiagnosticStatus,timeout=None)
		rospy.wait_for_message("/argos/odometry/global_filtered",Odometry,timeout=None)
		print("Start-up messages recieved")
		self.pathfollow()


	# main
	def pathfollow(self):
		self.path.data = self.loc
		self.path.addData(self.loc)
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			# disconnected
			if self.connected == 2:
				print("Recovery Mode")
				self.pid()
			else:
				if self.path.idx<2:
					self.path.data = self.loc
					self.path.addData(self.loc)
				else:
					# store path data
					d = math.sqrt((self.path.data[-1][0]-self.loc[0])**2+(self.path.data[-1][1]-self.loc[1])**2)
					if d > 0.5:
						self.path.addData(self.loc)
			r.sleep()


	# update current location
	def updateLoc(self,data):
		quat_inv = quaternion_inverse([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
		self.loc = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,quat_inv[0],quat_inv[1],quat_inv[2],quat_inv[3],self.connected]
		

	# pid control
	def pid(self):
		p = self.path.getData()
		if p:
			goal = MoveBaseGoal()
			# x and y coordinates are flipped? Check with real sensors
			goal.target_pose.pose.position.x = p[1]
			goal.target_pose.pose.position.y = -p[0]
			goal.target_pose.pose.position.z = p[2]
			goal.target_pose.pose.orientation.x = p[3]
			goal.target_pose.pose.orientation.y = p[4]
			goal.target_pose.pose.orientation.z = p[5]
			goal.target_pose.pose.orientation.w = p[6]
			goal.target_pose.header.frame_id = 'odom'
			goal.target_pose.header.stamp = rospy.Time.now()
			print("sent goal: {}".format(goal))
			print(self.loc)
			self.client.send_goal(goal)
			self.client.wait_for_result()
			print("Reached waypoint")



	def connection(self,diag):
		self.connected = diag.level


if __name__=="__main__":
	try:
		rospy.init_node('PATH_FOLLOW_NODE', anonymous=True)
		myMove = PathFollow()
	except rospy.ROSInterruptException:
		pass
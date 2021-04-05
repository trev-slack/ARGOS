#!/usr/bin/python3
import os
os.environ["ROS_NAMESPACE"] = "/argos"
import rospy
import time
import csv
import numpy as np 
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
import actionlib

class PathFollow():
	def __init__(self):
		self.path = np.array([0,0,0,0,0,0,0])
		self.currentfollow = False
		self.connected = 3
		self.checked = False
		self.flag = False
		#tmp connection function
		# self.glob_sub = rospy.Subscriber("/argos/odometry/global_filtered",Odometry,self.pathStore)
		# self.connectionSub = rospy.Subscriber("/argos/connection",DiagnosticStatus,self.connection)
		self.sac = actionlib.SimpleActionClient('/argos/move_base/', MoveBaseAction)
		self.pathfollow()


	# main
	def pathfollow(self):
		self.x = []
		self.y = []
		with open('local_data.txt') as csv_file:
		    csv_reader = csv.reader(csv_file, delimiter=',')
		    line_count = 0
		    for row in csv_reader:
		    	self.x.append(float(row[0]))
		    	self.y.append(float(row[1]))
		print(self.x)
		for i in range(0,len(self.x)):
			try:
				goal = MoveBaseGoal()
				goal.target_pose.pose.position.x = self.x[i]
				goal.target_pose.pose.position.y = self.y[i]
				goal.target_pose.pose.orientation.w = 1
				goal.target_pose.header.frame_id = 'odom'
				goal.target_pose.header.stamp = rospy.Time.now()
				self.sac.send_goal(goal)
				self.sac.wait_for_result()
			except KeyboardInterrupt:
				break

if __name__=="__main__":
	try:
		rospy.init_node('PATH_FOLLOW_NODE', anonymous=True)
		myMove = PathFollow()
	except rospy.ROSInterruptException:
		pass
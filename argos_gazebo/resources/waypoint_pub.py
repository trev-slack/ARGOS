#!/usr/bin/env python
"""publishes a waypoint from a file"""
import rospy
import sys
import actionlib
from openpyxl import load_workbook
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import pandas as pd


def waypoints(num):
	#load in waypoint from file, in position argv num
	path = '/home/cohrint/argos_ws/src/argos_gazebo/resources/waypoints.xlsx'
	df = pd.read_excel(path)
	x = df['x'].iloc[num]
	y = df['y'].iloc[num]
	rospy.loginfo("Waypoint: ({},{})".format(x,y))
	#create message
	dest = MoveBaseGoal()
	dest.target_pose.header.stamp = rospy.Time.now()
	dest.target_pose.pose.position.x = x
	dest.target_pose.pose.position.y = y
	dest.target_pose.pose.orientation.w = 1
	dest.target_pose.header.frame_id = 'argos/odom'
	print(dest)
	#publish
	client = actionlib.SimpleActionClient('argos/move_base',MoveBaseAction)
	client.wait_for_server()
	client.send_goal(dest)



if __name__=="__main__":
	try:
		num = int(sys.argv[1])
		#ROS node
		rospy.init_node('waypoint_node', anonymous=True)
		waypoints(num)
	except rospy.ROSInterruptException:
		pass	
#!/usr/bin/env python
"""stores the time to reach goal"""
import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from openpyxl import workbook #pip install openpyxl
from openpyxl import load_workbook
import os

class reached():
	def __init__(self):
		#start and end time
		self.start = -1
		self.end = -1
		self.reached_main()

	def reached_main(self):
		#waypoint listener
		tmp = rospy.wait_for_message("/argos/move_base/goal", MoveBaseActionGoal,  timeout=None)
		self.start = rospy.get_rostime()
		rospy.loginfo(self.start)
		#result listener
		tmp2 = rospy.wait_for_message("/argos/move_base/result", MoveBaseActionResult,  timeout=None)
		self.end = rospy.get_rostime()
		rospy.loginfo(self.end)
		self.logfile()

	# def callbackGoal(self,data):
	# 	#get time that waypoint message was published
	# 	self.start = rospy.get_rostime()

	# def callbackResult(self,data):
	# 	#get time that result was published
	# 	self.end = rospy.get_rostime()
	# 	self.logfile()

	def logfile(self):
		#write the data to a file
		time = float(self.end.secs)-float(self.start.secs)
		rospy.loginfo(time)
		wb = load_workbook("/home/cohrint/argos_ws/src/argos_gazebo/resources/time_data.xlsx")
		page = wb.active
		page.append([time])
		wb.save("/home/cohrint/argos_ws/src/argos_gazebo/resources/time_data.xlsx") 
		#kill nodes
		os.system("killall -9 gzserver")
		os.system("killall -9 gzclient")
		os.system('killall -9 rosmaster')

if __name__=="__main__":
    try:
        #ROS node
        rospy.init_node('tester_node', anonymous=True)
        checker = reached()
    except rospy.ROSInterruptException:
        pass	
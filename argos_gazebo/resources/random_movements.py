#!/usr/bin/python3
import rospy
import numpy as np 
import time
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

class RandomMove():
	def __init__(self,trials):
		self.trials = trials
		#self.time = time
		self.str1="0,0,0\n"
		self.pub = rospy.Publisher("/argos/argos_velocity_controller/cmd_vel",Twist,queue_size=1)
		self.storeData()
		timer = time.time()
		while time.time()-timer < 100:
			continue
		#self.genPath()


	def genPath(self):
		x_command = np.random.uniform(-0.3,1.75,size=self.trials)
		z_command = np.random.uniform(-1,1,size=self.trials)
		for i in range(0,self.trials):
			start_time = rospy.get_time()
			try:
				while  True:
					current_time = rospy.get_time()
					elapsed_time = current_time-start_time
					if elapsed_time > self.time:
						rospy.loginfo("New Command: {}".format(i))
						break
					command = Twist()
					command.linear.x = x_command[i]
					command.angular.z = z_command[i]
					self.pub.publish(command)
			except KeyboardInterrupt:
				pass

	def storeData(self):
		self.file1 = open('truth_data.txt','w')
		self.file2 = open('local_data.txt','w')
		self.file3 = open('global_data.txt','w')
		self.file4 = open('time_data.txt','w')
		self.truth_sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.saveTruth)
		self.loc_sub = rospy.Subscriber("/argos/odometry/local_filtered",Odometry,self.saveLocal)
		self.glob_sub = rospy.Subscriber("/argos/odometry/global_filtered",Odometry,self.saveGlobal)
		self.clock_sub = rospy.Subscriber('/clock',Clock,self.saveClock)


	def saveTruth(self,truth):
		x = truth.pose[1].position.x
		y = truth.pose[1].position.y
		self.str1 = str(x) + "," + str(y) + "\n"
		#rospy.loginfo(x)

	def saveLocal(self,loc):
		x = loc.pose.pose.position.x
		y = loc.pose.pose.position.y
		t = loc.header.stamp.nsecs
		#rospy.loginfo(x)
		str2 = str(x) + "," + str(y) +"," + str(t)+"\n"
		self.file2.write(str2)		
		self.file1.write(self.str1)

	def saveGlobal(self,glob):
		x = glob.pose.pose.position.x
		y = glob.pose.pose.position.y
		t = glob.header.stamp.nsecs
		#rospy.loginfo(x)
		str3 = str(x) + "," + str(y) +"," + str(t)+"\n"
		self.file3.write(str3)		
		#self.file1.write(self.str1)

	def saveClock(self,clo):
		t = clo.clock.nsecs
		#rospy.loginfo(clo)
		str4 = str(t) + "\n"
		self.file4.write(str4)

if __name__=="__main__":
	try:
		rospy.init_node('ARGOS_RANDOM_MOVE_NODE', anonymous=True)
		myMove = RandomMove(360)
	except rospy.ROSInterruptException:
		pass
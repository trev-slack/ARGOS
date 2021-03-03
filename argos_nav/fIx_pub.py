
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix



if __name__=="__main__":
	rospy.init_node("remapper")
	pub = rospy.Publisher('argos/gps/fix',NavSatFix,queue_size=10)
	rospy.Subscriber('/argos/fix',NavSatFix,callback)
	rospy.spin()

def callback(data):
	pub.publish(data)
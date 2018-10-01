#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts


def callback(data):
	summation = Int16(data.a + data.b)
	rospy.loginfo("The sum of {} and {} is {}".format(str(data.a), str(data.b), str(summation)))
	pub.publish(summation)


def talker():
	rospy.init_node('summation_publisher', anonymous=True)
	global pub
	pub = rospy.Publisher('sum', Int16, queue_size=1)


def listener():
	talker()
	rospy.Subscriber("two_ints", TwoInts, callback)
	rospy.spin()


if __name__ == "__main__":
	listener()
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def sr_result_callback(msg):
    rospy.loginfo("Speech Recognition Result: {}".format(msg.data))

if __name__ == '__main__':
    rospy.init_node('sr_node_test')
    rospy.Subscriber('/sr_result', String, sr_result_callback)
    rospy.spin()

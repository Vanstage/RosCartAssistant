#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

def image_detection_result_callback(msg):
    rospy.loginfo("Image Detection Result: {}".format(msg.data))

if __name__ == '__main__':
    rospy.init_node('image_viewer_test')
    rospy.Subscriber('/image_detection_result', String, image_detection_result_callback)
    rospy.spin()

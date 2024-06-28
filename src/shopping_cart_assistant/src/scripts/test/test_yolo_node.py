#!/usr/bin/env python
import rospy
from yolo_ros_msgs.msg import BoundingBoxes

def bounding_boxes_callback(msg):
    for box in msg.bounding_boxes:
        rospy.loginfo("Detected {} with confidence {}".format(box.Class,box.probability))

if __name__ == '__main__':
    rospy.init_node('yolo_test')
    rospy.Subscriber('/yolo_ros/bounding_boxes', BoundingBoxes, bounding_boxes_callback)
    rospy.spin()

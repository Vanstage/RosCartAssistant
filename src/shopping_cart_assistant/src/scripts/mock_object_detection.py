#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time

def mock_object_detection():
    rospy.init_node('mock_object_detection', anonymous=True)
    pub = rospy.Publisher('result', String, queue_size=10)

    while not rospy.is_shutdown():
        # Simulate an object detection event
        rospy.sleep(10)  # Wait for 10 seconds before detecting an object
        pub.publish("object detected")
        rospy.sleep(5)  # Wait for 5 seconds before detecting the next object

if __name__ == '__main__':
    try:
        mock_object_detection()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from darknet_ros.darknet_ros_msgs.msg import BoundingBoxes # Replace with actual YOLO detection message type

class YoloImageNameExtractor:
    def __init__(self):
        rospy.init_node('extract_yolo_image_names')

        # Parameters and topic names
        self.input_topic = rospy.get_param('~input_yolo_topic', '/yolo_ros/output_topic')
        self.output_topic = rospy.get_param('~output_image_name_topic', '/yolo_image_names')

        # Subscribers and publishers
        self.subscriber = rospy.Subscriber(self.input_topic, BoundingBoxes, self.callback)
        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=10)

    def callback(self, msg):
        # Assuming YOLODetection message contains image names or IDs
        image_names = []  # Placeholder for extracted image names

        for detection in msg.detections:
            image_names.append(detection.image_name)  # Modify this based on actual YOLO detection message structure

        # Publish extracted image names
        image_names_msg = String()
        image_names_msg.data = ','.join(image_names)
        self.publisher.publish(image_names_msg)

if __name__ == '__main__':
    try:
        YoloImageNameExtractor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

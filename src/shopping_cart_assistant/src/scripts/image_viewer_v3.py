#!/usr/bin/env python

import cv2
import rospy
from sensor_msgs.msg import Image
from darknet_ros.darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class ImageViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
        self.obj_pub = rospy.Publisher('input', String, queue_size=10)
        self.cv_image = None
        self.detected_objects = []

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        if self.cv_image is not None:
            # Display the image with rectangles
            for obj in self.detected_objects:
                x, y, w, h, label = obj
                cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(self.cv_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow("Image with Rectangles", self.cv_image)
            cv2.waitKey(1)

    def bounding_boxes_callback(self, data):
        self.detected_objects = []
        for box in data.bounding_boxes:
            x = box.xmin
            y = box.ymin
            w = box.xmax - box.xmin
            h = box.ymax - box.ymin
            label = box.Class
            self.detected_objects.append((x, y, w, h, label))

        # Publish detected objects to the 'input' topic
        detected_labels = [label for _, _, _, _, label in self.detected_objects]
        if detected_labels:
            rospy.loginfo("Detected objects: %s", detected_labels)
            self.obj_pub.publish(str(detected_labels))

if __name__ == '__main__':
    rospy.init_node('image_viewer', anonymous=True)
    iv = ImageViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

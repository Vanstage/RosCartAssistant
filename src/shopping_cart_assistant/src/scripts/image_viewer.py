#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from yolo_ros_msgs.msg import BoundingBoxes



class ImageViewer:
    def __init__(self):
        rospy.init_node('image_viewer', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.obj_sub = rospy.Subscriber("/yolo_ros/bounding_boxes", BoundingBoxes, self.object_detection_callback)
        self.obj_pub = rospy.Publisher('input', String, queue_size=10)
        self.detected_objects = []

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
       
        # Process image here if needed, e.g., drawing rectangles and labels

    def object_detection_callback(self, data):
        self.detected_objects = []
        
        for box in data.bounding_boxes:
            x = box.xmin
            y = box.ymin
            w = box.xmax - box.xmin
            h = box.ymax - box.ymin
            label = box.Class
            self.detected_objects.append((x, y, w, h, label))
        
        # Publish detected object labels as a single string
        detected_labels = [label for (_, _, _, _, label) in self.detected_objects]
        if detected_labels:
            rospy.loginfo("Detected objects: %s", detected_labels)
            label_string = ", ".join(detected_labels)
            self.obj_pub.publish(label_string)
        else:
            rospy.loginfo("No objects detected.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    iv = ImageViewer()
    iv.run()

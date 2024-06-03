#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Example bounding box coordinates obtained from YOLO
        bounding_boxes = [(100, 100, 50, 50), (200, 200, 30, 30)]  # Example coordinates

        # Draw rectangles on the image for each bounding box
        for box in bounding_boxes:
            x, y, w, h = box
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle with thickness 2

        # Display the image with rectangles
        cv2.imshow("Image with Rectangles", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('image_viewer', anonymous=True)
    iv = ImageViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

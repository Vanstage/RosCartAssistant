#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from yolo_ros_msgs.msg import BoundingBoxes
from price_list import price_dict



class ImageViewer:
    def __init__(self):
        rospy.init_node('image_viewer', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.obj_sub = rospy.Subscriber("/yolo_ros/bounding_boxes", BoundingBoxes, self.object_detection_callback)
        self.obj_pub = rospy.Publisher('input', String, queue_size=10)
        self.cart_sub = rospy.Subscriber("/cart", String, self.cart_callback)
       
        self.detected_objects = []
        self.no_object_timer = None
        self.no_object_timeout = rospy.Duration(10)  # Timeout duration in seconds
        self.price_dict = price_dict
        self.cart_contents = ""
        self.cart_total = "Total: $0"

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
       
        except CvBridgeError as e:
            rospy.loginfo("Error".format(cv_image.shape))
            rospy.logerr(e)


        if self.detected_objects:
            for (x, y, w, h, label) in self.detected_objects:
                # Draw bounding box
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                
                    price = self.get_price(label)
                    if price == "unknown":
                        text = "{}: ${:.2f}".format(label, price)
                    else:
                        text = "{}: ${:.2f}".format(label, price)
                        cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)


        # Assuming cv2 is imported and cv_image is defined
        total_text = self.cart_total
       

        # Calculate total_position
        total_position = (10, 40)  # Adjust x and y 
        cv2.putText(cv_image, total_text, total_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        if self.cart_contents:
            if self.cart_total != "$0":
                cv2.putText(cv_image, "Cart Contents:", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                items = self.cart_contents.split(',')
                index = 0
                for item in items:
                    item_position = (20, 100 + index * 20)  # Adjust y position for each item
                    cv2.putText(cv_image, "- " + item.strip(), item_position, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
                    index += 1


        self.display_image(cv_image)
       

    def cart_callback(self, data):
        self.cart_contents = data.data
        rospy.loginfo("Cart contents received: %s", self.cart_contents)

        # Calculate total price when cart contents change
        if self.cart_contents:
            items = self.cart_contents.split(',')
            total_price = 0.0
            for item in items:
                total_price += self.price_dict.get(item, 0)
            self.cart_total = "Total: ${:.2f}".format(total_price) if total_price > 0 else "$0"
        else:
            self.cart_total = "Total: $0"

    def display_image(self, image):
        cv2.imshow("Image with Bounding Boxes", image)
        cv2.waitKey(1)  # Refresh display

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
            label_string = ",".join(detected_labels)
            self.obj_pub.publish(label_string)
            detected_labels = []
            # Cancel any existing no object timer
            if self.no_object_timer:
                self.no_object_timer.shutdown()
            self.no_object_timer = None
        else:
            rospy.loginfo("No objects detected.")
            if not self.no_object_timer:
                self.no_object_timer = rospy.Timer(self.no_object_timeout, self.no_object_timeout_callback)



    def get_price(self, label):
        # Example function to fetch price based on label
        return self.price_dict.get(label, "unknown")

    def no_object_timeout_callback(self, event):
        rospy.loginfo("No objects detected laaaa.")
        self.obj_pub.publish("No objects detected.")
        self.no_object_timer = None  # Reset the timer

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    iv = ImageViewer()
    try:
        iv.run()
    finally:
        cv2.destroyAllWindows()  # Close OpenCV windows properly

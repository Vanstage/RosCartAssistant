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
        rospy.init_node('image_viewer', anonymous=True)  # Initialize ROS node
        self.bridge = CvBridge()  # Initialize CvBridge for converting ROS Image messages to OpenCV images
        # Subscribe to image and object detection topics
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.obj_sub = rospy.Subscriber("/yolo_ros/bounding_boxes", BoundingBoxes, self.object_detection_callback)
        # Publish detected object labels and cart total to a topic
        self.obj_pub = rospy.Publisher('/image_detection_result', String, queue_size=10)
        # Subscribe to cart content topic
        self.cart_sub = rospy.Subscriber("/cart_content", String, self.cart_callback)
       
        # Initialize variables for detected objects, cart contents, and cart total
        self.detected_objects = []
        self.no_object_timer = None
        self.no_object_timeout = rospy.Duration(20)  # Timeout duration in seconds if no object detected
        self.price_dict = price_dict
        self.cart_contents = ""
        self.cart_total = "Total: $0"
        self.no_response_timeout = rospy.Duration(300) 
        self.no_response_timer = None  

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Convert ROS Image to OpenCV format
        except CvBridgeError as e:
            rospy.loginfo("Error converting ROS Image: {}".format(e))
       
        # Draw bounding boxes and display prices of detected objects
        if self.detected_objects:
            for (x, y, w, h, label) in self.detected_objects:
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Draw bounding box
                price = self.get_price(label)
                if "unknown" == price:
                    text = "{}: ${}".format(label, price)
                else:
                    text = "{}: ${:.2f}".format(label, price)
                    cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        # Display cart total and contents on the image
        total_text = self.cart_total
        total_position = (10, 40)  # Adjust position of total text
        cv2.putText(cv_image, total_text, total_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        
        if self.cart_contents:
            if self.cart_total != "Total: $0":
                cv2.putText(cv_image, "Cart Contents:", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                items = self.cart_contents.split(',')
                index = 0
                for item in items:
                    item_position = (20, 100 + index * 20)  # Adjust position of each item in the cart
                    cv2.putText(cv_image, "- " + item.strip(), item_position, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
                    index += 1

        self.display_image(cv_image)  # Display the image with annotations

    def cart_callback(self, data):
        self.cart_contents = data.data  # Update cart contents
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
        cv2.imshow("Cart Assistant", image)  # Display the image with bounding boxes
        cv2.waitKey(1)  # Refresh display

    def object_detection_callback(self, data):
        self.detected_objects = []
        
        # Process bounding boxes detected by YOLO
        for box in data.bounding_boxes:
            x = box.xmin
            y = box.ymin
            w = box.xmax - box.xmin
            h = box.ymax - box.ymin
            label = box.Class
            if "person" in label:
                continue
            if label in price_dict:
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
            if not self.no_response_timer:
                self.no_response_timer = rospy.Timer(self.no_response_timeout, self.no_respond_timeout_callback)    

    def get_price(self, label):
        return self.price_dict.get(label, "unknown")  # Retrieve price from price dictionary

    def no_object_timeout_callback(self, event):
        rospy.loginfo("No objects detected.")
        self.obj_pub.publish("No objects detected.")
        self.check_input_timer()

    def no_respond_timeout_callback(self, event):
        rospy.loginfo("Thank you for using me")
        self.obj_pub.publish("Thank you for using me")
        self.check_input_timer()
        self.no_response_timer.shutdown()
        self.no_response_timer = None  # Reset the timer

    def check_input_timer(self):
        if self.no_object_timer:
            # Shutdown existing input timer if exist
            self.no_object_timer.shutdown()  
            # Reset input timer 
            self.no_object_timer = None  
    def run(self):
        rospy.spin()  # ROS main loop

if __name__ == '__main__':
    iv = ImageViewer()
    try:
        iv.run()  # Run the image viewer node
    finally:
        cv2.destroyAllWindows()  # Close OpenCV windows properly

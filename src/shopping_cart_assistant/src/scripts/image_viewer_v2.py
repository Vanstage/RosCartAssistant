#!/usr/bin/env python
import cv2
from builtins import int # type: ignore
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class ImageViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.obj_pub = rospy.Publisher('input', String, queue_size=10)

        # Load YOLO model
        self.net = cv2.dnn.readNet("model/yolov3.weights", "model/yolov3.cfg")
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        with open("model/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Perform object detection
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Information to display on the screen
        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * cv_image.shape[1])
                    center_y = int(detection[1] * cv_image.shape[0])
                    w = int(detection[2] * cv_image.shape[1])
                    h = int(detection[3] * cv_image.shape[0])
                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        detected_objects = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                detected_objects.append(label)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the image with rectangles
        cv2.imshow("Image with Rectangles", cv_image)
        cv2.waitKey(1)

        # Publish detected objects to the 'input' topic
        if detected_objects:
            rospy.loginfo("Detected objects: %s", detected_objects)
            self.obj_pub.publish(str(detected_objects))


if name == '__main__':
    rospy.init_node('image_viewer', anonymous=True)
    iv = ImageViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()